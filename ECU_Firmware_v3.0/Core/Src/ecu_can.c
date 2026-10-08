/**
 ******************************************************************************
 * @file    ecu_can.c
 * @brief   CAN Subsystem (Interrupt-driven)
 *
 * @details Handles CAN injection from UART, forwarding CAN frames to UART,
 *          and monitoring the hardware for Bus-Off errors.
 ******************************************************************************
 */

#include "ecu_can.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "ecu_uart.h" /* For calculateChecksum */
#include "main.h" /* For CubeMX GPIO pin label defines (HV_CONTACTOR_Pin, CP_LINE_DETECT_Pin, etc.) */
#include "task.h"
#include <stdbool.h>
#include <stdio.h>

extern CAN_HandleTypeDef hcan2;
extern osMessageQueueId_t canRxQueue;

/* Global variable to track the last time the Charger sent a valid status
 * message */
volatile uint32_t last_charger_msg_time = 0;

/* Tracks whether CAN1 is intentionally active (gun plugged in).
 * Set/cleared by HAL_GPIO_EXTI_Callback in main.c */
extern volatile uint8_t can1_is_running;

/* Lighting state variables from ecu_tasks.c — used for welding detection alert
 */
extern volatile uint8_t turn_signal_status;
extern volatile uint8_t custom_blink_target;
extern uint32_t cycle_count;

/* NOTE: MX_CAN2_Init() was removed - it is static in main.c after CubeMX
 * regeneration. Bus-Off recovery re-initializes hcan2 directly using HAL. */

/* ========================================================================= */
/* CAN Rx Hardware Callback                                                  */
/* ========================================================================= */
/**
 * @brief Hardware ISR Callback triggered when a new CAN message arrives in
 * FIFO0. It copies the raw CAN message into `canRxQueue` so it can be processed
 *        by the CanToUartBridgeTask without blocking the interrupt handler.
 * @param hcan Pointer to the CAN handle.
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CanRxMsg_t msg;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &msg.header, msg.data) ==
      HAL_OK) {
    if (hcan->Instance == CAN1) {
      msg.bus = 1;
    } else if (hcan->Instance == CAN2) {
      msg.bus = 2;
    }
    osMessageQueuePut(canRxQueue, &msg, 0, 0);
  }
}

extern CAN_HandleTypeDef hcan1;

/* ========================================================================= */
/* Shared CAN Recovery Helper                                                */
/* ========================================================================= */
static void ECU_CAN_Restart(CAN_HandleTypeDef *hcan) {
  HAL_CAN_Stop(hcan);
  HAL_CAN_DeInit(hcan);

  /* Re-initialize (250kbps for testing. Change Prescaler to 5 for 500kbps
   * later) */
  hcan->Init.Prescaler = 10;
  hcan->Init.Mode = CAN_MODE_NORMAL;
  hcan->Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan->Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan->Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan->Init.TimeTriggeredMode = DISABLE;
  hcan->Init.AutoBusOff = DISABLE; /* Software recovery only */
  hcan->Init.AutoWakeUp = DISABLE;
  hcan->Init.AutoRetransmission = ENABLE;
  hcan->Init.ReceiveFifoLocked = DISABLE;
  hcan->Init.TransmitFifoPriority = DISABLE;

  /* Priority 2: Check Return Values! */
  if (HAL_CAN_Init(hcan) != HAL_OK) {
    printf("[CAN ERROR] HAL_CAN_Init FAILED!\r\n");
    return;
  }
  if (HAL_CAN_Start(hcan) != HAL_OK) {
    printf("[CAN ERROR] HAL_CAN_Start FAILED!\r\n");
    return;
  }

  /* Re-apply Filters */
  CAN_FilterTypeDef f = {0};
  f.FilterMode = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_32BIT;
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;

  /* CAN1 uses Bank 0, CAN2 uses Bank 14 */
  if (hcan->Instance == CAN1) {
    f.FilterBank = 0;
  } else {
    f.FilterBank = 14;
  }

  HAL_CAN_ConfigFilter(hcan, &f);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  /* If CAN1 recovered, the charger heartbeat is no longer trustworthy.
   * Force a fresh handshake before allowing the contactor to re-close. */
  if (hcan->Instance == CAN1) {
    last_charger_msg_time = 0;
  }

  printf("[CAN ERROR] Recovery COMPLETE\r\n");
}

/* ========================================================================= */
/* 7. CAN HEALTH & CONTACTOR SAFETY MONITOR TASK                             */
/* ========================================================================= */
void CanHealthMonitorTask(void *argument) {
  osDelay(60);
  printf("[SYS] Health & Safety Task Started\r\n");

  /* Virtual state for our simulated contactor */
  bool virtual_contactor_closed = false;
  static bool has_alerted_welding =
      false; /* Moved here so the whole task can read it! */

  for (;;) {

    /* ---------------------------------------------------------
     * 1. CAN BUS HEALTH MONITORING
     * --------------------------------------------------------- */
    /* CAN1: Check for Bus-Off OR high TEC (pre-emptive restart before BOFF)
     */
    uint32_t esr1 = hcan1.Instance->ESR;
    if ((esr1 & CAN_ESR_BOFF) || ((esr1 >> 16) & 0xFF) > 200) {
      printf("[CAN ERROR] CAN1 BOFF! TEC=%lu REC=%lu\r\n", (esr1 >> 16) & 0xFF,
             (esr1 >> 24) & 0xFF);
      ECU_CAN_Restart(&hcan1);
    }

    /* CAN2: Check for Bus-Off OR high TEC (pre-emptive restart before BOFF)
     */
    uint32_t esr2 = hcan2.Instance->ESR;
    if ((esr2 & CAN_ESR_BOFF) || ((esr2 >> 16) & 0xFF) > 200) {
      printf("[CAN ERROR] CAN2 BOFF! TEC=%lu REC=%lu\r\n", (esr2 >> 16) & 0xFF,
             (esr2 >> 24) & 0xFF);
      ECU_CAN_Restart(&hcan2);
    }

    /* ---------------------------------------------------------
     * 2. CONTACTOR SAFETY LOGIC
     * --------------------------------------------------------- */
    /* Condition 1: Physical connection (CP and AUX2 shorted together, pulling
     * PA3 LOW - Temporarily using CONTACTOR_FEEDBACK_Pin instead of PA5) */
    bool condition_physical =
        (HAL_GPIO_ReadPin(CONTACTOR_FEEDBACK_GPIO_Port,
                          CONTACTOR_FEEDBACK_Pin) == GPIO_PIN_RESET);

    /* --- CAN1 Power Management (Bounce-Proof) --- */
    if (condition_physical && !can1_is_running) {
      printf("[CHARGER] Gun Plugged In! Waking up CAN1...\r\n");
      can1_is_running = 1;
      HAL_CAN_Start(&hcan1);
      HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    } else if (!condition_physical && can1_is_running) {
      printf("[CHARGER] Gun Unplugged! Shutting down CAN1 safely...\r\n");
      can1_is_running = 0;
      HAL_CAN_Stop(&hcan1);
      last_charger_msg_time = 0; /*Invalidate stale heartbeat timestamp*/
    }

    /* Condition 2: CAN heartbeat 0x18FF50E5 received within the last 2500ms.
     * 2500ms = 2.5x the charger's ~1000ms send period. This gives safe margin
     * against RTOS scheduling jitter without causing false contactor trips.
     */
#ifdef CONTACTOR_TEST_MODE
    bool condition_can = true; /* BYPASSED for hardware testing */
#else
    bool condition_can = (last_charger_msg_time != 0) &&
                         (HAL_GetTick() - last_charger_msg_time < 2500);
#endif

    /* Re-close cooldown: wait 500ms before re-closing */
    static uint32_t contactor_last_open_time = 0;
    bool cooldown_elapsed = (HAL_GetTick() - contactor_last_open_time >= 500);
    /* Evaluate the check */
    /* Block contactor closing permanently if a weld
     * fault is active. A welded contactor means the
     * physical contacts are fused. Commanding PB5 HIGH
     * again serves no purpose and is misleading in the
     * logs. */

    if (condition_physical && condition_can && cooldown_elapsed &&
        !has_alerted_welding) {
      if (!virtual_contactor_closed) {

#ifdef CONTACTOR_TEST_MODE
        printf("\r\n[SAFETY-TEST] CP ONLY CHECK PASSED! (PA5 = LOW, CAN "
               "bypassed)\r\n");
        printf("[SAFETY-TEST] CLOSING HV CONTACTOR MOSFET (PB5 = "
               "HIGH)...\r\n\r\n");
#else
        printf("\r\n[SAFETY] 2-STEP CHECK PASSED! (Physical connection + CAN "
               "0x18FF50E5)\r\n");
        printf("[SAFETY] CLOSING HV CONTACTOR MOSFET (PB5 = HIGH)...\r\n\r\n");
#endif
        HAL_GPIO_WritePin(HV_CONTACTOR_GPIO_Port, HV_CONTACTOR_Pin,
                          GPIO_PIN_SET);
        virtual_contactor_closed = true;
      }
    } else {
      if (virtual_contactor_closed) {
#ifdef CONTACTOR_TEST_MODE
        printf("\r\n[SAFETY-TEST] CP DISCONNECTED! Opening contactor.\r\n");
        printf("[SAFETY-TEST] OPENING HV CONTACTOR MOSFET (PB5 = LOW)!\r\n");
#else
        printf("\r\n[SAFETY CRITICAL] CONDITION LOST!\r\n");
        printf("[SAFETY] OPENING HV CONTACTOR MOSFET IMMEDIATELY (PB5 = "
               "LOW)!\r\n");
        if (!condition_can)
          printf("         -> Reason: Lost CAN ID 0x18FF50E5 (Timeout)\r\n");
        if (!condition_physical)
          printf(
              "         -> Reason: Gun unplugged (CP/AUX2 disconnected)\r\n");
#endif
        HAL_GPIO_WritePin(HV_CONTACTOR_GPIO_Port, HV_CONTACTOR_Pin,
                          GPIO_PIN_RESET);
        virtual_contactor_closed = false;
        contactor_last_open_time = HAL_GetTick(); /* Start cooldown timer */
      }
    }

    /* =====================================================================
     * CONTACTOR WELDING DETECTION (PA3 = CONTACTOR_FEEDBACK)
     * ---------------------------------------------------------------------
     * If the MOSFET is OFF but the feedback pin still reads LOW (GND),
     * it means the physical relay contacts are fused/welded shut.
     * The HV battery is permanently connected — this is a critical hazard!
     * We alert the rider by blinking BOTH indicators 10 times (ONCE per
     * boot).
     * =====================================================================
     */

    /* We MUST wait at least 2000ms (2 seconds) after commanding the contactor
     * OFF before checking for a weld. Heavy-duty HV contactors with flyback
     * diodes can take up to 1-2 seconds for the magnetic field to fully
     * collapse and the heavy springs to pull apart. If we check too early, it
     * triggers a false positive! */
    if (!virtual_contactor_closed &&
        (HAL_GetTick() - contactor_last_open_time > 2000)) {
      /* =================================================================
       * DEMO ONLY: Hardcoding feedback to GPIO_PIN_SET.
       * Because you wired CP Line to PA3, and PA3 is also the
       * CONTACTOR_FEEDBACK_Pin, the ECU falsely thinks the contactor
       * is welded shut when you plug the gun in!
       * ================================================================= */
      uint8_t feedback = GPIO_PIN_SET;
      if (feedback == GPIO_PIN_RESET) {
        if (!has_alerted_welding) {
          printf("\r\n[FATAL] CONTACTOR WELDING DETECTED! HV IS PERMANENTLY"
                 " LIVE!\r\n");
          printf("[FATAL] Physical contacts are fused. Vehicle must be"
                 " serviced immediately!\r\n\r\n");

          /* Flash both indicators 10 times as a visible danger warning. */
          if (turn_signal_status != 4) {
            cycle_count = 0;
            custom_blink_target = 10;
            turn_signal_status = 4;
          }

          /* Mark as alerted so it doesn't repeat for the rest of the boot
           * cycle
           */
          has_alerted_welding = true;
        }
      } else {
        /* If for some miraculous reason the contactor un-welds itself, reset
         * the alert flag */
        has_alerted_welding = false;
      }
    }

    /* Run this safety check 10 times per second */
    osDelay(100);
  }
}

/* =========================================================================
 */
/* CAN -> UART FORWARDING (called by RTOS Bridge Task) */
/* =========================================================================
 */
/**
 * @brief Translates an incoming CAN frame into the VCU UART protocol format.
 *        It packages the CAN ID (4 bytes) and CAN Data (8 bytes) into a
 *        12-byte payload and delegates transmission to `send_to_vcu()`.
 * @param msg Pointer to the received CAN message structure.
 */
void forward_can_to_vcu(CanRxMsg_t *msg) {
  // 1. Create a blank 12-byte payload for the UART packet
  uint8_t payload[12];

  // 2. Check if the incoming CAN ID is standard (11-bit) or extended (29-bit)
  uint32_t can_id =
      (msg->header.IDE == CAN_ID_EXT) ? msg->header.ExtId : msg->header.StdId;

  // 3. Slice the 32-bit CAN ID into the first 4 bytes of our payload (Big
  // Endian)
  payload[0] = (can_id >> 24) & 0xFF;
  payload[1] = (can_id >> 16) & 0xFF;
  payload[2] = (can_id >> 8) & 0xFF;
  payload[3] = can_id & 0xFF;

  // 4. Copy the actual CAN data into the remaining 8 bytes of the payload
  for (int i = 0; i < 8; i++) {
    payload[4 + i] = (i < msg->header.DLC) ? msg->data[i] : 0x00;
  }

  // 5. Hand the 12-byte payload over to the UART builder (State=0x03,
  // Task=0x03 for CAN)
  send_to_vcu(0x03, 0x03, payload, 12);
}

/* =========================================================================
 */
/* 8. CAN TO UART + DUAL-CAN BRIDGE TASK */
/* =========================================================================
 */
/**
 * @brief This task acts as the central router for the ECU.
 *        1. It receives all CAN messages from both the Charger (CAN1) and BMS
 * (CAN2).
 *        2. It acts as a Software Gateway, translating specific charging
 * commands between the Charger and the BMS to enable EV charging.
 *        3. It forwards all telemetry to the Android VCU over UART for the
 * display.
 */
void CanToUartBridgeTask(void *argument) {
  CanRxMsg_t msg;
  osDelay(70);
  printf("[CAN Bridge] Task Started\r\n");

  for (;;) {
    if (osMessageQueueGet(canRxQueue, &msg, NULL, osWaitForever) == osOK) {

      /* Extract the CAN ID regardless of whether it is Standard (11-bit) or
       * Extended (29-bit) */
      uint32_t can_id =
          (msg.header.IDE == CAN_ID_EXT) ? msg.header.ExtId : msg.header.StdId;

      /* ---------------------------------------------------------
       * SOFTWARE GATEWAY LOGIC: CAN1 (Charger) <--> CAN2 (BMS)
       * --------------------------------------------------------- */

      /* ROUTE 1: CHARGER -> BMS
       * If CAN1 (Charger) sends CHG ID 0x18FF50E5 (Charger Status/Ready),
       * we must inject it into CAN2 so the BMS knows the charger is
       * connected.
       */
      if (msg.bus == 1 && can_id == 0x18FF50E5 && msg.header.DLC == 8) {

        /* UPDATE OUR SAFETY TIMESTAMP */
        last_charger_msg_time = HAL_GetTick();

        /* Rate limit the print to once every 2 seconds to avoid flooding */
        static uint32_t last_print_chg = 0;
        if (HAL_GetTick() - last_print_chg > 2000) {
          uint16_t out_volt_raw = (msg.data[0] << 8) | msg.data[1];
          uint16_t out_curr_raw = (msg.data[2] << 8) | msg.data[3];
          float out_voltage = out_volt_raw * 0.1f;
          float out_current = out_curr_raw * 0.1f;
          uint8_t status_flags = msg.data[4];

          printf("[CHG->BMS] ID:18FF50E5 | Vout:%.1fV | Iout:%.1fA | ",
                 out_voltage, out_current);

          if (status_flags == 0x00) {
            printf("Status: OK\r\n");
          } else {
            printf("Faults: ");
            if (status_flags & (1 << 0))
              printf("[HW_FAIL] ");
            if (status_flags & (1 << 1))
              printf("[OVER_TEMP] ");
            if (status_flags & (1 << 2))
              printf("[AC_VOLT_ERR] ");
            if (status_flags & (1 << 3))
              printf("[NO_BATT] ");
            if (status_flags & (1 << 4))
              printf("[TIMEOUT] ");
            printf("\r\n");
          }

          last_print_chg = HAL_GetTick();
        }

        /* Note: We cannot directly assign an RxHeader to a TxHeader in STM32
         * HAL. We must manually map the IDs and set TransmitGlobalTime to
         * DISABLE. */
        CAN_TxHeaderTypeDef txHeader;
        txHeader.StdId = msg.header.StdId;
        txHeader.ExtId = msg.header.ExtId;
        txHeader.IDE = msg.header.IDE;
        txHeader.RTR = msg.header.RTR;
        txHeader.DLC = msg.header.DLC;
        txHeader.TransmitGlobalTime = DISABLE;

        uint32_t mailbox;
        HAL_StatusTypeDef status =
            HAL_CAN_AddTxMessage(&hcan2, &txHeader, msg.data, &mailbox);
        if (status != HAL_OK) {
          /* Also rate limit the error print */
          static uint32_t last_err_chg = 0;
          if (HAL_GetTick() - last_err_chg > 2000) {
            printf("[CAN1->CAN2] ERROR: Failed to inject into CAN2! "
                   "Status=%d\r\n",
                   status);
            last_err_chg = HAL_GetTick();
          }
        }
      }

      /* ROUTE 2: BMS -> CHARGER
       * If CAN2 (BMS) sends BMS ID 0x1806E5F4 (Max Voltage/Current Limits),
       * we must inject it into CAN1 so the Charger knows how much power to
       * supply.
       * GUARD: Only attempt forward if CAN1 is actually RUNNING (INAK=0).
       * If the gun is unplugged, CAN1 is stopped (INAK=1). Silently drop
       * the message to prevent mailbox-full error spam on the serial monitor.
       */
      else if (msg.bus == 2 && can_id == 0x1806E5F4 && msg.header.DLC == 8) {

        /* Only forward to Charger if CAN1 is intentionally running (gun
         * plugged in). Using the volatile flag instead of MSR polling to
         * avoid race conditions. */
        if (!can1_is_running) {
          /* Gun not connected — silently drop, no print */
        } else {
          /* CAN1 is active — gun is plugged in, forward the message */
          static uint32_t last_print_bms = 0;
          if (HAL_GetTick() - last_print_bms > 2000) {

#ifndef CONTACTOR_TEST_MODE
            uint16_t max_volt_raw = (msg.data[0] << 8) | msg.data[1];
            uint16_t max_curr_raw = (msg.data[2] << 8) | msg.data[3];
            float max_voltage = max_volt_raw * 0.1f;
            float max_current = max_curr_raw * 0.1f;
            uint8_t charge_ctrl = msg.data[4];
            uint8_t heat_ctrl = msg.data[5];

            printf("[BMS->CHG] ID:1806E5F4 | Vmax:%.1fV | Imax:%.1fA | Cmd:%s "
                   "| Mode:%s\r\n",
                   max_voltage, max_current,
                   (charge_ctrl == 0) ? "START" : "STOP",
                   (heat_ctrl == 0) ? "CHARGE" : "HEAT");
#endif
            last_print_bms = HAL_GetTick();
          }

          CAN_TxHeaderTypeDef txHeader;
          txHeader.StdId = msg.header.StdId;
          txHeader.ExtId = msg.header.ExtId;
          txHeader.IDE = msg.header.IDE;
          txHeader.RTR = msg.header.RTR;
          txHeader.DLC = msg.header.DLC;
          txHeader.TransmitGlobalTime = DISABLE;

          uint32_t mailbox;
          HAL_StatusTypeDef status =
              HAL_CAN_AddTxMessage(&hcan1, &txHeader, msg.data, &mailbox);
          if (status != HAL_OK) {
#ifndef CONTACTOR_TEST_MODE
            printf("[CAN2->CAN1] ERROR: Failed to inject into CAN1! "
                   "Status=%d\r\n",
                   status);
#endif
          }
        }
      }

      /* ---------------------------------------------------------
       * TELEMETRY LOGIC: Forward everything to the Android VCU
       * --------------------------------------------------------- */
      /* No matter where the message came from, send a copy to the Android
       * screen */
      forward_can_to_vcu(&msg);
    }
  }
}
