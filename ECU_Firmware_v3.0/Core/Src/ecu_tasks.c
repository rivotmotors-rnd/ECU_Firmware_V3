/**
 ******************************************************************************
 * @file    ecu_tasks.c
 * @brief   Vehicle Application & Telemetry RTOS Tasks
 *
 * @details Architectural Context:
 *          This module decouples vehicle state management and periodic
 * operational tasks from low-level peripheral initialization:
 *          1. Non-Blocking Telemetry: `send_heartbeat_frame()` atomically
 * copies vehicle status into the interrupt-driven circular buffer
 * (`tx_byte_queue`) and arms the UART TXE interrupt without freezing the RTOS
 * scheduler.
 *          2. Vehicle Lighting: Blinks indicators at 300ms intervals using RTOS
 *             tick counting, monitors hazard switches, and handles the PC8
 *             10-press emergency MCU shutdown trigger.
 *          3. Brake Monitor: Samples ADC channel 4 (Hall Effect brake sensor)
 *             with hysteresis thresholds to reliably detect brake pulls and
 *             control the rear Tail Lamp output (PA6).
 *          4. VcuCommandDispatcherTask: Executes actions based on verified
 *             commands delivered over UART from the Android VCU.
 ******************************************************************************
 */

#include "ecu_tasks.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "ecu_actuators.h"
#include "ecu_uart.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================= */
/* VEHICLE STATE VARIABLES                                                   */
/* ========================================================================= */
volatile uint8_t turn_signal_status = 0;
volatile uint8_t turn_signal_status_1 = 0;
volatile uint8_t handle_lock_state = 0;
volatile uint8_t seat_lock_state = 0;
volatile uint8_t led_w1_state = 0;
volatile uint8_t custom_blink_target = 0; // blink for lock(1)and unlock(2)
uint32_t cycle_count = 0;

/* ========================================================================= */
/* EXTERNAL HARDWARE & RTOS HANDLES                                          */
/* ========================================================================= */
extern osMessageQueueId_t vcuCommandQueue;
extern osMessageQueueId_t actuatorCommandQueue;
extern ADC_HandleTypeDef hadc1;

#define TX_BYTE_QUEUE_SIZE 8192
extern uint8_t tx_byte_queue[TX_BYTE_QUEUE_SIZE];
extern volatile uint16_t tx_byte_head;
extern volatile uint16_t tx_byte_tail;
extern void kick_tx(void);

/* ========================================================================= */
/* 1. VCU COMMAND DISPATCHER TASK                                            */
/* ========================================================================= */
/**
 * @brief Processes incoming commands dispatched from the VCU (via UART parser).
 *        It waits on `vcuCommandQueue`. Depending on the command type (0x01,
 * 0x07), it either sends a lock command to `actuatorCommandQueue` or switches
 *        the 12V Aux / 12V VCU MOSFETs.
 * @param argument Unused FreeRTOS argument.
 */
void VcuCommandDispatcherTask(void *argument) {
  ControlCmd_t cmd;
  osDelay(10);
  printf("[TASK] VCU Dispatcher Started\r\n");
  for (;;) {
    // 1. Wait indefinitely for a parsed command from the UART parser
    if (osMessageQueueGet(vcuCommandQueue, &cmd, NULL, osWaitForever) == osOK) {
      // 2. We only process Write/Set commands (State 0x01)
      if (cmd.state != 0x01) {
        continue;
      }

      // 3. Route based on the Task ID
      switch (cmd.type) {
      case 0x01: { // Handle Lock Task
        uint8_t requested_state = (cmd.data[0] == 0x00) ? 0 : 1;
        // Edge detection: Only trigger if the requested state is different!
        if (requested_state != handle_lock_state) {
          LockCmd_t lockCmd;
          lockCmd.command = requested_state;
          printf("[VCU] Handle Lock Command: %s\r\n",
                 (requested_state == 0) ? "LOCK" : "UNLOCK");

          /* ====================================================================
           * VEHICLE LOCK/UNLOCK LIGHTING FEEDBACK
           * --------------------------------------------------------------------
           * When the vehicle is Locked or Unlocked, we want to flash the
           * indicators (like a car) to give the user visual confirmation.
           *   - Lock   (requested_state == 0) -> Flashes 1 time
           *   - Unlock (requested_state == 1) -> Flashes 2 times
           * ====================================================================
           */
          extern uint32_t
              cycle_count; /* Tracks how many times the lights have blinked */

          /* Reset the blink counter to 0 so it starts fresh every time.
           * CRITICAL: Do NOT overwrite the blinker if it is already flashing 
           * for a critical hardware fault (like the 10-flash contactor weld alert)!
           */
          if (turn_signal_status != 4 || custom_blink_target < 10) {
            cycle_count = 0;
            custom_blink_target = (requested_state == 0) ? 1 : 2;
            turn_signal_status = 4;
          }
          // Push to the Actuator queue
          osMessageQueuePut(actuatorCommandQueue, &lockCmd, 0, 0);
          handle_lock_state = requested_state;
        }
      } break;

      case 0x02: { // Seat Unlock Task
        // Time-based debouncing: The VCU might spam 0x01 without ever sending a
        // 0x00 reset. We use a 2-second cooldown to ignore the spam but still
        // allow repeated unlocks later.
        if (cmd.data[0] == 0x01) {
          static uint32_t last_seat_pop_time = 0;
          if (HAL_GetTick() - last_seat_pop_time > 2000) {
            printf("[VCU] Seat Unlock Command Received!\r\n");
            LockCmd_t seatCmd;
            seatCmd.command = 2;
            if (osMessageQueuePut(actuatorCommandQueue, &seatCmd, 0, 0) ==
                osOK) {
              seat_lock_state = 1;
            }
            last_seat_pop_time = HAL_GetTick();
          }
        }
      } break;

      case 0x07: // 12V Aux Switching Task (Boost Converter)
        // Edge detection: Only trigger if the state actually changes
        if (cmd.data[0] != led_w1_state) {
          if (cmd.data[0] == 0x01) {
            printf("[VCU] 12V Aux Power & Boost Converter turned ON\r\n");
            HAL_GPIO_WritePin(SWITCHING_12V_PORT, SWITCHING_12V, GPIO_PIN_SET);
            led_w1_state = 1;
          } else {
            printf("[VCU] 12V Aux Power & Boost Converter turned OFF\r\n");
            HAL_GPIO_WritePin(SWITCHING_12V_PORT, SWITCHING_12V,
                              GPIO_PIN_RESET);
            led_w1_state = 0;
          }
        }
        break;

      default:
        break;
      }
    }
  }
}

/* ========================================================================= */
/* 2. TELEMETRY HEARTBEAT TASK                                               */
/* ========================================================================= */
/**
 * @brief Constructs the 12-byte telemetry heartbeat payload and delegates
 *        transmission to `send_to_vcu()` to inform the VCU of the ECU's current
 * status. The payload is mapped according to the VCU Protocol Specification.
 * @param status Current state of the turn signals (0=Off, 1=Left, 2=Right,
 * 3=Hazard).
 */
void send_heartbeat_to_vcu(uint8_t status) {
  // 1. Create a blank 12-byte array for our payload
  uint8_t payload[12];
  memset(payload, 0, 12);

  // 2. Map our variables to the exact bytes specified in the protocol document

  /* Byte 0: Head Lock Status */
  payload[0] = handle_lock_state;

  /* Byte 1: Seat Lock Status */
  payload[1] = seat_lock_state;

  /* Bytes 2-8: Reserved/Unused (Side support, Battery, Temp, Gesture) */

  /* Byte 9: Headlight Status */
  if (HAL_GPIO_ReadPin(HIGH_BEAM_PORT, HIGH_BEAM_PIN) == GPIO_PIN_SET) {
    payload[9] = 0x01;
  }

  /* Byte 10: Turn Signal Status (0=Off, 1=Left, 2=Right, 3=Hazard) */
  payload[10] = status;

  /* Byte 11: LED W1 Status (12V Aux) */
  payload[11] = led_w1_state;

  // 3. Hand the 12-byte payload over to the UART builder to package and send!
  /* State=0x03, Task=0x01 */
  send_to_vcu(FRAME_STATE, FRAME_TASK, payload, 12);
}

/**
 * @brief Periodic RTOS task that continuously fires the heartbeat frame
 *        every 1000ms (1Hz) to maintain connectivity with the VCU.
 * @param argument Unused FreeRTOS argument.
 */
extern IWDG_HandleTypeDef hiwdg;

void TelemetryHeartbeatTask(void *argument) {
  osDelay(20);
  printf("[TASK] Telemetry Heartbeat Started\r\n");
  for (;;) {
    send_heartbeat_to_vcu(turn_signal_status_1);

    /* Pet the Watchdog (IWDG) every 1000ms.
     * If this task stops running, the MCU will reset in 3 seconds. */
    HAL_IWDG_Refresh(&hiwdg);

    osDelay(1000);
  }
}

/* ========================================================================= */
/* 3. VEHICLE LIGHTING TASK                                                  */
/* ========================================================================= */
/**
 * @brief A state machine task that controls vehicle lighting based on inputs.
 *        It checks physical toggle switches (Left/Right/Off/Hazard) and
 *        handles the 500ms blink rate for indicators. It also includes an
 *        emergency power-off sequence triggered by the HIGH_BEAM pin.
 * @param argument Unused FreeRTOS argument.
 */
void VehicleLightingTask(void *argument) {
  osDelay(30);
  printf("[TASK] Vehicle Lighting Started\r\n");
  uint8_t prev_status = 0xFF;
  uint8_t saved_turn_signal = 0;
  uint32_t blink_timer = 0;
  uint8_t blink_state = 0;
  // uint32_t cycle_count = 0;
  uint8_t left_prev = GPIO_PIN_SET;
  uint8_t right_prev = GPIO_PIN_SET;
  uint8_t pb2_prev = GPIO_PIN_SET;

  uint8_t pc8_prev = HAL_GPIO_ReadPin(HIGH_BEAM_PORT, HIGH_BEAM_PIN);
  uint8_t pc8_press_count = 0;
  uint32_t pc8_last_press_time = 0;

  for (;;) {
    uint32_t now = osKernelGetTickCount();

    uint8_t left_inp =
        HAL_GPIO_ReadPin(LEFT_INDICATOR_INP_PORT, LEFT_INDICATOR_INP);
    uint8_t right_inp =
        HAL_GPIO_ReadPin(RIGHT_INDICATOR_INP_PORT, RIGHT_INDICATOR_INP);
    uint8_t off_inp =
        HAL_GPIO_ReadPin(OFF_INDICATOR_INP_PORT, OFF_INDICATOR_INP);
    uint8_t haz_inp = HAL_GPIO_ReadPin(HAZARD_INP_PORT, HAZARDAS);
    uint8_t pc8 = HAL_GPIO_ReadPin(HIGH_BEAM_PORT, HIGH_BEAM_PIN);
    if (off_inp == GPIO_PIN_RESET && pb2_prev == GPIO_PIN_SET) {
      printf("[GPIO] Indicator Switch Canceled (OFF)\r\n");
      turn_signal_status = 0;
      cycle_count = 0;
    }
    pb2_prev = off_inp;

    if (haz_inp == GPIO_PIN_RESET) {
      if (turn_signal_status != 3) {
        printf("[GPIO] Hazard Switch turned ON\r\n");
        saved_turn_signal = turn_signal_status;
        turn_signal_status = 3;
      }
    } else {
      if (turn_signal_status == 3) {
        printf("[GPIO] Hazard Switch turned OFF\r\n");
        turn_signal_status = saved_turn_signal;
      }
    }

    if (turn_signal_status != 3) {
      if (left_inp == GPIO_PIN_RESET && left_prev == GPIO_PIN_SET) {
        printf("[GPIO] Left Indicator Switch turned ON\r\n");
        turn_signal_status = 1;
        cycle_count = 0;
      }
      left_prev = left_inp;
      if (right_inp == GPIO_PIN_RESET && right_prev == GPIO_PIN_SET) {
        printf("[GPIO] Right Indicator Switch turned ON\r\n");
        turn_signal_status = 2;
        cycle_count = 0;
      }
      right_prev = right_inp;
    }

    switch (turn_signal_status) {
    case 0:
      turn_signal_status_1 = 0;
      blink_state = 0;
      cycle_count = 0;
      HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                        GPIO_PIN_RESET);
      HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                        GPIO_PIN_RESET);
      break;
    case 1:
      if (cycle_count >= 50) {
        turn_signal_status = 0;
        break;
      }
      if (now - blink_timer >= 333) {
        blink_timer = now;
        blink_state = !blink_state;
        if (blink_state) {
          HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                            GPIO_PIN_SET);
          turn_signal_status_1 = 1;
        } else {
          HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                            GPIO_PIN_RESET);
          turn_signal_status_1 = 0;
          cycle_count++;
        }
        HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                          GPIO_PIN_RESET);
      }
      break;
    case 2:
      if (cycle_count >= 50) {
        turn_signal_status = 0;
        break;
      }
      if (now - blink_timer >= 333) {
        blink_timer = now;
        blink_state = !blink_state;
        if (blink_state) {
          HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                            GPIO_PIN_SET);
          turn_signal_status_1 = 2;
        } else {
          HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                            GPIO_PIN_RESET);
          turn_signal_status_1 = 0;
          cycle_count++;
        }
        HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                          GPIO_PIN_RESET);
      }
      break;
    case 3:
      if (now - blink_timer >= 333) {
        blink_timer = now;
        blink_state = !blink_state;
        if (blink_state) {
          HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                            GPIO_PIN_SET);
          HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                            GPIO_PIN_SET);
          turn_signal_status_1 = 3;
        } else {
          HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                            GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                            GPIO_PIN_RESET);
          turn_signal_status_1 = 0;
        }
      }
      break;
    /* ====================================================================
     * STATE 4: CUSTOM AUTO-CANCELING HAZARD (LOCK/UNLOCK FEEDBACK)
     * --------------------------------------------------------------------
     * This state is triggered by the VCU Command Dispatcher. It acts exactly
     * like the normal Hazard lights (flashing both left and right
     * simultaneously), but it automatically turns itself off (reverts to state
     * 0) once it reaches the `custom_blink_target`.
     * ==================================================================== */
    case 4:
      /* Step 1: Check if we have finished blinking the required number of times
       */
      if (cycle_count >= custom_blink_target) {
        turn_signal_status = 0; /* Auto-cancel: Return to OFF state */
        break;
      }

      /* Step 2: Toggle the lights every 333 milliseconds */
      if (now - blink_timer >= 333) {
        blink_timer = now;
        blink_state = !blink_state; /* Flip state: ON -> OFF or OFF -> ON */

        if (blink_state) {
          /* TURN ON: Power both Left and Right indicators */
          HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                            GPIO_PIN_SET);
          HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                            GPIO_PIN_SET);

          /* Inform the VCU via Telemetry that Hazards (State 3) are active */
          turn_signal_status_1 = 3;
        } else {
          /* TURN OFF: Cut power to both Left and Right indicators */
          HAL_GPIO_WritePin(LH_INDICATOR_SIG_OP_PORT, LH_INDICATOR_SIG_OP,
                            GPIO_PIN_RESET);
          HAL_GPIO_WritePin(RH_INDICATOR_SIG_OP_PORT, RH_INDICATOR_SIG_OP,
                            GPIO_PIN_RESET);

          /* Inform the VCU via Telemetry that indicators are off */
          turn_signal_status_1 = 0;

          /* CRITICAL: A full "blink" is complete only after the lights turn
           * OFF. We increment our counter here to track how many full blinks
           * have occurred. */
          cycle_count++;
        }
      }
      break;
    }

    if (turn_signal_status_1 != prev_status) {
      send_heartbeat_to_vcu(turn_signal_status_1);
      prev_status = turn_signal_status_1;
    }

    /* ---- PC8: High-beam edge detection + 10-press power off ---- */
    uint8_t pc8_old = pc8_prev;
    if (pc8 != pc8_old) {
      if (pc8 == GPIO_PIN_SET) {
        printf("[GPIO] Headlight (High Beam) turned ON\r\n");
      } else {
        printf("[GPIO] Headlight (High Beam) turned OFF\r\n");
      }
      send_heartbeat_to_vcu(turn_signal_status_1);
    }

    if (pc8 == GPIO_PIN_SET && pc8_old == GPIO_PIN_RESET) {
      if ((now - pc8_last_press_time) > 100) {    // Debounce 100ms
        if ((now - pc8_last_press_time) > 1500) { // Timeout 1.5s
          pc8_press_count = 0;
        }
        pc8_press_count++;
        pc8_last_press_time = now;

        if (pc8_press_count == 10) {
          printf("\r\n[EMERGENCY] 10-Press Detected! Shutting down VCU and "
                 "Resetting MCU...\r\n");
          // Turn off 12V VCU MOSFET (active-low: RESET = OFF)
          HAL_GPIO_WritePin(VCU_SIG_OP_PORT, VCU_SIG_OP, GPIO_PIN_RESET);
          osDelay(2000);
          __disable_irq();
          NVIC_SystemReset();
        }
      }
    }
    pc8_prev = pc8;
    osDelay(10);
  }
}

/* ========================================================================= */
/* 4. SEAT UNLOCK BUTTON TASK                                                */
/* ========================================================================= */
/**
 * @brief Monitors a physical push-button on the vehicle to release the seat.
 *        Implements debouncing and a long-press requirement (3000ms) before
 *        sending a command to `actuatorCommandQueue` to pop the seat lock.
 * @param argument Unused FreeRTOS argument.
 */
void SeatUnlockButtonTask(void *argument) {
  osDelay(40);
  printf("[TASK] Seat Unlock Button Started\r\n");
  uint32_t press_start_time = 0;
  uint8_t is_pressing = 0, action_triggered = 0;
  for (;;) {
    uint8_t button = HAL_GPIO_ReadPin(BUTTON_LOCK_PORT, BUTTON_LOCK_PIN);
    if (button == GPIO_PIN_RESET) {
      if (is_pressing == 0) {
        is_pressing = 1;
        press_start_time = osKernelGetTickCount();
        action_triggered = 0;
      } else {
        if (!action_triggered &&
            (osKernelGetTickCount() - press_start_time) >= 500) {
          if (led_w1_state == 1) {
            printf("[GPIO] Physical Seat Unlock Button Pressed!\r\n");
            LockCmd_t seatCmd;
            seatCmd.command = 2;
            osMessageQueuePut(actuatorCommandQueue, &seatCmd, 0, 0);
            seat_lock_state = 1;
            send_heartbeat_to_vcu(turn_signal_status_1);
            seat_lock_state = 0;
          }
          action_triggered = 1;
        }
      }
    } else {
      is_pressing = 0;
      action_triggered = 0;
    }
    osDelay(50);
  }
}

/* ========================================================================= */
/* 5. BRAKE MONITOR TASK                                                     */
/* ========================================================================= */
/**
 * @brief Monitors the analog voltage from the brake lever's Hall Effect sensor.
 *        The diodes (D1, D2) OR the left/right brake signals into PA4 (ADC).
 *        When the voltage crosses BRAKE_THRESHOLD_ON, we consider the brake
 * "Pulled" and turn on the physical Tail Lamp (PA6).
 */
void BrakeMonitorTask(void *argument) {
  osDelay(50);
  printf("[TASK] Brake Monitor Started\r\n");
  uint16_t adc_value = 0;
  uint8_t output_state = 0, high_count = 0, low_count = 0;
  HAL_GPIO_WritePin(TAIL_LAMP_SIG_OP_PORT, TAIL_LAMP_SIG_OP, GPIO_PIN_RESET);

  /* Debug removed: Baseline confirmed (Released=~490, Pulled=~1800) */

  for (;;) {
    HAL_ADC_Start(&hadc1);

    uint32_t start_tick = osKernelGetTickCount();
    HAL_StatusTypeDef status;
    do {
      status = HAL_ADC_PollForConversion(&hadc1, 0);
      if (status == HAL_OK)
        break;
      osDelay(1);
    } while (osKernelGetTickCount() - start_tick < 2);

    if (status == HAL_OK) {
      adc_value = HAL_ADC_GetValue(&hadc1);
      HAL_ADC_Stop(&hadc1);

      if (output_state == 0) {
        if (adc_value > BRAKE_THRESHOLD_ON) {
          high_count++;
          low_count = 0;
          if (high_count >= 3) {
            output_state = 1;
            printf("[BRAKE] Pulled! (ADC: %d)\r\n", adc_value);
            HAL_GPIO_WritePin(TAIL_LAMP_SIG_OP_PORT, TAIL_LAMP_SIG_OP,
                              GPIO_PIN_SET);
            high_count = 0;
          }
        } else {
          high_count = 0;
        }
      } else {
        if (adc_value < BRAKE_THRESHOLD_OFF) {
          low_count++;
          high_count = 0;
          if (low_count >= 3) {
            output_state = 0;
            printf("[BRAKE] Released (ADC: %d)\r\n", adc_value);
            HAL_GPIO_WritePin(TAIL_LAMP_SIG_OP_PORT, TAIL_LAMP_SIG_OP,
                              GPIO_PIN_RESET);
            low_count = 0;
          }
        } else {
          low_count = 0;
        }
      }
    } else {
      HAL_ADC_Stop(&hadc1);
    }

    osDelay(50);
  }
}
