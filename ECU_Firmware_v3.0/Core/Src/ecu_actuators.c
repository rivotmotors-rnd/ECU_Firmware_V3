/**
 ******************************************************************************
 * @file    ecu_actuators.c
 * @brief   Actuator Control Implementation (Handle Lock & Seat Lock)
 *
 * @details Architectural Context:
 *          Physical actuators require precise timing waveforms to trigger
 * mechanical solenoids and smart locks. In the original prototype, these
 * routines were tightly coupled inside main.c.
 *
 *          Key architectural elements in this module:
 *          1. DWT Cycle Counter: The ARM Cortex-M4 Data Watchpoint and Trace
 * (DWT) unit provides sub-microsecond timing accuracy (`delay_us`), which is
 * far more accurate than software loops and does not monopolize a hardware
 * timer.
 *          2. RTOS Cooperative Yielding: During the microsecond delays,
 * `taskYIELD()` is called so higher-priority real-time tasks (like CAN bridge
 * or UART reception) are never starved while waiting for pulse transitions.
 *          3. Event-Driven Execution: The `ActuatorControlTask` blocks on
 *             `actuatorCommandQueue` and consumes 0% CPU until a command
 * arrives from the VCU or physical button.
 *          4. State Synchronization: Once an actuation completes, a heartbeat
 * frame is automatically dispatched to inform the telematics unit/VCU of the
 * new state.
 ******************************************************************************
 */

#include "ecu_actuators.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include <stdio.h>
#include "ecu_tasks.h"

/* ========================================================================= */
/* EXTERNAL DEPENDENCIES & OS HANDLES                                        */
/* ========================================================================= */
extern osMessageQueueId_t actuatorCommandQueue;
extern volatile uint8_t turn_signal_status_1;
extern volatile uint8_t seat_lock_state;

/* send_heartbeat_to_vcu() is declared in ecu_tasks.h (already included above) */

/* ========================================================================= */
/* LOW-LEVEL TIMING (DWT Hardware Cycle Counter)                            */
/* ========================================================================= */

/**
 * @brief  Enables the Cortex-M DWT CYCCNT register for accurate microsecond
 * counting.
 */
void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief Helper function for precise microsecond delays.
 *        Uses the DWT (Data Watchpoint and Trace) cycle counter for
 * high-resolution blocking delays needed during bit-banged protocols.
 * @param us The number of microseconds to delay.
 */
static inline void delay_us(uint32_t us) {
  uint32_t start = DWT->CYCCNT;
  uint32_t cycles = us * (SystemCoreClock / 1000000U);
  while ((DWT->CYCCNT - start) < cycles) {
    taskYIELD();
  }
}

/* ========================================================================= */
/* BIT-BANGING PROTOCOL FOR HANDLE LOCK (PC2)                                */
/* ========================================================================= */

/**
 * @brief Transmits a logical '1' bit for the Handle Lock protocol.
 *        Pattern: HIGH for 3000us, LOW for 1000us.
 */
static void send_bit_1(void) {
  HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_SET);
  delay_us(3000);
  HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_RESET);
  delay_us(1000);
}

/**
 * @brief Transmits a logical '0' bit for the Handle Lock protocol.
 *        Pattern: HIGH for 1000us, LOW for 3000us.
 */
static void send_bit_0(void) {
  HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_SET);
  delay_us(1000);
  HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_RESET);
  delay_us(3000);
}

/**
 * @brief Transmits a full 8-bit byte using the handle lock timing protocol.
 * @param data The byte to transmit.
 */
static void send_byte(uint8_t data) {
  for (int i = 7; i >= 0; i--) {
    if (data & (1U << i))
      send_bit_1();
    else
      send_bit_0();
  }
}

/**
 * @brief Executes the 'Lock' sequence for the handle lock via bit-banging.
 *        Protocol: HIGH for 5ms, then 3 repeat packets of 0xAA 0xAA.
 */
void send_handle_lock(void) {
  HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_SET);
  osDelay(5);
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN,
                      GPIO_PIN_RESET);
    osDelay(4);
    send_byte(0xAA);
    send_byte(0xAA);
    HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_SET);
    if (i < 2)
      osDelay(12);
  }
}

/**
 * @brief Executes the 'Unlock' sequence for the handle lock via bit-banging.
 *        Protocol: HIGH for 5ms, then 3 repeat packets of 0x55 0x55.
 */
void send_handle_unlock(void) {
  HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_SET);
  osDelay(5);
  for (int i = 0; i < 3; i++) {
    HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN,
                      GPIO_PIN_RESET);
    osDelay(4);
    send_byte(0x55);
    send_byte(0x55);
    HAL_GPIO_WritePin(HNDLLOCK_SIG_OP_PORT, HNDLLOCK_SIG_OP_PIN, GPIO_PIN_SET);
    if (i < 2)
      osDelay(12);
  }
}

/* ========================================================================= */
/* SEAT LOCK PULSE SEQUENCE (PC9)                                            */
/* ========================================================================= */

/**
 * @brief Executes the physical seat unlock bit-banged sequence on PC9.
 *        This sequence involves specific microsecond high/low pulsing to
 *        trigger the seat latch solenoid.
 */
void send_lock_signal_pc9(void) {
  HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN, GPIO_PIN_RESET);
  delay_us(2000);
  HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN, GPIO_PIN_SET);
  delay_us(8000);
  for (int i = 0; i < 8; i++) {
    HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN,
                      GPIO_PIN_RESET);
    delay_us(3000);
    HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN, GPIO_PIN_SET);
    delay_us(1000);
  }
  for (int i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN,
                      GPIO_PIN_RESET);
    delay_us(1000);
    HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN, GPIO_PIN_SET);
    delay_us(3000);
    HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN,
                      GPIO_PIN_RESET);
    delay_us(3000);
    HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN, GPIO_PIN_SET);
    delay_us(1000);
  }
  HAL_GPIO_WritePin(SEATLOCK_SIG_OP_PORT, SEATLOCK_SIG_OP_PIN, GPIO_PIN_RESET);
}

/* ========================================================================= */
/* 9. ACTUATOR CONTROL TASK                                                  */
/* ========================================================================= */
/**
 * @brief RTOS Task that listens on `actuatorCommandQueue` for hardware lock
 * triggers. Because the lock protocols require strict microsecond timing, this
 * task is kept separate from the main VCU dispatcher to prevent blocking.
 * @param argument Unused FreeRTOS argument.
 */
void ActuatorControlTask(void *argument) {
  LockCmd_t cmd;
  osDelay(80);
  printf("[TASK] Actuator Control Started\r\n");
  for (;;) {
    // 1. Wait here indefinitely using 0% CPU until a lock command arrives
    if (osMessageQueueGet(actuatorCommandQueue, &cmd, NULL, osWaitForever) ==
        osOK) {
      // 2. Execute the required blocking bit-bang sequence
      switch (cmd.command) {
      case 0:
        printf("[ACTUATOR] Firing Handle LOCK waveform...\r\n");
        send_handle_lock();
        break;
      case 1:
        printf("[ACTUATOR] Firing Handle UNLOCK waveform...\r\n");
        send_handle_unlock();
        break;
      case 2:
        printf("[ACTUATOR] Firing Seat Release waveform...\r\n");
        send_lock_signal_pc9();
        break;
      default:
        break;
      }

      // 3. Physical action is complete → inform the ESP / telematics unit of the new state
      send_heartbeat_to_vcu(turn_signal_status_1);

      // 4. Reset the momentary seat latch state
      if (cmd.command == 2) {
        seat_lock_state = 0; 
      }
    }
  }
}
