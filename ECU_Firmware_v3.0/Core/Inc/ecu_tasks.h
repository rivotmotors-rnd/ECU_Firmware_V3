/**
 ******************************************************************************
 * @file    ecu_tasks.h
 * @brief   Header for High-Level Vehicle Logic & Telemetry Tasks
 *
 * @details Governs the high-level operational tasks of the ECU:
 *          - VcuCommandDispatcherTask: Central dispatcher for vehicle commands
 *          - TelemetryHeartbeatTask: 1Hz telemetry frame generator for VCU
 *          - VehicleLightingTask: Indicator light state machine & PC8 power-off
 *          - SeatUnlockButtonTask: Hardware momentary seat-release push button
 *          - BrakeMonitorTask: Analog Hall Effect ADC sampling & tail lamp control
 ******************************************************************************
 */

#ifndef ECU_TASKS_H
#define ECU_TASKS_H

#include "ecu_types.h"
#include "stm32f4xx_hal.h"

/* ========================================================================= */
/* HARDWARE PIN & THRESHOLD DEFINITIONS                                      */
/* ========================================================================= */
#include "ecu_config.h"

#define BRAKE_THRESHOLD_ON 1200
#define BRAKE_THRESHOLD_OFF 800

/* ========================================================================= */
/* TELEMETRY CONSTANTS (VCU Heartbeat Protocol)                              */
/* ========================================================================= */
#define FRAME_HEADER_0 0xFF
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0x77
#define FRAME_STATE 0x03
#define FRAME_TASK 0x01
#define FRAME_LEN_HIGH 0x00
#define FRAME_LEN_LOW 0x15

/* ========================================================================= */
/* GLOBAL VEHICLE STATE VARIABLES (Shared across modules)                    */
/* ========================================================================= */
extern volatile uint8_t turn_signal_status;
extern volatile uint8_t turn_signal_status_1;
extern volatile uint8_t handle_lock_state;
extern volatile uint8_t seat_lock_state;
extern volatile uint8_t led_w1_state;


/* ========================================================================= */
/* FUNCTION PROTOTYPES                                                       */
/* ========================================================================= */

/**
 * @brief  Builds and enqueues a 21-byte vehicle status packet to the UART TX
 * queue.
 * @param  status: Current active flasher/indicator lamp status.
 */
void send_heartbeat_to_vcu(uint8_t status);

/**
 * @brief  Task worker for incoming high-level control commands (12V switch,
 * locks).
 */
void VcuCommandDispatcherTask(void *argument);

/**
 * @brief  Periodic 1Hz telemetry task that sends the vehicle heartbeat frame.
 */
void TelemetryHeartbeatTask(void *argument);

/**
 * @brief  State machine for turn indicators, hazard lights, and PC8 power off.
 */
void VehicleLightingTask(void *argument);

/**
 * @brief  Monitors physical push button for seat latch release.
 */
void SeatUnlockButtonTask(void *argument);

/**
 * @brief  Monitors digital Push-to-Off brake switch (PA0), drives Tail Lamp
 *         (PA6) and generates motor controller Regen DAC voltage (PA4).
 */
void BrakeMonitorTask(void *argument);

#endif /* ECU_TASKS_H */

