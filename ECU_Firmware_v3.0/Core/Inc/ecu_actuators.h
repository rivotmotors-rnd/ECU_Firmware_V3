/**
 ******************************************************************************
 * @file    ecu_actuators.h
 * @brief   Header for Vehicle Actuators Subsystem (Handle Lock & Seat Lock)
 *
 * @details This module manages the physical locking actuators of the vehicle:
 *          1. Handlebar Steering Lock (PC2): Controlled via a custom
 * single-wire pulse-width modulation protocol (bit-banged using ARM Cortex-M
 * DWT).
 *          2. Seat Latch Solenoid (PC9): Triggered via a timed pulse sequence.
 *
 *          All physical actuations are executed in a dedicated, queued RTOS
 * task
 *          (`ActuatorControlTask`) to prevent blocking other vehicle telemetry.
 ******************************************************************************
 */

#ifndef ECU_ACTUATORS_H
#define ECU_ACTUATORS_H

#include "ecu_types.h"
#include "stm32f4xx_hal.h"

#include "ecu_config.h"

/* ========================================================================= */
/* FUNCTION PROTOTYPES                                                       */
/* ========================================================================= */

/**
 * @brief  Initializes the Data Watchpoint and Trace (DWT) cycle counter.
 * @note   Used for precise microsecond pulse generation in waveform generation.
 */
void DWT_Init(void);

/**
 * @brief  Transmits the physical pulse sequence to lock the handlebar.
 */
void send_handle_lock(void);

/**
 * @brief  Transmits the physical pulse sequence to unlock the handlebar.
 */
void send_handle_unlock(void);

/**
 * @brief  Transmits the physical pulse sequence to unlatch the seat solenoid.
 */
void send_lock_signal_pc9(void);

/**
 * @brief  FreeRTOS task that processes incoming lock commands from
 * `actuatorCommandQueue`.
 * @param  argument: Unused.
 */
void ActuatorControlTask(void *argument);

#endif /* ECU_ACTUATORS_H */
