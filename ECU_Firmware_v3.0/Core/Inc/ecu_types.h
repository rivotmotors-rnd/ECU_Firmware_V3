/**
 ******************************************************************************
 * @file    ecu_types.h
 * @brief   Global Data Structures for the RIVOT ECU
 *
 * @details This file acts as the central "dictionary" for the entire project.
 *          It defines the exact shape of the data packets that are passed
 *          between different RTOS tasks and hardware interrupts.
 *          By keeping these structures here, any .c file (like ecu_uart.c or
 *          ecu_can.c) can include this header and safely communicate.
 ******************************************************************************
 */

#ifndef ECU_TYPES_H
#define ECU_TYPES_H

#include "stm32f4xx_hal.h"

/* ========================================================================= */
/* 1. UART PARSING STRUCTURES (From VCU to ECU)                              */
/* ========================================================================= */

/**
 * @brief  Raw UART Buffer Frame
 * @note   Used by the DMA to grab large chunks of raw bytes from the Android
 *         VCU before they are sorted. Matches the RX_SIZE definition.
 */
typedef struct {
  uint16_t length;
  uint8_t data[64];
} UartRxFrame_t;

/**
 * @brief  Parsed ECU Command Frame
 * @note   Once the raw bytes are sorted, they are placed into this structured
 *         format so the ECU knows exactly what task to perform and what data
 *         belongs to it.
 */
typedef struct {
  uint8_t fh[3];   // Frame Header (e.g., 0xFF, 0xAA, 0x55)
  uint8_t state;   // Current vehicle state or command state
  uint8_t task;    // The specific subsystem this command targets
  uint8_t len[2];  // Length of the incoming payload
  uint8_t data[64]; // The actual payload instructions
} rcv_frame;

/* ========================================================================= */
/* 2. CAN BUS STRUCTURES (From Motor to ECU)                                 */
/* ========================================================================= */

/**
 * @brief  Hardware CAN Reception Message
 * @note   When the CAN interrupt fires, it extracts the hardware header (ID)
 *         and the 8 data bytes and safely packages them into this struct so
 *         FreeRTOS can process it without losing data.
 */
typedef struct {
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint8_t bus; // 1 = CAN1 (Charger), 2 = CAN2 (BMS/Motor/OBC)
} CanRxMsg_t;


/* ========================================================================= */
/* 3. RTOS INTERNAL QUEUE STRUCTURES (Task-to-Task Communication)            */
/* ========================================================================= */

/**
 * @brief  General RTOS Control Command
 * @note   Passed into `controlQueue`. Tasks use this to command other tasks
 *         to do work (like turning on a light or changing a state).
 */
typedef struct {
  uint8_t state;   // 0x01 = Write/Set, 0x02 = Read/Get, 0x03 = Heartbeat
  uint8_t type;    // Type of control action requested (Task)
  uint8_t data[8]; // Accompanying data for the action
} ControlCmd_t;

/**
 * @brief  Physical Lock Actuator Command
 * @note   Passed into `lockQueueHandle`. Controls the physical
 * solenoids/motors.
 */
typedef struct {
  uint8_t command; // 0 = Handle Lock, 1 = Handle Unlock, 2 = Seat Unlock
} LockCmd_t;

#endif /* ECU_TYPES_H */
