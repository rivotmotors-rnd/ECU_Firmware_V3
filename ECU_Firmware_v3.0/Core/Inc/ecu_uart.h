/**
 ******************************************************************************
 * @file    ecu_uart.h
 * @brief   Header for UART Parsing Subsystem
 ******************************************************************************
 */
#ifndef ECU_UART_H
#define ECU_UART_H

#include "ecu_types.h"
#include <stddef.h>

/* Expose these functions so main.c can call them */
uint16_t calculate_checksum(uint8_t *buffer, size_t totalLength);
void parse_vcu_frame(uint8_t *buf, uint16_t len);
void UartRxTask(void *argument);
void send_to_vcu(uint8_t state, uint8_t task, uint8_t *payload, uint16_t payload_len);

#endif /* ECU_UART_H */
