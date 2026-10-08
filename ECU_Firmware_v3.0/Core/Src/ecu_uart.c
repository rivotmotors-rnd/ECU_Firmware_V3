/**
 ******************************************************************************
 * @file    ecu_uart.c
 * @brief   UART Parsing Subsystem
 *
 * @details Handles receiving DMA buffers from the Android VCU, parsing the
 *          raw bytes into frames, verifying them, and pushing them to the
 *          FreeRTOS Control Queue.
 ******************************************************************************
 */

#include "ecu_uart.h"
#include "cmsis_os2.h"
#include "ecu_config.h" /* For CONTACTOR_TEST_MODE */
#include <stdio.h>
#include <string.h>

/* We need to talk to the queues created in main.c */
extern osMessageQueueId_t vcuCommandQueue;
extern osMessageQueueId_t uartRxQueue;

#include "FreeRTOS.h"
#include "task.h"

/* --- External UART TX Queue Variables --- */
#define TX_BYTE_QUEUE_SIZE 8192
extern uint8_t tx_byte_queue[TX_BYTE_QUEUE_SIZE];
extern volatile uint16_t tx_byte_head;
extern volatile uint16_t tx_byte_tail;
extern void kick_tx(void);

/* ========================================================================= */
/* VCU PACKET BUILDER                                                        */
/* ========================================================================= */
/**
 * @brief  Builds a protocol-compliant UART frame and pushes it to the DMA TX
 * queue.
 * @param  state       The State byte (e.g., 0x03 for Heartbeat).
 * @param  task        The Task byte (e.g., 0x01 for telemetry, 0x03 for CAN).
 * @param  payload     Pointer to the data payload bytes.
 * @param  payload_len Number of bytes in the payload.
 */
void send_to_vcu(uint8_t state, uint8_t task, uint8_t *payload,
                 uint16_t payload_len) {
  // 1. Create a large enough array for the final packet
  uint8_t packet[256];

  // 2. Calculate the total UART frame length
  // (payload bytes + 9 overhead bytes for header, lengths, and CRC)
  uint16_t total_len = payload_len + 9;

  if (total_len > 256)
    return;

  // 3. Attach the strict protocol headers
  packet[0] = 0xFF;  // Frame Header
  packet[1] = 0xAA;  // Frame Header
  packet[2] = 0x77;  // Frame Header (MCU to VCU)
  packet[3] = state; // e.g. 0x03 for Heartbeat State
  packet[4] = task;  // e.g. 0x01 for Heartbeat Task

  // 4. Attach the length (split into 2 bytes: High and Low)
  packet[5] = (total_len >> 8) & 0xFF;
  packet[6] = total_len & 0xFF;

  // 5. Copy the payload perfectly into the packet directly after the headers
  if (payload != NULL && payload_len > 0) {
    memcpy(&packet[7], payload, payload_len);
  }

  // 6. Calculate the mathematical checksum (CRC) across the packet
  uint16_t crc = calculate_checksum(packet, total_len);

  // 7. Attach the 2-byte CRC to the very end of the packet
  packet[total_len - 2] = (crc >> 8) & 0xFF;
  packet[total_len - 1] = crc & 0xFF;

  // 8. Lock the RTOS so we don't get interrupted while writing to the DMA queue
  taskENTER_CRITICAL();

  // 9. Calculate how much space is left in our circular transmission queue
  uint16_t free = (TX_BYTE_QUEUE_SIZE + tx_byte_tail - tx_byte_head - 1) %
                  TX_BYTE_QUEUE_SIZE;

  // 10. If there's enough space, dump our packet into the queue!
  if (free >= total_len) {
    for (int i = 0; i < total_len; i++) {
      tx_byte_queue[tx_byte_head] = packet[i];
      tx_byte_head = (tx_byte_head + 1) % TX_BYTE_QUEUE_SIZE;
    }
  }

  // 11. Unlock the RTOS
  taskEXIT_CRITICAL();

  // 12. Pull the trigger to wake up hardware DMA transmission!
  if (free >= total_len) {
    kick_tx();
  }
}

/* ---------- Private ECU parser state ---------- */
/* Because these are "static", they are totally hidden from main.c! */

/* ========================================================================= */
/* CHECKSUM CALCULATOR                                                       */
/* ========================================================================= */
/**
 * @brief Calculates a simple 8-bit checksum for an array of bytes.
 *        Used to validate the integrity of incoming UART frames.
 * @param buffer Pointer to the data array.
 * @param totalLength Length of the data array.
 * @return The calculated 8-bit checksum.
 */
uint16_t calculate_checksum(uint8_t *buffer, size_t totalLength) {
  uint16_t sum = 0;
  for (size_t i = 0; i < totalLength - 2; i++) {
    sum += buffer[i];
  }
  return sum;
}

/* ========================================================================= */
/* VCU FRAME PARSER (UNIFIED)                                                */
/* ========================================================================= */
/**
 * @brief Unified state machine that parses raw bytes from the UART DMA ring
 * buffer. It detects the frame header (0xFF 0xAA 0x55), checks the length,
 * validates the CRC, and routes commands either to the CAN bus or the VCU
 * Command Queue.
 * @param buf Pointer to the raw received byte buffer.
 * @param len Number of raw bytes to process.
 */
void parse_vcu_frame(uint8_t *buf, uint16_t len) {
  static uint8_t frame[128];
  static uint16_t f_idx = 0;
  static uint16_t expected_len = 0;
  /* State machine:
   *  0 = Waiting for 0xFF
   *  1 = Got 0xFF, waiting for 0xAA
   *  2 = Got 0xAA, waiting for 0x55
   *  3 = Got 0x55, reading header bytes (State, Task, LenH, LenL)
   *  4 = Reading data + CRC
   */
  static uint8_t state = 0;

  for (uint16_t i = 0; i < len; i++) {
    uint8_t ch = buf[i];
    switch (state) {

    case 0: /* Waiting for 0xFF */
      if (ch == 0xFF) {
        frame[0] = ch;
        state = 1;
      }
      break;

    case 1: /* Got 0xFF, waiting for 0xAA */
      if (ch == 0xAA) {
        frame[1] = ch;
        state = 2;
      } else if (ch == 0xFF) {
        /* Back-to-back 0xFF: stay in state 1, keep last 0xFF */
        frame[0] = ch;
        state = 1;
      } else {
        /* Unexpected byte — drop 0xFF, re-examine this byte from state 0 */
        state = 0;
        i--; /* re-process this byte */
      }
      break;

    case 2: /* Got 0xAA, waiting for 0x55 */
      if (ch == 0x55) {
        frame[2] = ch;
        f_idx = 3;
        state = 3;
      } else if (ch == 0xFF) {
        /* Could be start of a new sync — step back into state 1 */
        frame[0] = ch;
        state = 1;
      } else {
        /* Not 0x55, not 0xFF — drop and re-examine from state 0 */
        state = 0;
        i--;
      }
      break;

    case 3: /* Reading State, Task, LenH, LenL bytes */
      frame[f_idx++] = ch;
      if (f_idx == 7) {
        /* Check if Task byte is a sync byte — screen lock/unlock mid-tx */
        if (frame[4] == 0xFF || frame[4] == 0xAA || frame[4] == 0x55) {
          static uint32_t last_sync_err = 0;
          if (HAL_GetTick() - last_sync_err > 2000) {
            printf("\r\n[UART ERROR] Incomplete VCU packet! Task byte=0x%02X is "
                   "a sync byte. VCU sent State=0x%02X with no Task/CRC.\r\n",
                   frame[4], frame[3]);
            last_sync_err = HAL_GetTick();
          }
          /* Re-sync: if the task byte itself is 0xFF, restart from state 1 */
          state = 0;
          if (frame[4] == 0xFF) { i--; } /* re-process the 0xFF */
          break;
        }

        expected_len = (frame[5] << 8) | frame[6];
        if (expected_len >= 9 && expected_len <= 128) {
          state = 4;
        } else {
          static uint32_t last_len_err = 0;
          if (HAL_GetTick() - last_len_err > 2000) {
            printf("\r\n[UART ERROR] Invalid Length (%d=0x%04X). Raw hdr: "
                   "State=0x%02X Task=0x%02X LenH=0x%02X LenL=0x%02X\r\n",
                   expected_len, expected_len,
                   frame[3], frame[4], frame[5], frame[6]);
            last_len_err = HAL_GetTick();
          }
          state = 0;
        }
      }
      break;

    case 4: /* Reading data and CRC */
      frame[f_idx++] = ch;
      if (f_idx == expected_len) {
        uint16_t crc_calc = calculate_checksum(frame, expected_len);
        uint16_t crc_rx = (frame[expected_len - 2] << 8) | frame[expected_len - 1];

        if (crc_calc == crc_rx) {
          /* ---- VALID FRAME ---- */
          uint8_t cmd_state = frame[3];
          uint8_t cmd_task  = frame[4];
          uint16_t data_payload_len = expected_len - 9;

          if (cmd_task == 0x03) {
            /* Task 0x03: inject CAN frame */
            if (data_payload_len == 12) {
              uint32_t id = ((uint32_t)frame[7]  << 24) |
                            ((uint32_t)frame[8]  << 16) |
                            ((uint32_t)frame[9]  <<  8) | frame[10];
              CAN_TxHeaderTypeDef TxHeader;
              uint8_t TxData[8];
              uint32_t TxMailbox;
              TxHeader.IDE = (id <= 0x7FF) ? CAN_ID_STD : CAN_ID_EXT;
              if (id <= 0x7FF) TxHeader.StdId = id;
              else             TxHeader.ExtId = id;
              TxHeader.RTR = CAN_RTR_DATA;
              TxHeader.DLC = 8;
              TxHeader.TransmitGlobalTime = DISABLE;
              for (int j = 0; j < 8; j++) TxData[j] = frame[11 + j];

              extern CAN_HandleTypeDef hcan2;
              if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) > 0) {
                HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
              } else {
                static uint32_t last_can_err = 0;
                if (HAL_GetTick() - last_can_err > 2000) {
#ifndef CONTACTOR_TEST_MODE
                  printf("\r\n[CAN ERROR] TX Mailbox Full! Dropping frame. ESR=0x%08lX MSR=0x%08lX TSR=0x%08lX\r\n", 
                         hcan2.Instance->ESR, hcan2.Instance->MSR, hcan2.Instance->TSR);
#endif
                  last_can_err = HAL_GetTick();
                }
              }
            }
          } else {
            /* Regular Control Command */
            ControlCmd_t cmd;
            cmd.state = cmd_state;
            cmd.type  = cmd_task;
            memset(cmd.data, 0, 8);
            uint16_t copy_len = (data_payload_len > 8) ? 8 : data_payload_len;
            for (int j = 0; j < copy_len; j++) cmd.data[j] = frame[7 + j];
            osMessageQueuePut(vcuCommandQueue, &cmd, 0, 0);
          }

        } else {
          /* ---- CRC FAILED: scan buffered frame bytes for next sync ---- */
          static uint32_t last_crc_err = 0;
          if (HAL_GetTick() - last_crc_err > 2000) {
            printf("\r\n[UART ERROR] Checksum Failed!"
                   " Calc:0x%04X Rx:0x%04X"
                   " State=0x%02X Task=0x%02X Len=%d"
                   " Data[0..3]=%02X %02X %02X %02X\r\n",
                   crc_calc, crc_rx,
                   frame[3], frame[4], expected_len,
                   frame[7], frame[8], frame[9], frame[10]);
            last_crc_err = HAL_GetTick();
          }
          /* Scan the collected frame from byte 1 onwards for 0xFF 0xAA 0x55
           * so we don't have to wait for the next DMA burst to re-sync */
          state = 0;
          for (uint16_t s = 1; s < expected_len - 2; s++) {
            if (frame[s] == 0xFF && frame[s+1] == 0xAA && frame[s+2] == 0x55) {
              /* Found embedded sync — replay from here */
              frame[0] = 0xFF; frame[1] = 0xAA; frame[2] = 0x55;
              f_idx = 3;
              state = 3;
              break;
            }
          }
        }
        /* Reset for next frame only if we are not mid-resync */
        if (state == 4) state = 0;
      }
      break;

    default:
      state = 0;
      break;
    }
  }
}

/* ========================================================================= */
/* 6. UART RX TASK                                                           */
/* ========================================================================= */
/**
 * @brief RTOS task that continuously reads raw DMA frames from `uartRxQueue`.
 *        It feeds the bytes into the `processECUFrame` unified state machine.
 * @param argument Unused FreeRTOS argument.
 */
void UartRxTask(void *argument) {
  UartRxFrame_t frame;
  osDelay(90);
  printf("[TASK] UART RX Parser Started\r\n");
  for (;;) {
    // 1. Wait indefinitely for DMA to push a buffer of raw bytes into the queue
    if (osMessageQueueGet(uartRxQueue, &frame, NULL, osWaitForever) == osOK) {
      // 2. Feed the bytes into the parser state machine
      parse_vcu_frame(frame.data, frame.length);
    }
  }
}
