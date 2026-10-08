/**
 ******************************************************************************
 * @file    ecu_can.h
 * @brief   Header for CAN Subsystem
 ******************************************************************************
 */
#ifndef ECU_CAN_H
#define ECU_CAN_H

#include "ecu_types.h"

void forward_can_to_vcu(CanRxMsg_t *msg);
void CanToUartBridgeTask(void *argument);
void CanHealthMonitorTask(void *argument);

#endif /* ECU_CAN_H */
