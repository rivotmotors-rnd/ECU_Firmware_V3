/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for FreeRTOS applications & task initialization
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ecu_actuators.h"
#include "ecu_can.h"
#include "ecu_tasks.h"
#include "ecu_types.h"
#include "ecu_uart.h"

/* Declared in syscalls.c — initializes the thread-safe printf mutex */
extern void printf_mutex_init(void);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t canRxQueue;
osMessageQueueId_t vcuCommandQueue;
osMessageQueueId_t actuatorCommandQueue;
osMessageQueueId_t uartRxQueue;

/* Task Attributes */
const osThreadAttr_t bridgeTask_attr = {
    .name = "CanUartBridge", .stack_size = 512 * 4, .priority = osPriorityHigh};
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief  FreeRTOS initialization (Queues, Tasks, Mutexes)
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void) {
  /* Initialize thread-safe printf mutex FIRST, before any task can call printf */
  printf_mutex_init();
  /* ======================================================================= */
  /* 1. CREATE RTOS MESSAGE QUEUES                                           */
  /* ======================================================================= */
  /**
   * @brief canRxQueue: Stores incoming CAN messages received via interrupts.
   *        The CAN to UART bridge task reads from this queue.
   */
  canRxQueue = osMessageQueueNew(16, sizeof(CanRxMsg_t), NULL);

  /**
   * @brief vcuCommandQueue: Stores high-level commands (e.g., turn on 12V, lock
   * vehicle) parsed from the UART packets. The Dispatcher task processes these.
   */
  vcuCommandQueue = osMessageQueueNew(100, sizeof(ControlCmd_t), NULL);

  /**
   * @brief actuatorCommandQueue: Stores physical lock commands.
   *        The Actuator Control task reads from this to trigger bit-banged
   * pulse sequences.
   */
  actuatorCommandQueue = osMessageQueueNew(10, sizeof(LockCmd_t), NULL);

  /**
   * @brief uartRxQueue: Stores raw byte frames received via UART DMA.
   *        The UartRxTask reads these bytes to parse them into VCU commands.
   */
  uartRxQueue = osMessageQueueNew(10, sizeof(UartRxFrame_t), NULL);

  /* ======================================================================= */
  /* 2. SPAWN APPLICATION THREADS                                            */
  /* ======================================================================= */
  osThreadNew(UartRxTask, NULL,
              &(osThreadAttr_t){.name = "UartRx",
                                .stack_size = 2048,
                                .priority = osPriorityAboveNormal});

  osThreadNew(CanToUartBridgeTask, NULL, &bridgeTask_attr);

  osThreadNew(VcuCommandDispatcherTask, NULL,
              &(osThreadAttr_t){.name = "VcuDispatcher",
                                .stack_size = 1024,
                                .priority = osPriorityHigh});

  osThreadNew(ActuatorControlTask, NULL,
              &(osThreadAttr_t){.name = "Actuators",
                                .stack_size = 1024,
                                .priority = osPriorityHigh});

  osThreadNew(TelemetryHeartbeatTask, NULL,
              &(osThreadAttr_t){.name = "Telemetry",
                                .stack_size = 1024,
                                .priority = osPriorityNormal});

  osThreadNew(VehicleLightingTask, NULL,
              &(osThreadAttr_t){.name = "Lighting",
                                .stack_size = 1024,
                                .priority = osPriorityNormal});

  osThreadNew(SeatUnlockButtonTask, NULL,
              &(osThreadAttr_t){.name = "SeatBtn",
                                .stack_size = 1024,
                                .priority = osPriorityLow});

  osThreadNew(BrakeMonitorTask, NULL,
              &(osThreadAttr_t){.name = "BrakeMonitor",
                                .stack_size = 1024,
                                .priority = osPriorityNormal});

  osThreadNew(CanHealthMonitorTask, NULL,
              &(osThreadAttr_t){.name = "CanHealth",
                                .stack_size = 2048,
                                .priority = osPriorityNormal});

  printf("FreeRTOS Tasks Initialized Successfully!\r\n");
}
/* USER CODE END Application */

