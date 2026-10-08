/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern osMessageQueueId_t canRxQueue;
extern osMessageQueueId_t vcuCommandQueue;
extern osMessageQueueId_t actuatorCommandQueue;
extern osMessageQueueId_t uartRxQueue;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#include <stdio.h>
#define APP_ERROR(reason)                                                      \
  do {                                                                         \
    printf("\r\n[CRITICAL ERROR] %s (File: %s, Line: %d)\r\n", reason,         \
           __FILE__, __LINE__);                                                \
    Error_Handler();                                                           \
  } while (0)
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void MX_FREERTOS_Init(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BRAKE_INP_Pin GPIO_PIN_0
#define BRAKE_INP_GPIO_Port GPIOA
#define LEFT_IND_SW_Pin GPIO_PIN_0
#define LEFT_IND_SW_GPIO_Port GPIOC
#define SEAT_BTN_Pin GPIO_PIN_1
#define SEAT_BTN_GPIO_Port GPIOC
#define HANDLE_LOCK_Pin GPIO_PIN_2
#define HANDLE_LOCK_GPIO_Port GPIOC
#define RIGHT_IND_SW_Pin GPIO_PIN_1
#define RIGHT_IND_SW_GPIO_Port GPIOB
#define OFF_INDICATOR_INP_Pin GPIO_PIN_2
#define OFF_INDICATOR_INP_GPIO_Port GPIOB
#define CONTACTOR_FEEDBACK_Pin GPIO_PIN_3
#define CONTACTOR_FEEDBACK_GPIO_Port GPIOB
#define HV_CONTACTOR_Pin GPIO_PIN_5
#define HV_CONTACTOR_GPIO_Port GPIOB
#define CP_LINE_DETECT_Pin GPIO_PIN_7
#define CP_LINE_DETECT_GPIO_Port GPIOB
#define TAIL_LAMP_Pin GPIO_PIN_6
#define TAIL_LAMP_GPIO_Port GPIOA
#define VCU_PWR_Pin GPIO_PIN_14
#define VCU_PWR_GPIO_Port GPIOB
#define AUX_12V_EN_Pin GPIO_PIN_15
#define AUX_12V_EN_GPIO_Port GPIOB
#define HAZARD_SW_Pin GPIO_PIN_6
#define HAZARD_SW_GPIO_Port GPIOC
#define HIGH_BEAM_SENSE_Pin GPIO_PIN_8
#define HIGH_BEAM_SENSE_GPIO_Port GPIOC
#define SEAT_LOCK_Pin GPIO_PIN_9
#define SEAT_LOCK_GPIO_Port GPIOC
#define LEFT_IND_LAMP_Pin GPIO_PIN_10
#define LEFT_IND_LAMP_GPIO_Port GPIOC
#define RIGHT_IND_LAMP_Pin GPIO_PIN_12
#define RIGHT_IND_LAMP_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#include "ecu_config.h"

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
