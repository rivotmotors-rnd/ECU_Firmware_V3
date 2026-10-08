/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : ECU + CAN2 Bridge – DMA reception (V2‑style), reliable
 * heartbeats
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "ecu_actuators.h"
#include "ecu_can.h"
#include "ecu_tasks.h"
#include "ecu_types.h"
#include "ecu_uart.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_SIZE 64
#define PING_PONG_COUNT 2
#define TX_BYTE_QUEUE_SIZE 8192
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */
/* Manually declared USART2 handle since CubeMX skips it */
UART_HandleTypeDef huart2;

/* DMA reception buffer (ping-pong) */
uint8_t rx_buf[PING_PONG_COUNT][RX_SIZE];
volatile uint8_t current_dma_buffer = 0;

/* TX interrupt byte queue for CAN-to-UART & Telemetry */
volatile uint8_t tx_ongoing = 0;
uint8_t tx_byte_queue[TX_BYTE_QUEUE_SIZE];
volatile uint16_t tx_byte_head = 0;
volatile uint16_t tx_byte_tail = 0;

/* can1_is_running tracks the power state of the Charger CAN bus.
 * Initialized to 1 because MX_CAN1_Init() starts the peripheral.
 * CanHealthMonitorTask exclusively manages this flag and the CAN1
 * Start/Stop lifecycle to eliminate EXTI bounce bugs. */
volatile uint8_t can1_is_running = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN2_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
/* Custom USART2 initialization to bypass CubeMX restrictions on PA3 */
void Custom_USART2_UART_Init(void);
void kick_tx(void);
void MX_FREERTOS_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void kick_tx(void) {
  if (!tx_ongoing && (tx_byte_head != tx_byte_tail)) {
    tx_ongoing = 1;
    SET_BIT(huart3.Instance->CR1, USART_CR1_TXEIE);
  }
}

/**
 * @brief  Boot-time hardware self-test. Checks all PCB peripherals and pins
 *         before FreeRTOS starts. Reports faults over UART so you can
 *         immediately identify wiring, soldering, or component issues.
 *         This function NEVER blocks — it only logs and returns.
 */
static void Run_Hardware_Diagnostics(void) {
  uint8_t warn_count = 0;

  /* Pet the watchdog immediately at the start of diagnostics.
   * IWDG started ticking in MX_IWDG_Init(). The printf() calls below
   * are blocking (USART2 @ 115200 baud). Each character = ~87us.
   * ~2000 chars total = ~174ms. Refreshing here gives a fresh 3-sec window. */
  HAL_IWDG_Refresh(&hiwdg);

  /* =========================================================================
   * CHECK 1: RESET CAUSE
   * Read RCC->CSR to identify why the MCU last reset.
   * CRITICAL: This tells you immediately if the Watchdog triggered a reset.
   * =========================================================================
   */
  uint32_t rcc_csr = RCC->CSR;
  /* Clear the reset flags for the next boot cycle */
  RCC->CSR |= RCC_CSR_RMVF;

  printf("\r\n--- HARDWARE SELF-TEST BEGIN ---"
         "-\r\n");

  if (rcc_csr & RCC_CSR_IWDGRSTF) {
    printf("[RESET] *** WATCHDOG TIMEOUT *** A FreeRTOS task stopped running "
           "or took too long!\r\n");
    warn_count++;
  } else if (rcc_csr & RCC_CSR_WWDGRSTF) {
    printf("[RESET] *** WINDOW WATCHDOG TIMEOUT ***\r\n");
    warn_count++;
  } else if (rcc_csr & RCC_CSR_SFTRSTF) {
    printf("[RESET] Cause: Software Reset (HAL_NVIC_SystemReset called)\r\n");
  } else if (rcc_csr & RCC_CSR_PINRSTF) {
    printf("[RESET] Cause: Pin Reset (NRST button or external reset)\r\n");
  } else if (rcc_csr & RCC_CSR_BORRSTF) {
    printf("[RESET] Cause: Brown-Out Reset (supply voltage dropped below "
           "threshold!)\r\n");
    warn_count++;
  } else if (rcc_csr & RCC_CSR_PORRSTF) {
    printf("[RESET] Cause: Power-On Reset (normal cold boot)\r\n");
  } else {
    printf("[RESET] Cause: Unknown\r\n");
  }

  /* =========================================================================
   * CHECK 2: CAN BUS HEALTH
   * CAN pins (PB12/PB13 for CAN2, PA11/PA12 for CAN1) are ALTERNATE FUNCTION.
   * HAL_GPIO_ReadPin() does NOT work on AF pins. We check CAN->MSR register.
   * CAN_MSR_INAK=0 means peripheral is in Normal (active) mode — that's good.
   * CAN_MSR_INAK=1 means it is still in Init mode — peripheral failed to start.
   * =========================================================================
   */
  if (!(CAN2->MSR & CAN_MSR_INAK)) {
    printf("[DIAG] CAN2 (PB12/PB13) : OK  - Bus Active (Normal Mode)\r\n");
  } else {
    printf("[DIAG] CAN2 (PB12/PB13) : FAIL - Stuck in Init Mode! Check "
           "PB12/PB13 traces and termination resistor.\r\n");
    warn_count++;
  }

  if (!(CAN1->MSR & CAN_MSR_INAK)) {
    printf("[DIAG] CAN1 (PA11/PA12) : OK  - Bus Active (Normal Mode)\r\n");
  } else {
    /* Check if CAN1 was intentionally stopped because the gun is unplugged */
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) {
      printf("[DIAG] CAN1 (PA11/PA12) : OK  - Sleeping (No gun on PA5, "
             "expected)\r\n");
    } else {
      printf("[DIAG] CAN1 (PA11/PA12) : FAIL - Stuck in Init Mode! Check "
             "PA11/PA12 traces.\r\n");
      warn_count++;
    }
  }

  /* =========================================================================
   * CHECK 3: USART3 (VCU LINK) PERIPHERAL HEALTH
   * PC5/PB10 are ALTERNATE FUNCTION pins — cannot use HAL_GPIO_ReadPin().
   * Check USART3->SR: TXE bit should be SET when peripheral is ready and idle.
   * =========================================================================
   */
  if (USART3->SR & USART_SR_TXE) {
    printf("[DIAG] USART3 VCU Link   : OK  - TX Register Empty (Ready)\r\n");
  } else {
    printf("[DIAG] USART3 VCU Link   : WARN - TX not ready at boot. Check "
           "PB10/PC5.\r\n");
    warn_count++;
  }

  /* =========================================================================
   * CHECK 4: BRAKE SENSOR ADC (PA4 — ANALOG mode)
   * Do a single blocking conversion before RTOS starts.
   * ADC=0    → sensor open circuit, D1/D2 diodes not connected, or 12V Aux off
   * ADC=4095 → shorted to 3.3V rail
   * 0<x<4095 → electrically connected (even if 12V Aux is off, value ~0 is
   * normal)
   * =========================================================================
   */
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
    uint16_t adc_boot = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    if (adc_boot == 4095) {
      printf("[DIAG] Brake ADC  (PA4)  : WARN - Reads 4095 (SHORTED TO 3.3V! "
             "Check D1/D2 and PA4 trace)\r\n");
      warn_count++;
    } else if (adc_boot == 0) {
      printf("[DIAG] Brake ADC  (PA4)  : WARN - Reads 0 (sensor open or 12V "
             "Aux off — known HW issue if 12V off)\r\n");
      warn_count++;
    } else {
      printf("[DIAG] Brake ADC  (PA4)  : OK   - Reads %u (Sensor electrically "
             "connected)\r\n",
             adc_boot);
    }
  } else {
    HAL_ADC_Stop(&hadc1);
    printf("[DIAG] Brake ADC  (PA4)  : FAIL - PollForConversion timed out! "
           "ADC1 hardware issue.\r\n");
    warn_count++;
  }

  /* =========================================================================
   * CHECK 5: SWITCH & INPUT PINS (all are plain GPIO INPUT — valid to read)
   * These are all configured PULLUP, so idle state = HIGH = GPIO_PIN_SET.
   * If any read LOW at boot, the switch is physically pressed/stuck/shorted.
   * HIGH_BEAM (PC8) is PULLDOWN, so idle = LOW = GPIO_PIN_RESET.
   * =========================================================================
   */
  /* Left Indicator Switch (PC0) — PULLUP, active LOW */
  GPIO_PinState sw;
  sw = HAL_GPIO_ReadPin(LEFT_INDICATOR_INP_PORT, LEFT_INDICATOR_INP_PIN);
  printf("[DIAG] Left  Sw   (PC0)  : %s%s\r\n",
         (sw == GPIO_PIN_RESET) ? "ON " : "OFF",
         (sw == GPIO_PIN_RESET) ? " <-- WARN: switch active at boot!" : "");
  if (sw == GPIO_PIN_RESET)
    warn_count++;

  /* Seat Button (PC1) — PULLUP, active LOW */
  sw = HAL_GPIO_ReadPin(BUTTON_LOCK_PORT, BUTTON_LOCK_PIN);
  printf("[DIAG] Seat  Btn  (PC1)  : %s%s\r\n",
         (sw == GPIO_PIN_RESET) ? "ON " : "OFF",
         (sw == GPIO_PIN_RESET) ? " <-- WARN: button held at boot!" : "");
  if (sw == GPIO_PIN_RESET)
    warn_count++;

  /* Hazard Switch (PC6) — PULLUP, active LOW */
  sw = HAL_GPIO_ReadPin(HAZARD_INP_PORT, HAZARD_INP_PIN);
  printf("[DIAG] Hazard Sw  (PC6)  : %s%s\r\n",
         (sw == GPIO_PIN_RESET) ? "ON " : "OFF",
         (sw == GPIO_PIN_RESET) ? " <-- WARN: hazard active at boot!" : "");
  if (sw == GPIO_PIN_RESET)
    warn_count++;

  /* Right Indicator Switch (PB1) — PULLUP, active LOW */
  sw = HAL_GPIO_ReadPin(RIGHT_INDICATOR_INP_PORT, RIGHT_INDICATOR_INP_PIN);
  printf("[DIAG] Right Sw   (PB1)  : %s%s\r\n",
         (sw == GPIO_PIN_RESET) ? "ON " : "OFF",
         (sw == GPIO_PIN_RESET) ? " <-- WARN: switch active at boot!" : "");
  if (sw == GPIO_PIN_RESET)
    warn_count++;

  /* Indicator Off Switch (PB2) — PULLUP, active LOW */
  sw = HAL_GPIO_ReadPin(OFF_INDICATOR_INP_PORT, OFF_INDICATOR_INP_PIN);
  printf("[DIAG] Ind Off    (PB2)  : %s%s\r\n",
         (sw == GPIO_PIN_RESET) ? "ON " : "OFF",
         (sw == GPIO_PIN_RESET)
             ? " <-- WARN: indicator off switch held at boot!"
             : "");
  if (sw == GPIO_PIN_RESET)
    warn_count++;

  /* High Beam / Ignition Sense (PC8) — PULLDOWN, active HIGH */
  sw = HAL_GPIO_ReadPin(HIGH_BEAM_PORT, HIGH_BEAM_PIN);
  printf("[DIAG] HighBeam   (PC8)  : %s\r\n",
         (sw == GPIO_PIN_SET) ? "ON" : "OFF");

  /* =========================================================================
   * CHECK 5b: CONTACTOR FEEDBACK (PA3 — PULLUP, active LOW)
   * PA3 is the Auxiliary Contact of the HV Contactor relay.
   * At boot, the MOSFET is OFF, so the contactor MUST be open.
   * Expected state = HIGH (3.3V via pull-up = contactor open = safe).
   * If PA3 reads LOW at boot, the contactor contacts are WELDED SHUT!
   * This is a critical hardware fault — HV battery is permanently connected.
   * =========================================================================
   */
  GPIO_PinState contactor_fb =
      HAL_GPIO_ReadPin(CONTACTOR_FEEDBACK_GPIO_Port, CONTACTOR_FEEDBACK_Pin);
  if (contactor_fb == GPIO_PIN_SET) {
    printf("[DIAG] Contactor  (PA3)  : OK   - Feedback HIGH (Contactor OPEN,"
           " safe)\r\n");
  } else {
    printf("[DIAG] Contactor  (PA3)  : FATAL - Feedback LOW! CONTACTOR MAY BE"
           " WELDED SHUT! HV IS LIVE!\r\n");
    warn_count++;
  }

  /* =========================================================================
   * CHECK 6: MOSFET OUTPUT READBACK
   * Write a known state, read it back. If they differ, the output driver,
   * MOSFET gate, or trace has a fault (short, open, damaged FET).
   * VCU MOSFET (PB14) is set HIGH at boot. 12V Aux (PB15) is LOW at boot.
   * =========================================================================
   */
  GPIO_PinState vcu_out = HAL_GPIO_ReadPin(VCU_SIG_OP_PORT, VCU_SIG_OP_PIN);
  if (vcu_out == GPIO_PIN_SET) {
    printf("[DIAG] VCU  MOSFET(PB14) : OK   - Output HIGH (as expected)\r\n");
  } else {
    printf("[DIAG] VCU  MOSFET(PB14) : WARN - Output reads LOW! Expected HIGH. "
           "Check PB14 trace/FET gate.\r\n");
    warn_count++;
  }

  GPIO_PinState aux_out =
      HAL_GPIO_ReadPin(SWITCHING_12V_PORT, SWITCHING_12V_PIN);
  if (aux_out == GPIO_PIN_RESET) {
    printf("[DIAG] 12V  Aux   (PB15) : OK   - Output LOW  (as expected)\r\n");
  } else {
    printf("[DIAG] 12V  Aux   (PB15) : WARN - Output reads HIGH! Expected LOW. "
           "Check PB15 trace/FET gate.\r\n");
    warn_count++;
  }

  /* =========================================================================
   * CHECK 7: WATCHDOG (IWDG) CONFIRMATION
   * If MX_IWDG_Init ran successfully, the IWDG is running and cannot be
   * stopped. This is just a confirmation print. Presence of this log = IWDG is
   * armed.
   * =========================================================================
   */
  printf("[DIAG] IWDG Watchdog : ARMED (Prescaler=32, Reload=3000, "
         "Timeout~3s)\r\n");

  /* =========================================================================
   * SUMMARY
   * =========================================================================
   */
  if (warn_count == 0) {
    printf("--- SELF-TEST COMPLETE: ALL OK ---\r\n\r\n");
  } else {
    printf("--- SELF-TEST COMPLETE: %u WARNING(S) FOUND ---\r\n\r\n",
           warn_count);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_CAN2_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  /* Manually initialize USART2 for TX only (PA2) because CubeMX disabled it.
   * This completely avoids the PA3 conflict. */
  Custom_USART2_UART_Init();

  printf("\r\n================================\r\n");
  printf("  ECU BOOT SEQUENCE INITIATED   \r\n");
  printf("================================\r\n");
  printf("[SYS] GPIO + DMA + USART2 Initialized\r\n");
  printf("[SYS] ADC1 Initialized\r\n");
  printf("[SYS] CAN1 + CAN2 Initialized\r\n");
  printf("[SYS] USART3 (VCU Comms) Initialized\r\n");
  printf("[SYS] IWDG Watchdog Initialized\r\n");

  /* Start CAN1 and CAN2 FIRST so the hardware diagnostic can accurately check
   * whether CAN left Init Mode. Previously this ran after diagnostics,
   * meaning INAK was always 1 (false FAIL). */
  hcan1.Instance->MCR |= CAN_MCR_ABOM;
  hcan2.Instance->MCR |= CAN_MCR_ABOM;

  CAN_FilterTypeDef sFilterConfig = {0};
  sFilterConfig.FilterBank = 14;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    Error_Handler();

  /* CAN1 Filter — Bank 0, accept ALL messages from Charger (CHAdeMO) */
  CAN_FilterTypeDef sCan1FilterConfig = {0};
  sCan1FilterConfig.FilterBank = 0;
  sCan1FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sCan1FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sCan1FilterConfig.FilterIdHigh = 0x0000;
  sCan1FilterConfig.FilterIdLow = 0x0000;
  sCan1FilterConfig.FilterMaskIdHigh = 0x0000;
  sCan1FilterConfig.FilterMaskIdLow = 0x0000;
  sCan1FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sCan1FilterConfig.FilterActivation = ENABLE;
  sCan1FilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &sCan1FilterConfig) != HAL_OK)
    Error_Handler();

  /* Start CAN1 and CAN2 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
    Error_Handler();
  if (HAL_CAN_Start(&hcan2) != HAL_OK)
    Error_Handler();

  /* Arm CAN1 RX interrupt unconditionally — the EXTI callback will stop/start
   * CAN1, but the notification must be armed from the beginning so it works
   * correctly when CAN1 is woken up at runtime */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK)
    Error_Handler();

  /* Arm CAN2 RX interrupt */
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) !=
      HAL_OK)
    Error_Handler();

  /* Note: CAN1 power state (Start/Stop) and interrupt activation based on
   * the charger gun (CP line) is now exclusively managed by the
   * CanHealthMonitorTask in FreeRTOS to eliminate boot state bugs and
   * mechanical switch bouncing. CAN1 is started by default here via CubeMX,
   * but the RTOS task will shut it down within 100ms if no gun is present. */

  /* Run full hardware PCB self-test — CAN is now started so the diagnostic
   * will correctly report NORMAL mode instead of a false Init Mode FAIL */
  Run_Hardware_Diagnostics();

  /* Pet the watchdog after diagnostics completes. */
  HAL_IWDG_Refresh(&hiwdg);

  // Start DMA reception (same as V2 PCB)
  HAL_UART_Receive_DMA(&huart3, rx_buf[current_dma_buffer], RX_SIZE);
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  printf("[SYS] UART3 DMA RX Started & IDLE IT Enabled\r\n");

  DWT_Init();
  printf("[SYS] DWT Cycle Counter Initialized\r\n");
  printf("[SYS] Peripheral Initialization Complete. Starting FreeRTOS...\r\n");

  /* Pet the watchdog one final time before handing off to FreeRTOS.
   * osKernelInitialize() + MX_FREERTOS_Init() (9 tasks + 4 queues) can take
   * time. This ensures the scheduler starts within a fresh 3-second window.
   * After this, TelemetryHeartbeatTask takes over petting every 1000ms. */
  HAL_IWDG_Refresh(&hiwdg);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  MX_FREERTOS_Init();
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* INCREASED SAMPLING TIME: The brake signal passes through diodes (D1, D2).
   * This creates a high-impedance source. 3 cycles is too short to charge
   * the ADC capacitor and will result in 0V readings. 480 cycles fixes this. */
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

  /* USER CODE BEGIN CAN1_Init 0 */
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */
  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef can1Filter = {0};
  can1Filter.FilterBank = 0;
  can1Filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can1Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can1Filter.FilterIdHigh = 0x0000;
  can1Filter.FilterIdLow = 0x0000;
  can1Filter.FilterMaskIdHigh = 0x0000;
  can1Filter.FilterMaskIdLow = 0x0000;
  can1Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  can1Filter.FilterActivation = ENABLE;
  can1Filter.SlaveStartFilterBank = 14;
  HAL_CAN_ConfigFilter(&hcan1, &can1Filter);

  /* Enable CAN1 RX0 interrupt in the NVIC */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE END CAN1_Init 2 */
}

/**
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void) {

  /* USER CODE BEGIN CAN2_Init 0 */
  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */
  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 10;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  /* NOTE: AutoRetransmission is already set to ENABLE above before
   * HAL_CAN_Init. CAN bus is physically connected. Frames are retried on missed
   * ACK. If you disconnect the CAN bus for testing, set AutoRetransmission =
   * DISABLE in the HAL_CAN_Init block above to avoid TX Mailbox Full errors. */
  /* USER CODE END CAN2_Init 2 */
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Reload = 3000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC,
                    HANDLE_LOCK_Pin | SEAT_LOCK_Pin | LEFT_IND_LAMP_Pin |
                        RIGHT_IND_LAMP_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TAIL_LAMP_GPIO_Port, TAIL_LAMP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VCU_PWR_GPIO_Port, VCU_PWR_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AUX_12V_EN_Pin | HV_CONTACTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LEFT_IND_SW_Pin SEAT_BTN_Pin HAZARD_SW_Pin */
  GPIO_InitStruct.Pin = LEFT_IND_SW_Pin | SEAT_BTN_Pin | HAZARD_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HANDLE_LOCK_Pin SEAT_LOCK_Pin LEFT_IND_LAMP_Pin
   * RIGHT_IND_LAMP_Pin */
  GPIO_InitStruct.Pin =
      HANDLE_LOCK_Pin | SEAT_LOCK_Pin | LEFT_IND_LAMP_Pin | RIGHT_IND_LAMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CONTACTOR_FEEDBACK_Pin CP_LINE_DETECT_Pin */
  GPIO_InitStruct.Pin = CONTACTOR_FEEDBACK_Pin | CP_LINE_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : TAIL_LAMP_Pin */
  GPIO_InitStruct.Pin = TAIL_LAMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TAIL_LAMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RIGHT_IND_SW_Pin OFF_INDICATOR_INP_Pin */
  GPIO_InitStruct.Pin = RIGHT_IND_SW_Pin | OFF_INDICATOR_INP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : VCU_PWR_Pin AUX_12V_EN_Pin HV_CONTACTOR_Pin */
  GPIO_InitStruct.Pin = VCU_PWR_Pin | AUX_12V_EN_Pin | HV_CONTACTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HIGH_BEAM_SENSE_Pin */
  GPIO_InitStruct.Pin = HIGH_BEAM_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(HIGH_BEAM_SENSE_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* NOTE: PB5 (HV_CONTACTOR_Pin) and PA5 (CP_LINE_DETECT_Pin) are fully
   * configured by CubeMX in the auto-generated section above.
   * EXTI9_5_IRQn and CAN1_RX0_IRQn are also enabled by CubeMX.
   * No manual GPIO or NVIC init needed here. */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* =========================================================================
 * CUSTOM USART2 INITIALIZATION (Bypasses CubeMX limitation)
 * Enables PA2 as TX and leaves PA3 untouched (for GPIO use).
 * ========================================================================= */
void Custom_USART2_UART_Init(void) {
  /* 1. Enable Clocks */
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* 2. Configure PA2 as USART2_TX (Alternate Function 7) */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 3. Initialize UART peripheral for TX only */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    Error_Handler();
  }
}

/* Route printf output to UART2 (ESP32 debug link, TX only to PA2). */
int _write(int file, char *ptr, int len) {
  (void)file;
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}

/* Error Recovery Callback */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) {
    if (huart->ErrorCode &
        (HAL_UART_ERROR_ORE | HAL_UART_ERROR_NE | HAL_UART_ERROR_FE)) {
      __HAL_UART_CLEAR_OREFLAG(huart);
      __HAL_UART_CLEAR_NEFLAG(huart);
      __HAL_UART_CLEAR_FEFLAG(huart);
      HAL_UART_Receive_DMA(huart, rx_buf[current_dma_buffer], RX_SIZE);

      // Rate-limit the print to once every 5 seconds to prevent log flooding
      static uint32_t last_err_print = 0;
      if (HAL_GetTick() - last_err_print > 5000) {
        printf("[UART3] Error Recovered (Noise/Overrun/Disconnect)\r\n");
        last_err_print = HAL_GetTick();
      }
    }
  }
}

/**
 * @brief Hardware Interrupt Callback for GPIO Pins
 *        This fires instantly the exact millisecond the CHAdeMO gun is plugged
 * in or unplugged.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  printf("\r\n[CRITICAL ERROR] Error_Handler() invoked! System halted.\r\n");
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
