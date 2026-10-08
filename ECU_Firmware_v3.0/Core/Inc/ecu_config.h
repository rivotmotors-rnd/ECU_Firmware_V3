#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

#include "stm32f4xx_hal.h"

// =============================================================================
// 1. COMMUNICATION BUSES (UART & CAN)
// =============================================================================

// VCU UART (USART3 - Communication with Android VCU)
#define VCU_TX_PIN                  GPIO_PIN_10
#define VCU_TX_PORT                 GPIOB       // PB10
#define VCU_RX_PIN                  GPIO_PIN_5
#define VCU_RX_PORT                 GPIOC       // PC5
#define VCU_UART_AF                 GPIO_AF7_USART3

// Backward-compatibility aliases for VCU UART
#define VCU_TX_Pin                  VCU_TX_PIN
#define VCU_TX_GPIO_Port            VCU_TX_PORT
#define VCU_RX_Pin                  VCU_RX_PIN
#define VCU_RX_GPIO_Port            VCU_RX_PORT

// Debug UART (USART2 - ST-Link Virtual COM Port / Debug printf)
#define DEBUG_UART_TX_PIN           GPIO_PIN_2
#define DEBUG_UART_TX_PORT          GPIOA       // PA2
#define DEBUG_UART_RX_PIN           GPIO_PIN_3
#define DEBUG_UART_RX_PORT          GPIOA       // PA3
#define DEBUG_UART_AF               GPIO_AF7_USART2

// CAN Bus (CAN2 - Vehicle CAN Network)
#define CAN2_RX_PIN                 GPIO_PIN_12
#define CAN2_RX_PORT                GPIOB       // PB12
#define CAN2_TX_PIN                 GPIO_PIN_13
#define CAN2_TX_PORT                GPIOB       // PB13
#define CAN2_AF                     GPIO_AF9_CAN2

// Backward-compatibility aliases for CAN2
#define CAN2_RX_Pin                 CAN2_RX_PIN
#define CAN2_TX_Pin                 CAN2_TX_PIN
#define CAN2_GPIO_Port              CAN2_RX_PORT

// =============================================================================
// 2. POWER & SWITCHING (MOSFET CONTROL)
// =============================================================================

// 12V Main Load Switching MOSFET
#define SWITCHING_12V_PIN           GPIO_PIN_15
#define SWITCHING_12V_PORT          GPIOB       // PB15
#define SWITCHING_12V               SWITCHING_12V_PIN

// 12V VCU Power / Signal MOSFET
#define VCU_SIG_OP_PIN              GPIO_PIN_14
#define VCU_SIG_OP_PORT             GPIOB       // PB14
#define VCU_SIG_OP                  VCU_SIG_OP_PIN

// =============================================================================
// 3. ACTUATORS & LOCKS
// =============================================================================

// Handlebar Steering Lock (Single-wire DWT Pulse Protocol)
#define HNDLLOCK_SIG_OP_PIN         GPIO_PIN_2
#define HNDLLOCK_SIG_OP_PORT        GPIOC       // PC2

// Seat Latch Solenoid (Pulse Actuation)
#define SEATLOCK_SIG_OP_PIN         GPIO_PIN_9
#define SEATLOCK_SIG_OP_PORT        GPIOC       // PC9

// =============================================================================
// 4. LIGHTING OUTPUTS
// =============================================================================

// Left Turn Indicator Lamp
#define LH_INDICATOR_SIG_OP_PIN     GPIO_PIN_10
#define LH_INDICATOR_SIG_OP_PORT    GPIOC       // PC10
#define LH_INDICATOR_SIG_OP         LH_INDICATOR_SIG_OP_PIN

// Right Turn Indicator Lamp
#define RH_INDICATOR_SIG_OP_PIN     GPIO_PIN_12
#define RH_INDICATOR_SIG_OP_PORT    GPIOC       // PC12
#define RH_INDICATOR_SIG_OP         RH_INDICATOR_SIG_OP_PIN

// Tail Lamp / Backlight Output
#define TAIL_LAMP_SIG_OP_PIN        GPIO_PIN_6
#define TAIL_LAMP_SIG_OP_PORT       GPIOA       // PA6
#define TAIL_LAMP_SIG_OP            TAIL_LAMP_SIG_OP_PIN

// =============================================================================
// 5. VEHICLE INPUTS & SWITCHES
// =============================================================================

// Left Turn Indicator Switch
#define LEFT_INDICATOR_INP_PIN      GPIO_PIN_0
#define LEFT_INDICATOR_INP_PORT     GPIOC       // PC0
#define LEFT_INDICATOR_INP          LEFT_INDICATOR_INP_PIN

// Right Turn Indicator Switch
#define RIGHT_INDICATOR_INP_PIN     GPIO_PIN_1
#define RIGHT_INDICATOR_INP_PORT    GPIOB       // PB1
#define RIGHT_INDICATOR_INP         RIGHT_INDICATOR_INP_PIN

// Off / Kill Switch
#define OFF_INDICATOR_INP_PIN       GPIO_PIN_2
#define OFF_INDICATOR_INP_PORT      GPIOB       // PB2
#define OFF_INDICATOR_INP           OFF_INDICATOR_INP_PIN

// Hazard Light Switch
#define HAZARD_INP_PIN              GPIO_PIN_6
#define HAZARD_INP_PORT             GPIOC       // PC6
#define HAZARDAS                    HAZARD_INP_PIN

// Seat Latch Push Button
#define BUTTON_LOCK_PIN             GPIO_PIN_1
#define BUTTON_LOCK_PORT            GPIOC       // PC1

// High Beam / Ignition Status Sense
#define HIGH_BEAM_PIN               GPIO_PIN_8
#define HIGH_BEAM_PORT              GPIOC       // PC8

// =============================================================================
// 6. ANALOG INPUTS (ADC)
// =============================================================================

// Regenerative Braking / Throttle Hall Sensor
#define BRAKE_REGEN_ADC_PIN         GPIO_PIN_4
#define BRAKE_REGEN_ADC_PORT        GPIOA       // PA4
#define BRAKE_REGEN_OP_CH           ADC_CHANNEL_4

#endif // ECU_CONFIG_H
