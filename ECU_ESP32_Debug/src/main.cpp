#include <Arduino.h>

#define ESP_RX_PIN 16
#define ESP_TX_PIN 17

/*
 * =========================================================
 *  WIRING INSTRUCTIONS:
 * =========================================================
 *  STM32 (ECU)            ESP32 (This Board)
 *  -------------------------------------------------
 *  PA2 (USART2_TX)  --->  Pin 16 (ESP_RX_PIN)
 *  PA3 (USART2_RX)  --->  Pin 17 (ESP_TX_PIN)
 *  GND              --->  GND (CRITICAL: Must share ground)
 * =========================================================
 */

void setup() {
  // Initialize USB Serial for PC monitor
  Serial.begin(115200);
  
  // Initialize Hardware Serial 2 for STM32 USART2
  // Format: begin(baud, config, rx_pin, tx_pin)
  Serial2.begin(115200, SERIAL_8N1, ESP_RX_PIN, ESP_TX_PIN);

  Serial.println("======================================");
  Serial.println("   ESP32 UART Bridge Initialized      ");
  Serial.printf("   Listening to STM32 TX on Pin %d\n", ESP_RX_PIN);
  Serial.printf("   Transmitting to STM32 RX on Pin %d\n", ESP_TX_PIN);
  Serial.println("======================================");
}

void loop() {
  // Read from STM32 (Serial2) and print to PC (Serial)
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }

  // Read from PC (Serial) and send to STM32 (Serial2)
  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
  }
}
