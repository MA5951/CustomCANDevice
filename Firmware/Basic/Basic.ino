#include <Arduino.h>
#include "driver/twai.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define I2C_SDA 8  // ESP32-C3 Super Mini SDA
#define I2C_SCL 9  // ESP32-C3 Super Mini SCL

Adafruit_VL53L0X lox;

#define DEVICE_ID 0x0A
#define MANUFACTURER_ID 0x08
#define DEVICE_NUMBER 50
#define SENSOR_BASE_API_ID 0x0301

void setup() {
  Serial.begin(115200);

  Serial.println("Setup starting...");

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_20, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return; 
  }

  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.println("\nVL53L0X init...");

  if (!lox.begin()) {
    Serial.println("ERROR: VL53L0X not found. Check wiring and power (3.3V), address 0x29.");
    while (true) { delay(1000); }
  }

  Serial.println("VL53L0X ready.");
  Serial.println("Setup complete!");
}

void loop() {
  twai_message_t msg1 = {};
  msg1.identifier = makeCANSPID(DEVICE_ID, MANUFACTURER_ID, SENSOR_BASE_API_ID, DEVICE_NUMBER);
  msg1.extd = 1;
  msg1.data_length_code = 2;

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
 if (measure.RangeStatus != 4) {
    int mm = measure.RangeMilliMeter;
    msg1.data[0] = mm & 0xFF;  
    msg1.data[1] = (mm >> 8) & 0xFF;
  } 

  esp_err_t result = twai_transmit(&msg1, pdMS_TO_TICKS(10));

  if (result == ESP_OK) {
    Serial.println("Ok");
  } else {
    Serial.printf("Transmit failed: %d\n", result);
  }


  delay(15);
}

uint32_t makeCANSPID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID) << 24) | ((uint32_t)(manufacturerID) << 16) | ((uint32_t)(apiID & 0x3FF) << 6) | (deviceNumber & 0x3F);
}
