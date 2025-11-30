#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>
#include "driver/twai.h"

VL53L1X sensor;

// ----- LED setup -----
#define LED_PIN 3        // pick any valid GPIO on your ESP32-C3
#define LED_COUNT 1

Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ----- timing -----
unsigned long lastTime = 0;  // ms
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

#define DEVICE_ID 0x0A
#define MANUFACTURER_ID 0x08
#define DEVICE_NUMBER 20  
#define SENSOR_BASE_API_ID 0x0301


void setup() {
  Serial.begin(115200);
  delay(3000);

  // pixel.begin();
  // pixel.setBrightness(40); // not full power
  // pixel.show(); // off

  
  //Serial.println("Grrrrrrrrrr");

  // I2C for ESP32-C3 Super Mini (you used 8,9) 
  //Wire.begin(8, 9, 400000); // SDA=8, SCL=9 at 400kHz

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_20, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    // pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    // pixel.show();
    return; 
  }

  delay(3000);



  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    // pixel.setPixelColor(0, pixel.Color(255, 255, 0));
    // pixel.show();

    return;
  }

  delay(3000);


  // init LED

  

  //sensor.setROISize(8, 8);
  //sensor.setROICenter(199);
  // sensor.setDistanceMode(VL53L1X::Short);
  // sensor.setMeasurementTimingBudget(10000); // 7 ms budget
  // sensor.startContinuous(10);               // ask for ~100 Hz

  // lastTime = millis();
  // Serial.println("VL53L1X started in short 100 Hz mode (showing Δt).");

  // // green = OK init
  // pixel.setPixelColor(0, pixel.Color(0, 150, 0));
  // pixel.show();
}

void loop() {
  

  // uint16_t distance = sensor.read();
  // currentTime = millis();
  // deltaTime = currentTime - lastTime;
  // lastTime = currentTime;

  // // Serial.print(deltaTime);
  // // Serial.print(": ");
  // // Serial.println(distance);

  // // simple LED feedback:
  // // valid distances are usually < 4000 mm
  // if (distance > 0 && distance < 200) {
  //   // green if valid
  //   pixel.setPixelColor(0, pixel.Color(0, 120, 0));
  // } else {
  //   // red if out of range
  //   pixel.setPixelColor(0, pixel.Color(150, 0, 0));
  // }
  // pixel.show();

  twai_message_t msg1 = {};
  msg1.identifier = makeCANSPID(DEVICE_ID, MANUFACTURER_ID, SENSOR_BASE_API_ID, DEVICE_NUMBER);
  msg1.extd = 1;
  msg1.data_length_code = 2;


    msg1.data[0] = 50 & 0xFF;  
    msg1.data[1] = (50 >> 8) & 0xFF;
  

  esp_err_t result = twai_transmit(&msg1, pdMS_TO_TICKS(10));

  if (result == ESP_OK) {
    Serial.println("Ok");
    pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    pixel.show();
  } else {
    Serial.printf("Transmit failed: %d\n", result);
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
  }



  delay(15);

 
}

uint32_t makeCANSPID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID) << 24) | ((uint32_t)(manufacturerID) << 16) | ((uint32_t)(apiID & 0x3FF) << 6) | (deviceNumber & 0x3F);
}
