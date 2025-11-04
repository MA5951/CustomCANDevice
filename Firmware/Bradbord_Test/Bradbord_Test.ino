#include <Wire.h>
#include <VL53L1X.h>

VL53L1X sensor;

unsigned long lastTime = 0;  // ms
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9,400000); // SDA=6, SCL=7 for ESP32-C3 Super Mini


  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L1X!");
    while (1);
  }

  //sensor.setROISize(8, 8);        // smaller = narrower field of view (~13°)
  //sensor.setROICenter(199);       // 199 = default center (middle of sensor)
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(7000); // 10 ms per measurement
  sensor.startContinuous(7); // 100 Hz mode


  lastTime = millis();
  Serial.println("VL53L1X started in short 100 Hz mode (showing Δt).");
}

void loop() {
  uint16_t distance = sensor.read();
  currentTime = millis();
  deltaTime = currentTime - lastTime;
  lastTime = currentTime;


  Serial.print(deltaTime);
  Serial.print(": ");
  Serial.println(distance);

  //delay(10); // keeps loop rate near 100 Hz
}
