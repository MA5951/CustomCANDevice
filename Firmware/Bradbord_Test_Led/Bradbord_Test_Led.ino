#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>

VL53L1X sensor;

// ----- LED setup -----
#define LED_PIN 2        // pick any valid GPIO on your ESP32-C3
#define LED_COUNT 1

Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ----- timing -----
unsigned long lastTime = 0;  // ms
unsigned long currentTime = 0;
unsigned long deltaTime = 0;

void setup() {
  Serial.begin(115200);

  // I2C for ESP32-C3 Super Mini (you used 8,9) 
  Wire.begin(8, 9, 400000); // SDA=8, SCL=9 at 400kHz

  // init LED
  pixel.begin();
  pixel.setBrightness(40); // not full power
  pixel.show(); // off

  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize VL53L1X!");
    // show red LED to indicate failure
    pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    pixel.show();
    while (1);
  }

  //sensor.setROISize(8, 8);
  //sensor.setROICenter(199);
  sensor.setDistanceMode(VL53L1X::Short);
  sensor.setMeasurementTimingBudget(8000); // 7 ms budget
  sensor.startContinuous(8);               // ask for ~100 Hz

  lastTime = millis();
  Serial.println("VL53L1X started in short 100 Hz mode (showing Δt).");

  // green = OK init
  pixel.setPixelColor(0, pixel.Color(0, 150, 0));
  pixel.show();
}

void loop() {
  uint16_t distance = sensor.read();
  currentTime = millis();
  deltaTime = currentTime - lastTime;
  lastTime = currentTime;

  Serial.print(deltaTime);
  Serial.print(": ");
  Serial.println(distance);

  // simple LED feedback:
  // valid distances are usually < 4000 mm
  if (distance > 0 && distance < 200) {
    // green if valid
    pixel.setPixelColor(0, pixel.Color(0, 120, 0));
  } else {
    // red if out of range
    pixel.setPixelColor(0, pixel.Color(150, 0, 0));
  }
  pixel.show();

  // no delay – sensor is running continuous
}
