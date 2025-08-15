#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define I2C_SDA 8   // ESP32-C3 Super Mini SDA
#define I2C_SCL 9   // ESP32-C3 Super Mini SCL

Adafruit_VL53L0X lox;

void setup() {
  Serial.begin(115200);
  // Optional: wait a moment for the USB CDC serial to connect
  unsigned long start = millis();
  while (!Serial && millis() - start < 2000) { /* wait up to 2s */ }

  // Init I2C on GPIO 8/9 and bump clock (sensor is fine with 400 kHz)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.println("\nVL53L0X init...");

  if (!lox.begin()) {
    Serial.println("ERROR: VL53L0X not found. Check wiring and power (3.3V), address 0x29.");
    while (true) { delay(1000); }
  }

  Serial.println("VL53L0X ready.");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); // 'true' prints debug data

  if (measure.RangeStatus != 4) {   // 4 = out of range
      Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Out of range");
  }

  delay(100); // 10 Hz updates
}
