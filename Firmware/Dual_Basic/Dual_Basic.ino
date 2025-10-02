#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "driver/twai.h"

// ---------- I2C (ESP32-C3 Super Mini) ----------
#define SDA_PIN   8
#define SCL_PIN   9

// ---------- XSHUT pins (per sensor) ----------
#define XSHUT_A   1
#define XSHUT_B   7

// ---------- I2C runtime addresses ----------
#define ADDR_A    0x30
#define ADDR_B    0x31

// ---------- CAN (TWAI) config ----------
#define CAN_TX_GPIO GPIO_NUM_21   // ESP32-C3 Super Mini: TXD on GPIO 21
#define CAN_RX_GPIO GPIO_NUM_20   // ESP32-C3 Super Mini: RXD on GPIO 20
#define CAN_RATE    TWAI_TIMING_CONFIG_1MBITS()

// Compose a CTRE/WPILib-style SPID (extended identifier)
static inline uint32_t makeCANSPID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID) << 24) | ((uint32_t)(manufacturerID) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) | (deviceNumber & 0x3F);
}

// ---- Customize these as you like ----
#define DEVICE_ID        0x0A
#define MANUFACTURER_ID  0x08
#define SENSOR_BASE_API  0x0301
#define DEVICE_NUMBER    50

Adafruit_VL53L0X loxA;
Adafruit_VL53L0X loxB;

// Helper: power up one sensor at 0x29, configure, then readdress to newAddr
void bringUpAndReaddress(int xshutPin, Adafruit_VL53L0X& sensor, uint8_t newAddr) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  delay(2);
  digitalWrite(xshutPin, HIGH);
  delay(10); // boot

  if (!sensor.begin(0x29, false, &Wire)) {
    Serial.println(F("ERROR: begin() failed at 0x29 (check wiring/power)"));
    while (true) delay(100);
  }

  // For 50 Hz, prefer HIGH_SPEED (short timing budget)
  sensor.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);

  sensor.setAddress(newAddr);
  delay(5);
}

static inline void u16_to_le(uint16_t v, uint8_t* p) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) { } // brief wait for CDC

  // I2C on custom pins @ 400 kHz
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  // Hold both sensors off-bus
  pinMode(XSHUT_A, OUTPUT);
  pinMode(XSHUT_B, OUTPUT);
  digitalWrite(XSHUT_A, LOW);
  digitalWrite(XSHUT_B, LOW);
  delay(10);

  // Bring up and readdress
  bringUpAndReaddress(XSHUT_A, loxA, ADDR_A);
  bringUpAndReaddress(XSHUT_B, loxB, ADDR_B);
  Serial.println(F("Two VL53L0X sensors ready at 0x30 and 0x31"));

  // ---- TWAI (CAN) init ----
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = CAN_RATE; // 1 Mbps
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println(F("TWAI driver installed"));
  } else {
    Serial.println(F("TWAI driver install FAILED"));
    while (true) delay(1000);
  }
  if (twai_start() == ESP_OK) {
    Serial.println(F("TWAI started"));
  } else {
    Serial.println(F("TWAI start FAILED"));
    while (true) delay(1000);
  }
}

void readSensor(Adafruit_VL53L0X& sensor, uint16_t& dist_mm, uint8_t& status) {
  VL53L0X_RangingMeasurementData_t m;
  sensor.rangingTest(&m, false);  // single-shot
  status = m.RangeStatus;         // 0 = valid
  if (status == 0) {
    dist_mm = (uint16_t)m.RangeMilliMeter;
  } else {
    dist_mm = 0; // encode 0 on invalid frames
  }
}

void loop() {
  static uint32_t last = 0;
  const uint32_t period_ms = 20; // 50 Hz

  uint32_t now = millis();
  if ((now - last) < period_ms) return;
  last = now;

  // Read A then B with a small stagger to reduce cross-talk
  uint16_t a_mm = 0, b_mm = 0;
  uint8_t  a_stat = 0xFF, b_stat = 0xFF;

  readSensor(loxA, a_mm, a_stat);
  delay(10);
  readSensor(loxB, b_mm, b_stat);

  // Pack CAN frame
  twai_message_t msg = {};
  msg.identifier = makeCANSPID(DEVICE_ID, MANUFACTURER_ID, SENSOR_BASE_API, DEVICE_NUMBER);
  msg.extd = 1;                 // extended ID
  msg.data_length_code = 6;     // A:2+1, B:2+1 = 6 bytes

  u16_to_le(a_mm, &msg.data[0]);  // bytes 1-2: A distance (LE)
  msg.data[2] = a_stat;           // byte 3:    A result/status
  u16_to_le(b_mm, &msg.data[3]);  // bytes 4-5: B distance (LE)
  msg.data[5] = b_stat;           // byte 6:    B result/status

  esp_err_t res = twai_transmit(&msg, pdMS_TO_TICKS(5));
  if (res != ESP_OK) {
    // Optional debug print; keep it lightweight at 50 Hz
    Serial.printf("CAN tx fail: %d\n", res);
  }

  // (Optional) serial monitor for quick sanity
  // Comment out if you need absolutely steady 50 Hz
  // Serial.print("A: ");
  // if (a_stat == 0) { Serial.print(a_mm); Serial.print("mm"); } else { Serial.print("S="); Serial.print(a_stat); }
  // Serial.print(" | B: ");
  // if (b_stat == 0) { Serial.print(b_mm); Serial.print("mm"); } else { Serial.print("S="); Serial.print(b_stat); }
  // Serial.println();
}
