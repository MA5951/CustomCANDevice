#include <Wire.h>
#include <VL53L0X.h>          // Pololu library
#include "driver/twai.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

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
#define CAN_TX_GPIO GPIO_NUM_21
#define CAN_RX_GPIO GPIO_NUM_20
#define CAN_RATE    TWAI_TIMING_CONFIG_1MBITS()

// ---- CAN ID compose (CTRE/WPILib-style SPID) ----
static inline uint32_t makeCANSPID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID) << 24) | ((uint32_t)(manufacturerID) << 16) |
         ((uint32_t)(apiID & 0x3FF) << 6) | (deviceNumber & 0x3F);
}

// ---- Customize these as you like ----
#define DEVICE_ID        0x0A
#define MANUFACTURER_ID  0x08
#define SENSOR_BASE_API  0x0301
#define DEVICE_NUMBER    30

// ---------- Helpers ----------
static inline void u16_to_le(uint16_t v, uint8_t* p) { p[0] = v & 0xFF; p[1] = (v >> 8) & 0xFF; }

// ---------- Globals ----------
VL53L0X loxA;
VL53L0X loxB;

static SemaphoreHandle_t g_i2cMutex;

// latest values shared to CAN task
typedef struct {
  uint16_t dist_mm;   // 0..65534 valid, 0xFFFF invalid
  uint8_t  status;    // 0 = valid; Pololu exposes timeoutOccurred() separately
  uint64_t t_us;      // timestamp when captured
} Sample;

static volatile Sample sA = {0xFFFF, 0xFF, 0};
static volatile Sample sB = {0xFFFF, 0xFF, 0};

static TaskHandle_t g_canTaskHandle = nullptr;

// ---------- Bring-up & readdress (Pololu) ----------
static void bringUpAndReaddress(int xshutPin, VL53L0X& sensor, uint8_t newAddr) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, LOW);
  vTaskDelay(pdMS_TO_TICKS(2));
  digitalWrite(xshutPin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(10)); // boot

  xSemaphoreTake(g_i2cMutex, portMAX_DELAY);
  sensor.setTimeout(50);         // ms safety
  if (!sensor.init()) {
    xSemaphoreGive(g_i2cMutex);
    Serial.println(F("ERROR: VL53L0X init() failed (check wiring/power)"));
    while (true) vTaskDelay(pdMS_TO_TICKS(100));
  }

  // High-speed timing budget ~20,000 us (≈50 Hz continuous potential)
  sensor.setMeasurementTimingBudget(20000); // in microseconds

  // Assign new I2C address
  sensor.setAddress(newAddr);
  xSemaphoreGive(g_i2cMutex);
  vTaskDelay(pdMS_TO_TICKS(2));
}

// ---------- Sensor tasks (continuous mode) ----------
static void TaskSensor(void* arg) {
  const bool isA = (bool)arg;
  VL53L0X* s = isA ? &loxA : &loxB;

  // Stagger one sensor to reduce IR cross-talk
  if (!isA) vTaskDelay(pdMS_TO_TICKS(10));

  // Start back-to-back continuous (period=0 => relies purely on timing budget)
  xSemaphoreTake(g_i2cMutex, portMAX_DELAY);
  s->startContinuous(0);
  xSemaphoreGive(g_i2cMutex);

  // Poll quickly for new distance; each read is a short I2C transaction
  uint16_t last_mm = 0xFFFF;

  for (;;) {
    uint16_t d;
    bool timeout = false;

    xSemaphoreTake(g_i2cMutex, portMAX_DELAY);
    d = s->readRangeContinuousMillimeters(); // gets latest sample
    timeout = s->timeoutOccurred();
    xSemaphoreGive(g_i2cMutex);

    uint8_t st = 0; // 0 = ok
    if (timeout) { st = 1; d = 0xFFFF; }

    // Only signal CAN when value actually updated (optional, reduces spam)
    if (d != last_mm || timeout) {
      uint64_t now = (uint64_t)esp_timer_get_time();
      if (isA) {
        sA.dist_mm = (d <= 65534u) ? d : (uint16_t)65534u;
        sA.status  = st;
        sA.t_us    = now;
      } else {
        sB.dist_mm = (d <= 65534u) ? d : (uint16_t)65534u;
        sB.status  = st;
        sB.t_us    = now;
      }
      last_mm = d;

      if (g_canTaskHandle) {
        xTaskNotifyGive(g_canTaskHandle); // wake CAN task
      }
    }

    // Budget is ~20 ms; reading faster than that returns "latest".
    // Sleep a tick so we don't hog CPU if nothing new.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- CAN TX + printing task ----------
static void TaskCAN(void* arg) {
  (void)arg;
  uint64_t last_us = esp_timer_get_time();

  for (;;) {
    // Wait for any sensor update
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Snapshot
    Sample a = { (uint16_t)sA.dist_mm, (uint8_t)sA.status, (uint64_t)sA.t_us };
    Sample b = { (uint16_t)sB.dist_mm, (uint8_t)sB.status, (uint64_t)sB.t_us };

    // Pack CAN frame (A:2+1, B:2+1)
    twai_message_t msg = {};
    msg.identifier = makeCANSPID(DEVICE_ID, MANUFACTURER_ID, SENSOR_BASE_API, DEVICE_NUMBER);
    msg.extd = 1;
    msg.data_length_code = 6;
    u16_to_le(a.dist_mm, &msg.data[0]);  msg.data[2] = a.status;
    u16_to_le(b.dist_mm, &msg.data[3]);  msg.data[5] = b.status;

    // Non-blocking-ish transmit; drop if bus busy to keep latency tiny
    (void)twai_transmit(&msg, pdMS_TO_TICKS(1));

    // ---- PRINT sensors data (concise) ----
    uint64_t now_us = esp_timer_get_time();
    uint32_t dt_ms  = (uint32_t)((now_us - last_us) / 1000);
    last_us = now_us;

    uint32_t ageA_ms = (uint32_t)((now_us - a.t_us) / 1000);
    uint32_t ageB_ms = (uint32_t)((now_us - b.t_us) / 1000);

    //Serial.print("Δt="); Serial.print(dt_ms); Serial.print("ms | A: ");
    // if (a.status == 0 && a.dist_mm != 0xFFFF) { Serial.println(a.dist_mm);
    //  }
    // else { Serial.print("INV"); }
    // Serial.print(" (S="); Serial.print(a.status); Serial.print(", age=");
    // Serial.print(ageA_ms); Serial.print("ms)");

    // Serial.print(" | B: ");
    // if (b.status == 0 && b.dist_mm != 0xFFFF) { Serial.println(b.dist_mm);
    // }
    // else { ///Serial.print("INV");
    // }
    // Serial.print(" (S="); Serial.print(b.status); Serial.print(", age=");
    // Serial.print(ageB_ms); Serial.print("ms)");
    // Serial.println();
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 1500) {}

  // I2C @ 400 kHz
  Wire.begin(SDA_PIN, SCL_PIN, 400000);

  // Hold both sensors off-bus
  pinMode(XSHUT_A, OUTPUT);
  pinMode(XSHUT_B, OUTPUT);
  digitalWrite(XSHUT_A, LOW);
  digitalWrite(XSHUT_B, LOW);
  vTaskDelay(pdMS_TO_TICKS(10));

  // Mutex for I2C
  g_i2cMutex = xSemaphoreCreateMutex();
  configASSERT(g_i2cMutex != NULL);

  // Bring up & readdress using Pololu API
  bringUpAndReaddress(XSHUT_A, loxA, ADDR_A);
  bringUpAndReaddress(XSHUT_B, loxB, ADDR_B);
  Serial.println(F("VL53L0X (Pololu) at 0x30 and 0x31, continuous mode planned."));

  // TWAI init
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = CAN_RATE;
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println(F("TWAI driver install FAILED"));
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  if (twai_start() != ESP_OK) {
    Serial.println(F("TWAI start FAILED"));
    while (true) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  Serial.println(F("TWAI started @ 1Mbps"));

  // Tasks
  configASSERT(xTaskCreatePinnedToCore(TaskSensor, "SensorA", 4096, (void*)true,  6, NULL, tskNO_AFFINITY) == pdPASS);
  configASSERT(xTaskCreatePinnedToCore(TaskSensor, "SensorB", 4096, (void*)false, 6, NULL, tskNO_AFFINITY) == pdPASS);
  configASSERT(xTaskCreatePinnedToCore(TaskCAN,    "CAN-TX",  4096, NULL,         5, &g_canTaskHandle, tskNO_AFFINITY) == pdPASS);

  Serial.println(F("Mode: CONTINUOUS (Pololu). Timing budget 20 ms. Expect ~100 Hz CAN frames (alt A/B ~10 ms)."));
}

void loop() {
  // RTOS takes it from here
}
