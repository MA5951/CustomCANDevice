#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>
#include "driver/twai.h"

// =====================
// FRC CAN Identity (29-bit)
// deviceType(5) | manufacturer(8) | api(10) | deviceId(6)
// =====================
static constexpr uint8_t  kDeviceType   = 5;  // Distance Sensor
static constexpr uint8_t  kManufacturer = 8;  // Team Use

// API IDs (10-bit)
static constexpr uint16_t API_STATUS      = 0x300; // periodic status (8 bytes)
static constexpr uint16_t API_DISTANCE    = 0x301; // periodic distance (2 bytes)

// Requests/commands
static constexpr uint16_t API_RTR_NAME    = 0x310; // RTR -> respond name (8)
static constexpr uint16_t API_SET_NAME    = 0x311; // write name (8)
static constexpr uint16_t API_SET_ID      = 0x312; // write newId (1)
static constexpr uint16_t API_IDENTIFY    = 0x313; // write seconds (1)
static constexpr uint16_t API_SET_ROI     = 0x314; // write w,h,center (3)
static constexpr uint16_t API_SET_TIMING  = 0x315; // write budUs(u16), interMs(u16), txMs(u16), mode(u8) (7)
static constexpr uint16_t API_RTR_CONFIG  = 0x316; // RTR -> respond config (8)

// HW
VL53L1X sensor;
Preferences prefs;

#define LED_PIN   3
#define LED_COUNT 1
Adafruit_NeoPixel pixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------
// Persistent config
// ---------------------
static uint8_t  g_canId          = 26;        // 0..63
static char     g_name[9]        = "MACam";   // 8 chars max
static uint8_t  g_roiW           = 8;
static uint8_t  g_roiH           = 8;
static uint8_t  g_roiCenter      = 199;
static uint16_t g_budgetUs       = 10000;     // 10ms
static uint16_t g_interMs        = 13;        // sensor loop
static uint16_t g_txMs           = 18;        // CAN publish => 50Hz
static uint8_t  g_mode           = 0;         // 0=Short,1=Long

// runtime
static volatile uint16_t g_lastDistance = 0;
static volatile uint32_t g_identifyUntilMs = 0;

static volatile bool g_applySensorConfig = false;

static portMUX_TYPE g_cfgMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------------
// Utils
// ---------------------
static inline uint32_t makeFrcCanId(uint8_t deviceType, uint8_t manufacturer, uint16_t apiId, uint8_t deviceId) {
  return ((uint32_t)(deviceType & 0x1F) << 24)
       | ((uint32_t)manufacturer << 16)
       | ((uint32_t)(apiId & 0x3FF) << 6)
       | ((uint32_t)(deviceId & 0x3F));
}

static inline void parseFrcCanId(uint32_t id, uint8_t &deviceType, uint8_t &manufacturer, uint16_t &apiId, uint8_t &deviceId) {
  deviceType   = (id >> 24) & 0x1F;
  manufacturer = (id >> 16) & 0xFF;
  apiId        = (id >> 6)  & 0x3FF;
  deviceId     = id & 0x3F;
}

static inline uint16_t u16le(const uint8_t* d, int off) {
  return (uint16_t)d[off] | ((uint16_t)d[off+1] << 8);
}

static inline void putU16LE(uint8_t* d, int off, uint16_t v) {
  d[off] = (uint8_t)(v & 0xFF);
  d[off+1] = (uint8_t)((v >> 8) & 0xFF);
}

static inline uint8_t clampU8(int v, int lo, int hi) {
  if (v < lo) return (uint8_t)lo;
  if (v > hi) return (uint8_t)hi;
  return (uint8_t)v;
}

static inline uint16_t clampU16(int v, int lo, int hi) {
  if (v < lo) return (uint16_t)lo;
  if (v > hi) return (uint16_t)hi;
  return (uint16_t)v;
}

static void setLed(bool on) {
  pixel.setPixelColor(0, on ? pixel.Color(0, 255, 0) : pixel.Color(0, 0, 0));
  pixel.show();
}

// ---------------------
// Config storage
// ---------------------
static void saveConfig() {
  prefs.putUChar("canId", g_canId);
  prefs.putString("name", String(g_name));
  prefs.putUChar("roiW", g_roiW);
  prefs.putUChar("roiH", g_roiH);
  prefs.putUChar("roiC", g_roiCenter);
  prefs.putUShort("budUs", g_budgetUs);
  prefs.putUShort("intMs", g_interMs);
  prefs.putUShort("txMs",  g_txMs);
  prefs.putUChar("mode",   g_mode);
}

static void loadConfig() {
  prefs.begin("macam", false);

  g_canId = prefs.getUChar("canId", g_canId) & 0x3F;

  String n = prefs.getString("name", g_name);
  memset(g_name, 0, sizeof(g_name));
  n = n.substring(0, 8);
  n.toCharArray(g_name, sizeof(g_name));

  g_roiW      = prefs.getUChar("roiW", g_roiW);
  g_roiH      = prefs.getUChar("roiH", g_roiH);
  g_roiCenter = prefs.getUChar("roiC", g_roiCenter);

  g_budgetUs  = prefs.getUShort("budUs", g_budgetUs);
  g_interMs   = prefs.getUShort("intMs", g_interMs);
  g_txMs      = prefs.getUShort("txMs",  g_txMs);
  g_mode      = prefs.getUChar("mode",   g_mode);

  if (g_txMs < 5) g_txMs = 5;
  if (g_interMs < 10) g_interMs = 10;
  if (g_mode > 1) g_mode = 0;
}

// ---------------------
// Sensor apply (ONLY in sensor task)
// ---------------------
static void applySensorConfig() {
  sensor.stopContinuous();
  sensor.setROISize(g_roiW, g_roiH);
  sensor.setROICenter(g_roiCenter);
  sensor.setDistanceMode(g_mode == 0 ? VL53L1X::Short : VL53L1X::Long);
  sensor.setMeasurementTimingBudget(g_budgetUs);
  sensor.startContinuous(g_interMs);
}

// ---------------------
// CAN send helpers
// ---------------------
static void canSendDistance(uint8_t id, uint16_t dist) {
  twai_message_t msg = {};
  msg.identifier = makeFrcCanId(kDeviceType, kManufacturer, API_DISTANCE, id);
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = 2;
  msg.data[0] = dist & 0xFF;
  msg.data[1] = (dist >> 8) & 0xFF;
  twai_transmit(&msg, 0);
}

static void canSendStatus(uint8_t id, uint16_t dist, uint8_t mode) {
  twai_message_t msg = {};
  msg.identifier = makeFrcCanId(kDeviceType, kManufacturer, API_STATUS, id);
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = 8;
  msg.data[0] = dist & 0xFF;
  msg.data[1] = (dist >> 8) & 0xFF;
  msg.data[2] = 0; // flags
  msg.data[3] = 0;
  msg.data[4] = 2; // fw major
  msg.data[5] = 2; // fw minor
  msg.data[6] = 0; // fw patch
  msg.data[7] = mode;
  twai_transmit(&msg, 0);
}

static void canRespondName(uint8_t id) {
  twai_message_t msg = {};
  msg.identifier = makeFrcCanId(kDeviceType, kManufacturer, API_RTR_NAME, id);
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = 8;
  for (int i = 0; i < 8; i++) msg.data[i] = (uint8_t)g_name[i];
  twai_transmit(&msg, 0);
}

static void canRespondConfig(uint8_t id) {
  twai_message_t msg = {};
  msg.identifier = makeFrcCanId(kDeviceType, kManufacturer, API_RTR_CONFIG, id);
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = 8;

  msg.data[0] = g_roiW;
  msg.data[1] = g_roiH;
  msg.data[2] = g_roiCenter;
  msg.data[3] = g_mode;

  uint16_t budgetMs = (uint16_t)(g_budgetUs / 1000);
  if (budgetMs > 255) budgetMs = 255;
  msg.data[4] = (uint8_t)budgetMs;

  msg.data[5] = (uint8_t)((g_interMs > 255) ? 255 : g_interMs);
  msg.data[6] = (uint8_t)((g_txMs   > 255) ? 255 : g_txMs);
  msg.data[7] = 0;

  twai_transmit(&msg, 0);
}

// ---------------------
// CAN RX handler (runs in RX task)
// ---------------------
static void handleRx(const twai_message_t &rx) {
  if (!rx.extd) return;

  uint8_t devType, mfr, devId;
  uint16_t api;
  parseFrcCanId(rx.identifier, devType, mfr, api, devId);

  if (devType != kDeviceType || mfr != kManufacturer) return;

  // read current id atomically
  uint8_t curId;
  portENTER_CRITICAL(&g_cfgMux);
  curId = g_canId;
  portEXIT_CRITICAL(&g_cfgMux);

  if (devId != curId) return;

  // RTR: respond immediately
  if (rx.rtr) {
    if (api == API_RTR_NAME && rx.data_length_code == 8) canRespondName(curId);
    else if (api == API_RTR_CONFIG && rx.data_length_code == 8) canRespondConfig(curId);
    return;
  }

  // Commands
  if (api == API_SET_NAME && rx.data_length_code == 8) {
    portENTER_CRITICAL(&g_cfgMux);
    memset(g_name, 0, sizeof(g_name));
    for (int i = 0; i < 8; i++) g_name[i] = (char)rx.data[i];
    saveConfig();
    portEXIT_CRITICAL(&g_cfgMux);
    return;
  }

  if (api == API_SET_ID && rx.data_length_code >= 1) {
    uint8_t newId = rx.data[0] & 0x3F;
    portENTER_CRITICAL(&g_cfgMux);
    if (newId != g_canId) {
      g_canId = newId;
      saveConfig();
    }
    portEXIT_CRITICAL(&g_cfgMux);
    return;
  }

  if (api == API_IDENTIFY && rx.data_length_code >= 1) {
    uint8_t seconds = rx.data[0];
    if (seconds == 0) seconds = 2;
    g_identifyUntilMs = millis() + (uint32_t)seconds * 1000u;
    return;
  }

  if (api == API_SET_ROI && rx.data_length_code >= 3) {
    portENTER_CRITICAL(&g_cfgMux);
    g_roiW = clampU8(rx.data[0], 4, 16);
    g_roiH = clampU8(rx.data[1], 4, 16);
    g_roiCenter = rx.data[2];
    saveConfig();
    g_applySensorConfig = true;
    portEXIT_CRITICAL(&g_cfgMux);
    return;
  }

  if (api == API_SET_TIMING && rx.data_length_code >= 7) {
    uint16_t budUs = u16le(rx.data, 0);
    uint16_t intMs = u16le(rx.data, 2);
    uint16_t txMs  = u16le(rx.data, 4);
    uint8_t  mode  = rx.data[6];

    portENTER_CRITICAL(&g_cfgMux);
    g_budgetUs = clampU16(budUs, 5000, 65000);
    g_interMs  = clampU16(intMs, 10,   65000);
    g_txMs     = clampU16(txMs,  5,    65000);
    g_mode     = (mode > 1) ? 0 : mode;
    saveConfig();
    g_applySensorConfig = true;
    portEXIT_CRITICAL(&g_cfgMux);
    return;
  }
}

// =====================
// FreeRTOS tasks
// =====================
static TaskHandle_t rxTaskH = nullptr;
static TaskHandle_t sensorTaskH = nullptr;
static TaskHandle_t txTaskH = nullptr;

void canRxTask(void*) {
  twai_message_t rx;
  for (;;) {
    // block briefly so this task is efficient
    if (twai_receive(&rx, pdMS_TO_TICKS(50)) == ESP_OK) {
      handleRx(rx);
    }
  }
}

void sensorTask(void*) {
  // all I2C / sensor operations live here
  for (;;) {
    bool apply = false;

    portENTER_CRITICAL(&g_cfgMux);
    apply = g_applySensorConfig;
    g_applySensorConfig = false;
    portEXIT_CRITICAL(&g_cfgMux);

    if (apply) {
      applySensorConfig();
    }

    if (sensor.dataReady()) {
      uint16_t d = sensor.read();
      g_lastDistance = d;
    }

    vTaskDelay(pdMS_TO_TICKS(2)); // small yield
  }
}

void canTxTask(void*) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastStatusMs = millis();

  for (;;) {
    uint8_t id;
    uint8_t mode;
    uint16_t txMs;

    portENTER_CRITICAL(&g_cfgMux);
    id = g_canId;
    mode = g_mode;
    txMs = g_txMs;
    portEXIT_CRITICAL(&g_cfgMux);

    const uint16_t dist = g_lastDistance;

    // distance at exact tx period (default 20ms => 50Hz)
    canSendDistance(id, dist);

    // status ~5Hz (used for discovery)
    const uint32_t now = millis();
    if ((uint32_t)(now - lastStatusMs) >= 200) {
      lastStatusMs = now;
      canSendStatus(id, dist, mode);
    }

    // identify blink
    if (g_identifyUntilMs > now) {
      bool on = ((now / 100) % 2) == 0;
      setLed(on);
    } else {
      setLed(false);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(txMs));
  }
}

// =====================
// Arduino setup
// =====================
void setup() {
  Serial.begin(115200);
  delay(200);

  pixel.begin();
  setLed(false);

  // I2C pins for ESP32-C3 Super Mini
  Wire.begin(8, 9, 400000);

  loadConfig();

  sensor.setTimeout(50);
  if (!sensor.init()) {
    Serial.println("VL53L1X init failed");
  } else {
    applySensorConfig();
  }

  // TWAI (TX=GPIO21, RX=GPIO20)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_20, TWAI_MODE_NORMAL);
  twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI install failed");
    return;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    return;
  }

  // FreeRTOS tasks (ESP32-C3 is single-core; no pinning)
  xTaskCreate(canRxTask,   "CAN_RX",   4096, nullptr, 3, &rxTaskH);
  xTaskCreate(sensorTask,  "SENSOR",   4096, nullptr, 2, &sensorTaskH);
  xTaskCreate(canTxTask,   "CAN_TX",   4096, nullptr, 2, &txTaskH);

  Serial.printf("MACam FreeRTOS started: ID=%u, name=%s, TX=%ums\n", g_canId, g_name, g_txMs);
}

void loop() {
  // not used (tasks handle everything)
  vTaskDelay(pdMS_TO_TICKS(1000));
}
