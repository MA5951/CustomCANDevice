#include <Adafruit_NeoPixel.h>
#include "driver/twai.h"

#define LED_PIN     1               // GPIO pin connected to the data line
#define LED_COUNT   400             // Total number of LEDs physically
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

int activeLedCount = LED_COUNT;     // Number of LEDs currently used

// Variables for patterns
int firstHue = 0;
bool ledOn = false;
unsigned long lastChange = 0;

// ---------- Function to set how many LEDs are active ----------
void setActiveLedCount(int count) {
  activeLedCount = constrain(count, 0, LED_COUNT);  // Keep within bounds
}

// ---------- Helper to set all LEDs to a color ----------
void setSingleColor(uint8_t g, uint8_t r, uint8_t b) {
  for (int i = 0; i < LED_COUNT; i++) {
    if (i < activeLedCount) {
      strip.setPixelColor(i, strip.Color(r, g, b));
    } else {
      strip.setPixelColor(i, 0); // Turn off unused LEDs
    }
  }
  strip.show();
}

// ---------- Rainbow pattern ----------
void rainbowColorPattern() {
  for (int i = 0; i < LED_COUNT; i++) {
    if (i < activeLedCount) {
      int hue = (firstHue + (i * 180 / activeLedCount)) % 180;
      uint32_t color = strip.ColorHSV(hue * 65536 / 180, 255, 128);
      strip.setPixelColor(i, color);
    } else {
      strip.setPixelColor(i, 0);
    }
  }
  firstHue = (firstHue + 3) % 180;
  strip.show();
}

// ---------- Blink between two colors ----------
void blinkColorPattern(unsigned long intervalMs, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2) {
  unsigned long current = millis();
  if (current - lastChange > intervalMs) {
    ledOn = !ledOn;
    lastChange = current;
  }

  if (ledOn) {
    setSingleColor(g1, r1, b1);
  } else {
    setSingleColor(g2, r2, b2);
  }
}

// ---------- Smooth wave pattern ----------
void smoothWaveColorPattern(int numColors, float period, float speed, uint8_t colors[][3]) {
  float elapsed = millis() / 1000.0;

  for (int i = 0; i < LED_COUNT; i++) {
    if (i < activeLedCount) {
      float position = (float)i / activeLedCount + (elapsed * speed / period);
      float progress = position - (int)position;

      int startIdx = ((int)position) % numColors;
      int endIdx = (startIdx + 1) % numColors;

      uint8_t r = colors[startIdx][0] + (colors[endIdx][0] - colors[startIdx][0]) * progress;
      uint8_t g = colors[startIdx][1] + (colors[endIdx][1] - colors[startIdx][1]) * progress;
      uint8_t b = colors[startIdx][2] + (colors[endIdx][2] - colors[startIdx][2]) * progress;

      strip.setPixelColor(i, strip.Color(r, g, b));
    } else {
      strip.setPixelColor(i, 0);
    }
  }

  strip.show();
}

// ---------- Setup and Loop ----------
void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  setActiveLedCount(400); // Turn ON only the first 400 LEDs

  setSingleColor(0,0,0);
}

void loop() {
  
}
