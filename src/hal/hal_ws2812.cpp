#include "hal_ws2812.h"

#include <Freenove_WS2812_Lib_for_ESP32.h>

#include "../config.h"
#include "hal_pins.h"

static Freenove_ESP32_WS2812 strip(WS2812_COUNT, PIN_WS2812, WS2812_RMT_CHANNEL, TYPE_GRB);
static bool active = false;

bool Ws2812_Begin(void) {
  if (active) {
    return true;
  }
  if (!strip.begin()) {
    return false;
  }
  strip.setBrightness(WS2812_BRIGHTNESS);
  active = true;
  Ws2812_Off();
  return true;
}

bool Ws2812_IsActive(void) {
  return active;
}

void Ws2812_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
  if (!active || index >= WS2812_COUNT) return;
  strip.setLedColorData(index, r, g, b);
}

void Ws2812_Fill(uint8_t r, uint8_t g, uint8_t b) {
  if (!active) return;
  strip.setAllLedsColorData(r, g, b);
}

void Ws2812_Show(void) {
  if (!active) return;
  strip.show();
}

void Ws2812_Off(void) {
  Ws2812_Fill(0, 0, 0);
  Ws2812_Show();
}
