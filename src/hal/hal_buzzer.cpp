#include "hal_buzzer.h"

#include "../config.h"
#include "hal_pins.h"

static bool sounding = false;
static unsigned long soundEndMs = 0;

void Buzzer_Setup(void) {
  // Arduino-ESP32 2.x のLEDC API（公式サンプルは3.x向けの ledcAttachChannel を使っている）
  ledcSetup(BUZZER_LEDC_CHANNEL, 2000, BUZZER_LEDC_BITS);
  ledcAttachPin(PIN_BUZZER, BUZZER_LEDC_CHANNEL);
  Buzzer_Off();
}

void Buzzer_Beep(uint16_t frequencyHz, uint16_t durationMs) {
  ledcWriteTone(BUZZER_LEDC_CHANNEL, constrain(frequencyHz, 100, 10000));
  soundEndMs = millis() + durationMs;
  sounding = true;
}

void Buzzer_Off(void) {
  ledcWriteTone(BUZZER_LEDC_CHANNEL, 0);
  sounding = false;
}

void Buzzer_Update(unsigned long nowMs) {
  if (sounding && (long)(nowMs - soundEndMs) >= 0) {
    Buzzer_Off();
  }
}
