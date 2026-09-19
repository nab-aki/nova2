#include "hal_ultrasonic.h"

#include <driver/gpio.h>

#include "../config.h"
#include "hal_pins.h"

#define SOUND_VELOCITY_M_S   340   // 公式サンプル 02.1_Ultrasonic_Ranging と同じ

static volatile unsigned long echoRiseUs = 0;
static volatile unsigned long echoFallUs = 0;
static volatile bool echoDone = false;

static bool waitingEcho = false;
static unsigned long lastTriggerMs = 0;
static float lastDistanceCm = -1.0f;

// エコー信号の変化ごとに時刻を記録する
static void IRAM_ATTR OnEchoChange(void) {
  if (gpio_get_level((gpio_num_t)PIN_SONIC_ECHO)) {
    echoRiseUs = micros();
  } else {
    echoFallUs = micros();
    echoDone = true;
  }
}

void Ultrasonic_Setup(void) {
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pinMode(PIN_SONIC_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SONIC_ECHO), OnEchoChange, CHANGE);
}

bool Ultrasonic_Update(unsigned long nowMs) {
  if (!waitingEcho) {
    if (nowMs - lastTriggerMs < ULTRASONIC_INTERVAL_MS) {
      return false;
    }
    lastTriggerMs = nowMs;
    echoDone = false;
    digitalWrite(PIN_SONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONIC_TRIG, LOW);
    waitingEcho = true;
    return false;
  }

  if (echoDone) {
    unsigned long pulseUs = echoFallUs - echoRiseUs;
    lastDistanceCm = (float)pulseUs * SOUND_VELOCITY_M_S / 2 / 10000.0f;
    waitingEcho = false;
    return true;
  }
  if (nowMs - lastTriggerMs >= ULTRASONIC_TIMEOUT_MS) {
    lastDistanceCm = -1.0f;   // 反応なし
    waitingEcho = false;
    return true;
  }
  return false;
}

float Ultrasonic_GetCm(void) {
  return lastDistanceCm;
}
