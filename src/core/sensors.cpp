#include "sensors.h"

#include "../config.h"
#include "../hal/hal_battery.h"
#include "../hal/hal_light.h"
#include "../hal/hal_log.h"
#include "../hal/hal_track.h"
#include "../hal/hal_ultrasonic.h"
#include "neck.h"

static SensorData data;
static unsigned long lastLightMs = 0;
static unsigned long lastTrackMs = 0;
static unsigned long lastBatteryMs = 0;

// エコーなしが続いていないかの見張り（センサーの断線・故障に気づくため。動作は変えない）
static bool noEchoRunning = false;
static unsigned long noEchoSinceMs = 0;
static bool noEchoWarned = false;

void Sensors_Setup(void) {
  data.distanceRawCm = -1.0f;
  data.distanceCm = -1.0f;
  data.distanceValid = false;
  data.distanceUpdated = false;
  data.distanceNeckSteady = false;
  data.distanceMs = 0;
  data.lightAdc = Light_Read();
  data.track = Track_Read();
  data.trackUpdated = false;
  data.batteryAdc = Battery_ReadAdc();
  data.batteryV = Battery_AdcToVoltage(data.batteryAdc);
}

// エコーが返らない状態が続いていないかを見る
static void WatchNoEcho(bool noEcho, unsigned long nowMs) {
  if (!noEcho) {
    noEchoRunning = false;
    noEchoWarned = false;
    return;
  }
  if (!noEchoRunning) {
    noEchoRunning = true;
    noEchoSinceMs = nowMs;
    return;
  }
  if (!noEchoWarned && nowMs - noEchoSinceMs >= ULTRASONIC_NO_ECHO_WARN_MS) {
    noEchoWarned = true;
    Log_Printf("超音波", "エコーなしが%lu秒続いています（配線・センサーを確認してください）",
               (unsigned long)(ULTRASONIC_NO_ECHO_WARN_MS / 1000));
  }
}

void Sensors_Update(unsigned long nowMs) {
  // 超音波：測距が1回完了したときだけ更新する
  data.distanceUpdated = Ultrasonic_Update(nowMs);
  if (data.distanceUpdated) {
    data.distanceMs = nowMs;
    data.distanceNeckSteady = Neck_IsSteady(nowMs);
    data.distanceRawCm = Ultrasonic_GetCm();
    data.distanceValid = data.distanceRawCm >= ULTRASONIC_MIN_CM && data.distanceRawCm <= ULTRASONIC_MAX_CM;
    data.distanceCm = data.distanceValid ? data.distanceRawCm : -1.0f;
    WatchNoEcho(data.distanceRawCm < 0.0f, nowMs);
  }

  if (nowMs - lastLightMs >= LIGHT_READ_INTERVAL_MS) {
    lastLightMs = nowMs;
    data.lightAdc = Light_Read();
  }
  data.trackUpdated = (nowMs - lastTrackMs >= TRACK_READ_INTERVAL_MS);
  if (data.trackUpdated) {
    lastTrackMs = nowMs;
    data.track = Track_Read();
  }
  if (nowMs - lastBatteryMs >= BATTERY_READ_INTERVAL_MS) {
    lastBatteryMs = nowMs;
    data.batteryAdc = Battery_ReadAdc();
    data.batteryV = Battery_AdcToVoltage(data.batteryAdc);
  }
}

const SensorData &Sensors_Get(void) {
  return data;
}
