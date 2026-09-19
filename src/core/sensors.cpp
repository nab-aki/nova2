#include "sensors.h"

#include "../config.h"
#include "../hal/hal_battery.h"
#include "../hal/hal_light.h"
#include "../hal/hal_track.h"
#include "../hal/hal_ultrasonic.h"

static SensorData data;
static unsigned long lastLightMs = 0;
static unsigned long lastTrackMs = 0;
static unsigned long lastBatteryMs = 0;

void Sensors_Setup(void) {
  data.distanceRawCm = -1.0f;
  data.distanceCm = -1.0f;
  data.distanceValid = false;
  data.lightAdc = Light_Read();
  data.track = Track_Read();
  data.batteryAdc = Battery_ReadAdc();
  data.batteryV = Battery_AdcToVoltage(data.batteryAdc);
}

void Sensors_Update(unsigned long nowMs) {
  // 超音波：測距が1回完了したときだけ更新する
  if (Ultrasonic_Update(nowMs)) {
    data.distanceRawCm = Ultrasonic_GetCm();
    data.distanceValid = data.distanceRawCm >= ULTRASONIC_MIN_CM && data.distanceRawCm <= ULTRASONIC_MAX_CM;
    data.distanceCm = data.distanceValid ? data.distanceRawCm : -1.0f;
  }

  if (nowMs - lastLightMs >= LIGHT_READ_INTERVAL_MS) {
    lastLightMs = nowMs;
    data.lightAdc = Light_Read();
  }
  if (nowMs - lastTrackMs >= TRACK_READ_INTERVAL_MS) {
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
