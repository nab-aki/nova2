// Nova（Freenove 4WD Car Kit for ESP32 FNK0053 ベース）
// スプリント0：土台コードと動作デモ
//   ・目：2〜6秒のランダム間隔でまばたき（ときどき2回連続）
//   ・車体：ゆっくり加速して前進し、ゆっくり減速して止まる、を繰り返す
//   ・シリアル（115200bps）に、状態の変化とセンサー値を出力する
//
// 構成：hal/（ハードウェア操作）→ core/（センサー集約・動き・表情・調停）→ behaviors/（振る舞い）
// delay() は使わず、loop() を回し続けて millis() で時間を管理する。

#include <Arduino.h>

#include "behaviors/blink.h"
#include "behaviors/demo_drive.h"
#include "config.h"
#include "core/arbiter.h"
#include "core/eyes.h"
#include "core/motion.h"
#include "core/sensors.h"
#include "hal/hal.h"
#include "hal/hal_log.h"

static BlinkBehavior blinkBehavior;
static DemoDriveBehavior demoDriveBehavior;

static unsigned long lastStatusMs = 0;

// センサー値と状態をまとめて1行表示する
static void PrintStatus(void) {
  const SensorData &s = Sensors_Get();

  char distance[24];
  if (s.distanceValid) {
    snprintf(distance, sizeof(distance), "%.1fcm", s.distanceCm);
  } else if (s.distanceRawCm < 0) {
    snprintf(distance, sizeof(distance), "反応なし");
  } else {
    snprintf(distance, sizeof(distance), "範囲外(%.1fcm)", s.distanceRawCm);
  }

  Log_Printf("状態",
             "距離:%s 光:%d ライン:%d%d%d(左中右) 電池:%.2fV(ADC %d) "
             "速度:%.2f(PWM %d) 首:%d/%d 目:%s 車体:%s 目の振る舞い:%s",
             distance, s.lightAdc,
             s.track & 0x01, (s.track >> 1) & 0x01, (s.track >> 2) & 0x01,
             s.batteryV, s.batteryAdc,
             Motion_GetSpeed(), Motor_SpeedToPwm(Motion_GetSpeed()),
             Servo_GetAngle(SERVO_PAN), Servo_GetAngle(SERVO_TILT),
             Eyes_Name(Eyes_Get()),
             Arbiter_ActiveName(LAYER_BODY), Arbiter_ActiveName(LAYER_EYES));
}

void setup() {
  Log_Setup();
  Log_Printf("起動", "Nova スプリント0（土台）");

  randomSeed(esp_random());

  bool trackOk = Hal_Setup();
  if (!trackOk) {
    Log_Printf("起動", "ライントラッキングセンサーが応答しません（起動は続けます）");
  }

  Sensors_Setup();
  Eyes_Setup();
  Motion_Setup();

  Arbiter_Register(&blinkBehavior);
  Arbiter_Register(&demoDriveBehavior);

  Log_Printf("起動", "初期化完了。%lums 後に最初の走行を始めます", (unsigned long)DEMO_REST_MS);
}

void loop() {
  unsigned long now = millis();

  Buzzer_Update(now);
  Sensors_Update(now);
  Arbiter_Update(Sensors_Get(), now);
  Eyes_Update(now);
  Motion_Update(now);

  if (now - lastStatusMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusMs = now;
    PrintStatus();
  }
}
