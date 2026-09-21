// Nova（Freenove 4WD Car Kit for ESP32 FNK0053 ベース）
// スプリント1：首の調停・障害物の判定・安全層と、ID9「気づく」v1
//   ・目：2〜6秒のランダム間隔でまばたき（ときどき2回連続）
//   ・車体：ゆっくり加速して前進し、ゆっくり減速して止まる、を繰り返す
//   ・正面の障害物に気づいたら即停止し、目を見開いて首を正面に向けたまま待つ
//   ・シリアル（115200bps）に、測距値・判定・首の状態・モーター状態を出力する
//
// 構成：hal/（ハードウェア操作）→ core/（センサー集約・首・障害物・安全・動き・表情・調停）
//       → behaviors/（振る舞い）
// delay() は使わず、loop() を回し続けて millis() で時間を管理する。

#include <Arduino.h>

#include "behaviors/blink.h"
#include "behaviors/notice.h"
#include "config.h"
#include "core/arbiter.h"
#include "core/eyes.h"
#include "core/motion.h"
#include "core/neck.h"
#include "core/obstacle.h"
#include "core/safety.h"
#include "core/sensors.h"
#include "hal/hal.h"
#include "hal/hal_log.h"

static BlinkBehavior blinkBehavior;
static NoticeBehavior noticeBehavior;
static NoticeEyesBehavior noticeEyesBehavior(&noticeBehavior);

static unsigned long lastStatusMs = 0;

// センサー値と状態をまとめて1行表示する
static void PrintStatus(unsigned long nowMs) {
  const SensorData &s = Sensors_Get();

  char distance[24];
  if (s.distanceValid) {
    snprintf(distance, sizeof(distance), "%.1fcm", s.distanceCm);
  } else if (s.distanceRawCm < 0) {
    snprintf(distance, sizeof(distance), "反応なし");
  } else {
    snprintf(distance, sizeof(distance), "範囲外(%.1fcm)", s.distanceRawCm);
  }

  // 接近速度（正＝近づいている）
  char approach[24];
  float cmPerSec = 0.0f;
  if (Obstacle_ApproachSpeed(&cmPerSec)) {
    snprintf(approach, sizeof(approach), "%+.1fcm/s", cmPerSec);
  } else {
    snprintf(approach, sizeof(approach), "-");
  }

  Log_Printf("状態",
             "距離:%s 障害物:%s(近%d/%d) 接近:%s 光:%d ライン:%d%d%d(左中右) 電池:%.2fV(ADC %d) "
             "速度:%.2f(PWM %d) 首:%d/%d(%s,%s)%s 目:%s 車体:%s 目の振る舞い:%s",
             distance,
             Obstacle_IsBlocked() ? "あり" : "なし", Obstacle_NearCount(), Obstacle_SampleCount(),
             approach, s.lightAdc,
             s.track & 0x01, (s.track >> 1) & 0x01, (s.track >> 2) & 0x01,
             s.batteryV, s.batteryAdc,
             Motion_GetSpeed(), Motor_SpeedToPwm(Motion_GetSpeed()),
             Servo_GetAngle(SERVO_PAN), Servo_GetAngle(SERVO_TILT),
             Neck_OwnerName(Neck_Owner()), Neck_IsSteady(nowMs) ? "安定" : "動作中",
             Safety_IsStopping() ? " 安全:停止中" : "",
             Eyes_Name(Eyes_Get()),
             Arbiter_ActiveName(LAYER_BODY), Arbiter_ActiveName(LAYER_EYES));
}

void setup() {
  Log_Setup();
  Log_Printf("起動", "Nova スプリント1（首の調停・障害物の判定・ID9 気づく）");

  randomSeed(esp_random());

  bool trackOk = Hal_Setup();
  if (!trackOk) {
    Log_Printf("起動", "ライントラッキングセンサーが応答しません（起動は続けます）");
  }

  Sensors_Setup();
  Neck_Setup();
  Obstacle_Setup();
  Safety_Setup();
  Eyes_Setup();
  Motion_Setup();

  Arbiter_Register(&noticeBehavior);
  Arbiter_Register(&noticeEyesBehavior);
  Arbiter_Register(&blinkBehavior);

  Log_Printf("起動", "初期化完了。停止閾値%.0fcm・巡航PWM%d で %lums 後に最初の走行を始めます",
             OBSTACLE_STOP_CM, Motor_SpeedToPwm(CRUISE_SPEED), (unsigned long)NOTICE_REST_MS);
}

void loop() {
  unsigned long now = millis();

  Buzzer_Update(now);
  Sensors_Update(now);
  Neck_Update(now);
  Obstacle_Update(Sensors_Get(), now);

  Arbiter_Update(Sensors_Get(), now);   // 振る舞いが車体・目・首を動かす
  Safety_Update(Sensors_Get(), now);    // 安全層が最後に上書きする

  Eyes_Update(now);
  Motion_Update(now);

  if (now - lastStatusMs >= STATUS_PRINT_INTERVAL_MS) {
    lastStatusMs = now;
    PrintStatus(now);
  }
}
