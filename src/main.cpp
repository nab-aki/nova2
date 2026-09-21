// Nova（Freenove 4WD Car Kit for ESP32 FNK0053 ベース）
// スプリント2：ID25「うろうろ」v1 と ID15「障害物で困る」最小版
//   ・目：2〜6秒のランダム間隔でまばたき（ときどき2回連続）
//   ・車体：止まって首で見回し、空いていそうな向きへ歩き、また止まる（ID25）
//   ・正面の障害物に気づいたら即停止し、目を見開く（ID9）
//   ・塞がったままなら、後退して左右を確かめ、その場回転で向きを変える（ID15）
//   ・持ち上げたら（ライントラッキング111）すぐモーターを止める（安全層）
//   ・シリアル（115200bps）に、測距値・判定・首の状態・モーター状態を出力する
//   ・シリアルの 3〜6 で、回転角の測定用に1ステップだけ回せる（? でキー一覧）
//
// 構成：hal/（ハードウェア操作）→ core/（センサー集約・首・障害物・安全・動き・表情・調停）
//       → behaviors/（振る舞い）
// delay() は使わず、loop() を回し続けて millis() で時間を管理する。

#include <Arduino.h>

#include "behaviors/blink.h"
#include "behaviors/debug_turn.h"
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
static DebugTurnBehavior debugTurnBehavior;

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

  // 車体の出力（回転中は生のPWM、直進中は正規化速度とPWM）
  char drive[40];
  if (Motion_IsTurning()) {
    snprintf(drive, sizeof(drive), "%s", Motion_TurnName(Motion_GetTurnKind()));
  } else {
    snprintf(drive, sizeof(drive), "速度:%.2f(PWM %d)",
             Motion_GetSpeed(), Motor_SpeedToPwm(Motion_GetSpeed()));
  }

  Log_Printf("状態",
             "距離:%s 障害物:%s(近%d/%d) 接近:%s 光:%d ライン:%d%d%d(左中右) 電池:%.2fV(ADC %d) "
             "%s 首:%d/%d(%s,%s)%s%s 目:%s 車体:%s 目の振る舞い:%s",
             distance,
             Obstacle_IsBlocked() ? "あり" : "なし", Obstacle_NearCount(), Obstacle_SampleCount(),
             approach, s.lightAdc,
             s.track & 0x01, (s.track >> 1) & 0x01, (s.track >> 2) & 0x01,
             s.batteryV, s.batteryAdc,
             drive,
             Servo_GetAngle(SERVO_PAN), Servo_GetAngle(SERVO_TILT),
             Neck_OwnerName(Neck_Owner()), Neck_IsSteady(nowMs) ? "安定" : "動作中",
             Safety_IsStopping() ? " 安全:停止中" : "",
             Safety_IsLifted() ? " 持ち上げ中" : "",
             Eyes_Name(Eyes_Get()),
             Arbiter_ActiveName(LAYER_BODY), Arbiter_ActiveName(LAYER_EYES));
}

// ------------------------ デバッグキー（回転角の測定用）------------------------ //

static void PrintKeyHelp(void) {
  Log_Printf("キー", "3:その場回転 左  4:その場回転 右（各 %lums）", (unsigned long)TROUBLE_TURN_STEP_MS);
  Log_Printf("キー", "5:片側旋回 左  6:片側旋回 右（各 %lums）", (unsigned long)WANDER_AVOID_PIVOT_MS);
  Log_Printf("キー", "いずれも1ステップだけ回します。止まっているときだけ受け付けます");
}

// 止まっていて、立て直しの最中でも持ち上げ中でもないときだけ受け付ける
static void RequestDebugTurn(TurnKind kind, unsigned long durationMs) {
  if (Safety_IsLifted()) {
    Log_Printf("キー", "持ち上げられているので無視します");
    return;
  }
  if (Motion_IsTurning() ||
      fabsf(Motion_GetSpeed()) >= MOTOR_SPEED_EPSILON ||
      fabsf(Motion_GetTarget()) >= MOTOR_SPEED_EPSILON) {
    Log_Printf("キー", "車体が動いているので無視します（止まってから押してください）");
    return;
  }
  debugTurnBehavior.request(kind, durationMs);
}

static void HandleSerialKeys(void) {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    switch (c) {
      case '3': RequestDebugTurn(TURN_ROTATE_LEFT, TROUBLE_TURN_STEP_MS);   break;
      case '4': RequestDebugTurn(TURN_ROTATE_RIGHT, TROUBLE_TURN_STEP_MS);  break;
      case '5': RequestDebugTurn(TURN_PIVOT_LEFT, WANDER_AVOID_PIVOT_MS);   break;
      case '6': RequestDebugTurn(TURN_PIVOT_RIGHT, WANDER_AVOID_PIVOT_MS);  break;
      case '?': PrintKeyHelp(); break;
      case '\r':
      case '\n':
        break;
      default:
        Log_Printf("キー", "不明なキー '%c'（? で一覧）", c);
        break;
    }
  }
}

void setup() {
  Log_Setup();
  Log_Printf("起動", "Nova スプリント2（ID25 うろうろ・ID15 障害物で困る）");

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

  // 登録順は同順位のときの優先順。優先度は config.h の PRIORITY_* で決まる
  Arbiter_Register(&debugTurnBehavior);
  Arbiter_Register(&noticeBehavior);
  Arbiter_Register(&noticeEyesBehavior);
  Arbiter_Register(&blinkBehavior);

  Log_Printf("起動", "初期化完了。停止閾値%.0fcm・巡航PWM%d",
             OBSTACLE_STOP_CM, Motor_SpeedToPwm(CRUISE_SPEED));
  PrintKeyHelp();
}

void loop() {
  unsigned long now = millis();

  HandleSerialKeys();
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
