#include "debug_turn.h"

#include "../config.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../hal/hal_battery.h"
#include "../hal/hal_log.h"

void DebugTurnBehavior::request(TurnKind kind, unsigned long durationMs) {
  kind_ = kind;
  durationMs_ = durationMs;
  requested_ = true;
}

int DebugTurnBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return (requested_ || running_) ? PRIORITY_DEBUG_TURN : 0;
}

// 電池電圧を1回読んで、ステップ中の最低値を更新する
void DebugTurnBehavior::sampleBattery(unsigned long nowMs) {
  float v = Battery_ReadVoltage();
  if (batterySamples_ == 0 || v < batteryMinV_) {
    batteryMinV_ = v;
    batteryMinAtMs_ = nowMs - startMs_;
  }
  batterySamples_++;
}

void DebugTurnBehavior::reportBattery() {
  if (batterySamples_ == 0) {
    return;
  }
  Log_Printf("デバッグ", "電池 動き出す前 %.2fV／ステップ中の最低 %.2fV（動き出して %lums 後、%d回読み取り）差 %.2fV",
             batteryIdleV_, batteryMinV_, batteryMinAtMs_, batterySamples_, batteryIdleV_ - batteryMinV_);
}

// 1ステップを始める
void DebugTurnBehavior::begin(unsigned long nowMs) {
  requested_ = false;
  running_ = true;
  startMs_ = nowMs;
  batteryIdleV_ = Battery_ReadVoltage();   // 動き出す前の電圧
  batterySamples_ = 0;
  Safety_SetDebugTurnActive(true);         // このステップの間だけ、障害物「あり」による停止を外す
  if (Obstacle_IsBlocked()) {
    Log_Printf("デバッグ", "前方に障害物あり（%.1fcm）ですが、この1ステップの間は障害物による停止を外します"
               "（%.0fcm 未満の非常停止と、持ち上げでは止まります）",
               Obstacle_LastCm(), OBSTACLE_EMERGENCY_CM);
  }
  Log_Printf("デバッグ", "%s を %lums（1ステップ）。回った角度を測ってください",
             Motion_TurnName(kind_), durationMs_);
  Motion_StartTurn(kind_, nowMs);
}

void DebugTurnBehavior::onStart(unsigned long nowMs) {
  begin(nowMs);
}

void DebugTurnBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (!running_) {
    // 1ステップが終わってから調停が手放すまでの間に、次の予約が入ることがある。
    // このとき調停は「まだ実行中」なので onStart が呼ばれない。ここで始めないと、
    // 予約（requested_）だけが残って優先度を返し続け、以後のキーがすべて効かなくなる
    if (requested_) {
      begin(nowMs);
    }
    return;
  }
  // 持ち上げられたら安全層が止める。こちらも1ステップを打ち切る
  if (Safety_IsLifted()) {
    Log_Printf("デバッグ", "持ち上げられたので中止します");
    Motion_StopTurn(nowMs);
    running_ = false;
    Safety_SetDebugTurnActive(false);
    return;
  }
  sampleBattery(nowMs);
  if (nowMs - startMs_ >= durationMs_) {
    Motion_StopTurn(nowMs);
    running_ = false;
    Safety_SetDebugTurnActive(false);
    Log_Printf("デバッグ", "%s の1ステップが終わりました", Motion_TurnName(kind_));
    reportBattery();
  }
}

void DebugTurnBehavior::onStop(unsigned long nowMs) {
  Motion_StopTurn(nowMs);
  running_ = false;
  requested_ = false;
  Safety_SetDebugTurnActive(false);
}
