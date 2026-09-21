#include "debug_turn.h"

#include "../config.h"
#include "../core/safety.h"
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

void DebugTurnBehavior::onStart(unsigned long nowMs) {
  requested_ = false;
  running_ = true;
  startMs_ = nowMs;
  Log_Printf("デバッグ", "%s を %lums（1ステップ）。回った角度を測ってください",
             Motion_TurnName(kind_), durationMs_);
  Motion_StartTurn(kind_, nowMs);
}

void DebugTurnBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (!running_) {
    return;
  }
  // 持ち上げられたら安全層が止める。こちらも1ステップを打ち切る
  if (Safety_IsLifted()) {
    Log_Printf("デバッグ", "持ち上げられたので中止します");
    Motion_StopTurn(nowMs);
    running_ = false;
    return;
  }
  if (nowMs - startMs_ >= durationMs_) {
    Motion_StopTurn(nowMs);
    running_ = false;
    Log_Printf("デバッグ", "%s の1ステップが終わりました", Motion_TurnName(kind_));
  }
}

void DebugTurnBehavior::onStop(unsigned long nowMs) {
  Motion_StopTurn(nowMs);
  running_ = false;
  requested_ = false;
}
