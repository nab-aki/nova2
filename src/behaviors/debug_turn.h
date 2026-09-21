// デバッグ：回転角の測定（#0。docs/specs/25_wander.md「回転角を測るためのデバッグキー」）
// シリアルの 3〜6 で、その場回転・片側旋回を1ステップだけ実行する。
// 1ステップで何度回るかを実測して、WANDER_AVOID_PIVOT_MS と TROUBLE_MAX_STEPS を詰めるための道具。
//
// 車体レイヤーのいちばん高い優先度（PRIORITY_DEBUG_TURN）で、受け付けたときだけ発動する。
// 受け付けてよいかの判断（停止中か・ID15 が動いていないか・持ち上げられていないか）は
// キーを読む側（main.cpp）が行う。
#ifndef NOVA_BEHAVIORS_DEBUG_TURN_H
#define NOVA_BEHAVIORS_DEBUG_TURN_H

#include "../core/behavior.h"
#include "../core/motion.h"

class DebugTurnBehavior : public Behavior {
 public:
  const char *name() const override { return "デバッグ回転"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 1ステップ分を予約する。次の調停で実行される
  void request(TurnKind kind, unsigned long durationMs);

 private:
  bool requested_ = false;
  bool running_ = false;
  TurnKind kind_ = TURN_ROTATE_LEFT;
  unsigned long durationMs_ = 0;
  unsigned long startMs_ = 0;
};

#endif // NOVA_BEHAVIORS_DEBUG_TURN_H
