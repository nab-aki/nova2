// スプリント0の動作デモ（車体レイヤー）
// 止まっている → ゆっくり加速 → 巡航 → ゆっくり減速 → 止まる、を繰り返す。
// 正面に近い物があるときは走り出さず、走行中なら減速して止まる（最低限の安全）。
// カタログの1つのIDに対応する振る舞いではなく、土台（ID5・ID7・調停）の動作確認用。
#ifndef NOVA_BEHAVIORS_DEMO_DRIVE_H
#define NOVA_BEHAVIORS_DEMO_DRIVE_H

#include "../core/behavior.h"

class DemoDriveBehavior : public Behavior {
 public:
  const char *name() const override { return "デモ走行"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

 private:
  enum State { STATE_REST, STATE_ACCEL, STATE_CRUISE, STATE_DECEL };

  void ChangeState(State next, unsigned long nowMs);
  static const char *StateName(State state);

  State state_ = STATE_REST;
  unsigned long stateStartMs_ = 0;
  bool blocked_ = false;   // 直前の判定（変化したときだけログに出す）
};

#endif // NOVA_BEHAVIORS_DEMO_DRIVE_H
