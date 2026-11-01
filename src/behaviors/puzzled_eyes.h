// 「困った」の目（目レイヤー。docs/specs/common_tof_scan.md 4章「段2 の方針」）
// ID25 が見えていない向きを1周回ってもだめで、回るのをやめて一時停止している間、「？」を出す。
// 点滅させない（設計原則5）。再開したら消える。
#ifndef NOVA_BEHAVIORS_PUZZLED_EYES_H
#define NOVA_BEHAVIORS_PUZZLED_EYES_H

#include "../core/behavior.h"
#include "wander.h"

class PuzzledEyesBehavior : public Behavior {
 public:
  explicit PuzzledEyesBehavior(const WanderBehavior *wander) : wander_(wander) {}

  const char *name() const override { return "？の目（見えない）"; }
  BehaviorLayer layer() const override { return LAYER_EYES; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

 private:
  const WanderBehavior *wander_;
};

#endif // NOVA_BEHAVIORS_PUZZLED_EYES_H
