// ID26 詰まり脱出の「？」の目（目レイヤー。docs/specs/26_stuck.md「？の目のタイミングと長さ」）
// 詰まりに気づいた瞬間から、後退を始めるまで「？」を出す（目が先。設計原則1）。
// 点滅させない（設計原則5）。この間はまばたき（10）・見開く（30）より優先する。
#ifndef NOVA_BEHAVIORS_STUCK_EYES_H
#define NOVA_BEHAVIORS_STUCK_EYES_H

#include "../core/behavior.h"
#include "stuck.h"

class StuckEyesBehavior : public Behavior {
 public:
  explicit StuckEyesBehavior(const StuckBehavior *stuck) : stuck_(stuck) {}

  const char *name() const override { return "？の目（詰まり）"; }
  BehaviorLayer layer() const override { return LAYER_EYES; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

 private:
  const StuckBehavior *stuck_;
};

#endif // NOVA_BEHAVIORS_STUCK_EYES_H
