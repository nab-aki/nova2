// ID1 まばたき（目レイヤー）：2〜6秒のランダム間隔で、ときどき2回連続
#ifndef NOVA_BEHAVIORS_BLINK_H
#define NOVA_BEHAVIORS_BLINK_H

#include "../core/behavior.h"

class BlinkBehavior : public Behavior {
 public:
  const char *name() const override { return "まばたき"; }
  BehaviorLayer layer() const override { return LAYER_EYES; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;

 private:
  void ScheduleNext(unsigned long nowMs);

  unsigned long nextBlinkMs_ = 0;
  bool secondBlinkPending_ = false;   // 2回連続の2回目が残っているか
};

#endif // NOVA_BEHAVIORS_BLINK_H
