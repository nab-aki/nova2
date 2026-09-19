// なめらかな加減速の共通処理（ID7 の土台）
// 目標値が変わったら、現在値から目標値まで指定時間かけて S字（smoothstep）で近づける。
// 出だしと終わりが緩やかなので、速度・角度などに共通で使える。
#ifndef NOVA_CORE_SMOOTHER_H
#define NOVA_CORE_SMOOTHER_H

#include <Arduino.h>

class Smoother {
 public:
  explicit Smoother(float initial = 0.0f)
      : start_(initial), target_(initial), current_(initial), startMs_(0), durationMs_(0) {}

  // 現在値と目標値を即座にそろえる（初期化・非常停止用）
  void reset(float value) {
    start_ = target_ = current_ = value;
    durationMs_ = 0;
  }

  // 目標値を設定する。同じ目標値の再設定は無視する（毎ループ呼んでも進行を邪魔しない）
  void setTarget(float target, unsigned long durationMs, unsigned long nowMs) {
    if (target == target_) {
      return;
    }
    start_ = current_;
    target_ = target;
    startMs_ = nowMs;
    durationMs_ = durationMs;
  }

  // loop() から呼ぶ。現在値を進める
  void update(unsigned long nowMs) {
    if (current_ == target_) {
      return;
    }
    unsigned long elapsed = nowMs - startMs_;
    if (durationMs_ == 0 || elapsed >= durationMs_) {
      current_ = target_;
      return;
    }
    float t = (float)elapsed / (float)durationMs_;
    float eased = t * t * (3.0f - 2.0f * t);   // smoothstep
    current_ = start_ + (target_ - start_) * eased;
  }

  float value() const { return current_; }
  float target() const { return target_; }
  bool done() const { return current_ == target_; }

 private:
  float start_;
  float target_;
  float current_;
  unsigned long startMs_;
  unsigned long durationMs_;
};

#endif // NOVA_CORE_SMOOTHER_H
