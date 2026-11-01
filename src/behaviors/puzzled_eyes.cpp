#include "puzzled_eyes.h"

#include "../config.h"
#include "../core/eyes.h"

int PuzzledEyesBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return wander_->isGivenUp() ? PRIORITY_PUZZLED_EYES : 0;
}

void PuzzledEyesBehavior::onStart(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_QUESTION);
}

void PuzzledEyesBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  Eyes_Set(EYE_QUESTION);   // 一時停止の合図などに上書きされたあとも戻す（変化したときだけ描画される）
}

void PuzzledEyesBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_NORMAL);
}
