#include "stuck_eyes.h"

#include "../config.h"
#include "../core/eyes.h"

int StuckEyesBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return stuck_->isThinking() ? PRIORITY_STUCK_EYES : 0;
}

void StuckEyesBehavior::onStart(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_QUESTION);
}

void StuckEyesBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  Eyes_Set(EYE_QUESTION);   // 一時停止の合図などに上書きされたあとも戻す（変化したときだけ描画される）
}

void StuckEyesBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_NORMAL);
}
