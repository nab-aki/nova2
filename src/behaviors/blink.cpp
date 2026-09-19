#include "blink.h"

#include "../config.h"
#include "../core/eyes.h"
#include "../hal/hal_log.h"

int BlinkBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return PRIORITY_IDLE_BLINK;   // 目に関する他の振る舞いがなければ、常にまばたきしたい
}

void BlinkBehavior::onStart(unsigned long nowMs) {
  secondBlinkPending_ = false;
  ScheduleNext(nowMs);
}

// 次のまばたきの時刻を、ランダムな間隔で決める
void BlinkBehavior::ScheduleNext(unsigned long nowMs) {
  nextBlinkMs_ = nowMs + random(BLINK_INTERVAL_MIN_MS, BLINK_INTERVAL_MAX_MS + 1);
}

void BlinkBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if ((long)(nowMs - nextBlinkMs_) < 0) {
    return;
  }

  if (!Eyes_StartBlink(nowMs)) {
    // 閉じ目などで今はできない。少し後でもう一度試す
    secondBlinkPending_ = false;
    ScheduleNext(nowMs);
    return;
  }

  if (secondBlinkPending_) {
    // 2回連続の2回目
    secondBlinkPending_ = false;
    ScheduleNext(nowMs);
    Log_Printf("まばたき", "2回目（次は %lums 後）", nextBlinkMs_ - nowMs);
  } else if ((int)random(100) < BLINK_DOUBLE_PERCENT) {
    // 2回連続にする：1回目が終わって少し開いてから、もう1回
    secondBlinkPending_ = true;
    nextBlinkMs_ = nowMs + Eyes_BlinkDurationMs() + BLINK_DOUBLE_GAP_MS;
    Log_Printf("まばたき", "1回目（2回連続）");
  } else {
    ScheduleNext(nowMs);
    Log_Printf("まばたき", "1回（次は %lums 後）", nextBlinkMs_ - nowMs);
  }
}
