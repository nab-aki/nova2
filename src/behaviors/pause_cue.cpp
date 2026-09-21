#include "pause_cue.h"

#include "../config.h"
#include "../core/eyes.h"

void PauseCueBehavior::trigger(bool paused) {
  nextPaused_ = paused;
  requested_ = true;
}

int PauseCueBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return (requested_ || running_) ? PRIORITY_PAUSE_CUE : 0;
}

void PauseCueBehavior::begin(unsigned long nowMs) {
  requested_ = false;
  running_ = true;
  paused_ = nextPaused_;
  startMs_ = nowMs;
}

void PauseCueBehavior::finish(void) {
  running_ = false;
  Eyes_Set(EYE_NORMAL);
}

void PauseCueBehavior::onStart(unsigned long nowMs) {
  begin(nowMs);
}

void PauseCueBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  // 合図が終わってから調停が手放すまでの間に、次の予約が入ることがある（そのとき onStart は呼ばれない）。
  // ここで始めないと、予約だけが残って優先度を返し続ける（debug_turn.cpp と同じ理由）
  if (!running_) {
    if (requested_) {
      begin(nowMs);
    } else {
      return;
    }
  }
  if (requested_) {
    begin(nowMs);   // 合図の途中で切り替えられたら、新しい合図を頭から
  }

  unsigned long elapsed = nowMs - startMs_;
  if (paused_) {
    // 一時停止：目を細める
    if (elapsed >= PAUSE_CUE_PAUSE_MS) {
      finish();
      return;
    }
    Eyes_Set(EYE_NARROW);
    return;
  }
  // 再開：ゆっくり閉じて開く（細める → 閉じる → 細める）
  unsigned long half = PAUSE_CUE_RESUME_HALF_MS;
  unsigned long closed = PAUSE_CUE_RESUME_CLOSED_MS;
  if (elapsed < half) {
    Eyes_Set(EYE_NARROW);
  } else if (elapsed < half + closed) {
    Eyes_Set(EYE_CLOSED);
  } else if (elapsed < half + closed + half) {
    Eyes_Set(EYE_NARROW);
  } else {
    finish();
  }
}

void PauseCueBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  running_ = false;
  requested_ = false;
  Eyes_Set(EYE_NORMAL);
}
