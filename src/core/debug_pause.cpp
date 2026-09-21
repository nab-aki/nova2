#include "debug_pause.h"

#include "../hal/hal_log.h"

static bool paused = false;

void Pause_Setup(bool startPaused) {
  paused = startPaused;
}

bool Pause_IsPaused(void) {
  return paused;
}

bool Pause_Toggle(void) {
  paused = !paused;
  if (paused) {
    Log_Printf("デバッグ", "一時停止：うろうろ・困る を止めます（3〜6 で回せます。p で再開）");
  } else {
    Log_Printf("デバッグ", "再開：うろうろを「ため」から始めます");
  }
  return paused;
}
