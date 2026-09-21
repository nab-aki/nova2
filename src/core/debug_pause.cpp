#include "debug_pause.h"

#include "../hal/hal_log.h"

static bool paused = false;

void Pause_Setup(bool startPaused) {
  paused = startPaused;
}

bool Pause_IsPaused(void) {
  return paused;
}

bool Pause_Toggle(const char *source) {
  paused = !paused;
  if (paused) {
    Log_Printf("デバッグ", "一時停止（%s）：うろうろ・困る を止めます（3〜6 で回せます。p かリモコンの ▶ で再開）", source);
  } else {
    Log_Printf("デバッグ", "再開（%s）：うろうろを「ため」から始めます", source);
  }
  return paused;
}
