// デバッグ：うろうろの一時停止（#0。docs/specs/25_wander.md「うろうろの一時停止」）
// 回転角を落ち着いて測るため、ID25「うろうろ」と ID15「障害物で困る」を止められるようにする。
// 一時停止中は、この2つの優先度が0になり、車体はその場で止まったままになる
// （目のまばたきと、ID9 の気づく反応、デバッグ回転（3〜6）は動く）。
// 再開すると、ID25 は「ため」から、ID15 は必要なら最初からやり直す。
#ifndef NOVA_CORE_DEBUG_PAUSE_H
#define NOVA_CORE_DEBUG_PAUSE_H

#include <Arduino.h>

// startPaused が true なら、一時停止の状態から始める（config.h の DEBUG_START_PAUSED）
void Pause_Setup(bool startPaused);

bool Pause_IsPaused(void);

// 一時停止と再開を切り替える。切り替えた結果（true=一時停止中）を返す。
// source は、どの入口から切り替えたか（シリアルに出す。例："キー p"、"リモコン ▶"）
bool Pause_Toggle(const char *source);

#endif // NOVA_CORE_DEBUG_PAUSE_H
