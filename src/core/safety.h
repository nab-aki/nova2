// 安全層（共通部品。docs/specs/common_obstacle.md）
// 振る舞いの指示を最後に上書きする。loop() では Arbiter_Update() の後、Motion_Update() の前に呼ぶ。
//   1. 走行中は首の使用権を「安全」で取り、正面・水平に固定する
//   2. 障害物ありのときは、なめらか加減速を通さずに即停止する
//      （設計原則3「急停止をしない」の例外。docs/decisions.md）
#ifndef NOVA_CORE_SAFETY_H
#define NOVA_CORE_SAFETY_H

#include <Arduino.h>

#include "sensors.h"

void Safety_Setup(void);

void Safety_Update(const SensorData &sensors, unsigned long nowMs);

// 安全層が車体を止めている状態か（表示用）
bool Safety_IsStopping(void);

#endif // NOVA_CORE_SAFETY_H
