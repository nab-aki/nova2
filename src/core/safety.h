// 安全層（共通部品。docs/specs/common_obstacle.md）
// 振る舞いの指示を最後に上書きする。loop() では Arbiter_Update() の後、Motion_Update() の前に呼ぶ。
//   1. 車体が動いている間は首の使用権を「安全」で取り、正面・水平に固定する
//   2. 持ち上げられた（ライントラッキングが 111）ら、すべての動きを即座に止める
//   3. 障害物ありのときは、前進と片側旋回だけを即停止する
//      （設計原則3「急停止をしない」の例外。docs/decisions.md）
//      後退とその場回転は止めない。止めると ID15「障害物で困る」が壁の前で立て直せないため。
#ifndef NOVA_CORE_SAFETY_H
#define NOVA_CORE_SAFETY_H

#include <Arduino.h>

#include "sensors.h"

void Safety_Setup(void);

void Safety_Update(const SensorData &sensors, unsigned long nowMs);

// 安全層が車体を止めている状態か（表示用）
bool Safety_IsStopping(void);

// 持ち上げられているか。振る舞いはこの間、状態を進めない。
// 床に戻ったら、続きからではなく最初からやり直す（docs/specs/25_wander.md）
bool Safety_IsLifted(void);

#endif // NOVA_CORE_SAFETY_H
