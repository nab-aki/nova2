// 車体の動き（ID7 なめらかな加減速の最小版）
// 振る舞いは速度の「目標」だけを指定し、実際の出力はここが Smoother でなめらかにして
// モーターに渡す。急発進・急停止をしない（設計原則3）。直進中はわずかな速度の揺らぎを重ねる。
#ifndef NOVA_CORE_MOTION_H
#define NOVA_CORE_MOTION_H

#include <Arduino.h>

void Motion_Setup(void);

// 目標速度（正規化速度 -1.0〜1.0）を設定し、rampMs かけて近づける
void Motion_SetSpeed(float target, unsigned long rampMs, unsigned long nowMs);

// MOTION_DECEL_MS かけてなめらかに止まる
void Motion_Stop(unsigned long nowMs);

// 即座に止める（非常時用。通常の停止には使わない）
void Motion_EmergencyStop(void);

// loop() から毎回呼ぶ。一定間隔でモーター出力を更新する
void Motion_Update(unsigned long nowMs);

float Motion_GetSpeed(void);      // なめらか化後の現在速度（揺らぎ除く）
float Motion_GetTarget(void);
bool Motion_IsAtTarget(void);     // 目標速度に到達したか

#endif // NOVA_CORE_MOTION_H
