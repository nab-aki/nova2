// 超音波センサー（車体前面固定・正面のみ測距）のラッパー
// 割り込みでエコーの立ち上がり/立ち下がり時刻を記録するノンブロッキング方式。
// （トリガーの10μs待ちだけは超音波モジュールの仕様上の最小待ち時間で、delay()には当たらない）
#ifndef NOVA_HAL_ULTRASONIC_H
#define NOVA_HAL_ULTRASONIC_H

#include <Arduino.h>

void Ultrasonic_Setup(void);

// loop() から毎回呼ぶ。測距が1回完了した（エコー受信または時間切れ）ときに true を返す
bool Ultrasonic_Update(unsigned long nowMs);

// 直近の測距結果（cm）。エコーが返らなかった場合は負の値
float Ultrasonic_GetCm(void);

#endif // NOVA_HAL_ULTRASONIC_H
