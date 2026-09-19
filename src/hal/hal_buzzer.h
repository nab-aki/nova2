// パッシブブザーのラッパー
// 長いビープ音は避ける方針（CLAUDE.md）なので、鳴らす長さを必ず指定する。
// 公式サンプルの Buzzer_Alert() は delay() を使うため使わず、時間管理は Buzzer_Update() で行う。
#ifndef NOVA_HAL_BUZZER_H
#define NOVA_HAL_BUZZER_H

#include <Arduino.h>

void Buzzer_Setup(void);

// 指定した周波数（Hz）を durationMs だけ鳴らす。時間が来たら Buzzer_Update() が止める
void Buzzer_Beep(uint16_t frequencyHz, uint16_t durationMs);

// すぐに止める
void Buzzer_Off(void);

// loop() から毎回呼ぶ
void Buzzer_Update(unsigned long nowMs);

#endif // NOVA_HAL_BUZZER_H
