// WS2812（12個）のラッパー
//
// 【注意】公式定義では WS2812 のデータ線と電池電圧の読み取りが同じ GPIO32。
// このため Ws2812_Begin() は自動では呼ばない（スプリント0では使用しない）。
// 使い始める前に、電池電圧の読み取りとの共存を実機で確認すること（docs/decisions.md 参照）。
#ifndef NOVA_HAL_WS2812_H
#define NOVA_HAL_WS2812_H

#include <Arduino.h>

// 初期化して全消灯する（複数回呼んでも1回だけ初期化する）。失敗したら false
bool Ws2812_Begin(void);

// 初期化済みか
bool Ws2812_IsActive(void);

// 1個の色を設定する（Ws2812_Show() を呼ぶまで反映されない）
void Ws2812_SetColor(uint8_t index, uint8_t r, uint8_t g, uint8_t b);

// 全部同じ色にする（Ws2812_Show() を呼ぶまで反映されない）
void Ws2812_Fill(uint8_t r, uint8_t g, uint8_t b);

// 設定した色を出力する
void Ws2812_Show(void);

// 全消灯して出力する
void Ws2812_Off(void);

#endif // NOVA_HAL_WS2812_H
