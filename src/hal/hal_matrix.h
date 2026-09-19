// LEDマトリクス（VK16K33。左右2面を目として使用）のラッパー
// 1面は8×8。8バイトで1面を表し、1バイト=1行（上の行から順）。
#ifndef NOVA_HAL_MATRIX_H
#define NOVA_HAL_MATRIX_H

#include <Arduino.h>

void Matrix_Setup(void);

// 左右の目のパターンを表示する（各8バイト）
void Matrix_Show(const uint8_t *left, const uint8_t *right);

// 明るさ（0〜15）
void Matrix_SetBrightness(uint8_t brightness);

// 全消灯
void Matrix_Clear(void);

#endif // NOVA_HAL_MATRIX_H
