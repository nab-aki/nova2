// 光センサー（フォトレジスタ）のラッパー
#ifndef NOVA_HAL_LIGHT_H
#define NOVA_HAL_LIGHT_H

#include <Arduino.h>

void Light_Setup(void);

// ADC生値（0〜4095）
int Light_Read(void);

#endif // NOVA_HAL_LIGHT_H
