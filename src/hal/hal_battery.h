// 電池電圧読み取りのラッパー
// 換算式（ADC 12bit・12dB減衰・5回平均・esp_adc_cal補正・分圧比4）は公式サンプル
// Sketches/01.4_Battery_level に合わせている（tools/pwm_test で動作確認済み）。
//
// 【注意】公式定義では WS2812 と同じ GPIO32。WS2812 を使い始めるときは共存を確認すること。
#ifndef NOVA_HAL_BATTERY_H
#define NOVA_HAL_BATTERY_H

#include <Arduino.h>

void Battery_Setup(void);

// ADC生値（5回平均）
int Battery_ReadAdc(void);

// ADC生値を電池電圧（V）に換算する（同じ読み取りからADC値と電圧の両方を得るため）
float Battery_AdcToVoltage(int adc);

// 電池電圧（V）。Battery_AdcToVoltage(Battery_ReadAdc()) と同じ
float Battery_ReadVoltage(void);

#endif // NOVA_HAL_BATTERY_H
