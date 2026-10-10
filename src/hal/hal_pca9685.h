// PCA9685（モーター・サーボ共用のPWMチップ）への窓口
// モーターとサーボは同じチップ・同じPWM周波数を共有するため、このモジュールが一元管理する。
// 他の層からは直接使わず、hal_motor / hal_servo 経由で使う。
#ifndef NOVA_HAL_PCA9685_H
#define NOVA_HAL_PCA9685_H

#include <Arduino.h>

// I2Cバスの初期化とPCA9685の初期化（周波数は config.h の PCA9685_FREQUENCY_HZ）
void Pca9685_Setup(void);

// チャンネルにパルス幅（0〜4095）を出力する
void Pca9685_SetPulseWidth(uint8_t channel, uint16_t pulseWidth);

// チャンネルの ON・OFF の値（LEDn_ON・LEDn_OFF。各16ビット）を IC から読み戻す。デバッグ用（見回しの試験）。
// 読めなければ false。Pca9685_SetPulseWidth(ch, w) で書いたあとは、ON＝0・OFF＝w が読めるはず（0 < w < 4095 のとき）
bool Pca9685_ReadOnOff(uint8_t channel, uint16_t *onTime, uint16_t *offTime);

#endif // NOVA_HAL_PCA9685_H
