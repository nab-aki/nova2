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

#endif // NOVA_HAL_PCA9685_H
