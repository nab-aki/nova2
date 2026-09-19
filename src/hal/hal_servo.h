// 首サーボ（パンチルト）のラッパー
// 角度は config.h の実測可動範囲に丸めてから出力する（機械的に当たる角度へ行かせない）。
#ifndef NOVA_HAL_SERVO_H
#define NOVA_HAL_SERVO_H

#include <Arduino.h>

enum ServoId {
  SERVO_PAN,   // servo1：左右
  SERVO_TILT   // servo2：上下
};

void Servo_Setup(void);

// 角度（度）を指定する。可動範囲外は範囲内に丸める。戻り値は実際に出力した角度
int Servo_SetAngle(ServoId servo, int angleDeg);

// 最後に出力した角度
int Servo_GetAngle(ServoId servo);

// 正面・水平（実測の基準角）に戻す
void Servo_Center(void);

#endif // NOVA_HAL_SERVO_H
