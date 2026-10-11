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

// デバッグ用（見回しの試験）：最後に書いたパルス幅と、IC から読み戻した ON・OFF の値を返す。読めなければ false。
// 一致していれば、readOn＝0・readOff＝written になる
bool Servo_ReadBack(ServoId servo, uint16_t *written, uint16_t *readOn, uint16_t *readOff);

// 正面・水平（実測の基準角）に戻す
void Servo_Center(void);

// 実験用（キー k。首の切り分け）：入にすると、上下のサーボの角度が今の値と同じなら PCA9685 へ書かない。
// 値が変わるときと、左右のサーボは今までどおり書く。起動時は切（今までどおり）。
void Servo_SetSkipUnchangedTilt(bool enabled);
bool Servo_IsSkipUnchangedTilt(void);
uint32_t Servo_TiltWritesSkipped(void);   // 入のあいだに省いた回数（起動からの累計）

#endif // NOVA_HAL_SERVO_H
