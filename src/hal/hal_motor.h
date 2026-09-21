// モーター（4輪）のラッパー
// 速度は正規化速度（-1.0〜1.0。正=前進）で扱い、PWM値への換算はここで行う。
// 換算：0 は停止、0 でなければ「最低PWM（実測）〜安全上限」の範囲に線形に割り当てる。
#ifndef NOVA_HAL_MOTOR_H
#define NOVA_HAL_MOTOR_H

#include <Arduino.h>

void Motor_Setup(void);

// 左側（M1, M2）と右側（M3, M4）の速さを指定する。同符号で直進、逆符号でその場回転。
void Motor_Drive(float left, float right);

// 生のPWM（符号付き。正=前進）で左右を指定する。
// 回転・片側旋回のように、正規化速度ではなく実測のPWM値そのもので出したいときに使う
// （最低PWMの下限を通さないので、呼ぶ側が動く値を渡すこと）。
void Motor_DrivePwm(int leftPwm, int rightPwm);

// 全輪を止める
void Motor_Stop(void);

// 正規化速度（-1〜1）をPWM値（符号付き）に換算する（表示・確認用に公開）
int Motor_SpeedToPwm(float speed);

#endif // NOVA_HAL_MOTOR_H
