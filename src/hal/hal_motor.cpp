#include "hal_motor.h"

#include <limits.h>

#include "../config.h"
#include "hal_pca9685.h"
#include "hal_pins.h"

int Motor_SpeedToPwm(float speed) {
  float magnitude = fabsf(speed);
  if (magnitude < MOTOR_SPEED_EPSILON) {
    return 0;
  }
  magnitude = min(magnitude, 1.0f);
  int pwm = (int)(MOTOR_PWM_MIN + (MOTOR_PWM_LIMIT - MOTOR_PWM_MIN) * magnitude);
  return (speed >= 0) ? pwm : -pwm;
}

// 1輪分の出力。正転は IN1、逆転は IN2 にPWMを出し、もう一方は0にする
static void SetWheel(uint8_t chIn1, uint8_t chIn2, int pwm) {
  pwm = constrain(pwm, -PCA9685_PWM_MAX, PCA9685_PWM_MAX);
  if (pwm >= 0) {
    Pca9685_SetPulseWidth(chIn1, pwm);
    Pca9685_SetPulseWidth(chIn2, 0);
  } else {
    Pca9685_SetPulseWidth(chIn1, 0);
    Pca9685_SetPulseWidth(chIn2, -pwm);
  }
}

// 直前に出力したPWM値。変わらないときはI2Cへ書き込まない（停止中に無駄な通信をしない）
#define PWM_UNKNOWN  INT_MIN
static int lastPwmLeft = PWM_UNKNOWN;
static int lastPwmRight = PWM_UNKNOWN;

void Motor_Setup(void) {
  // PCA9685 の初期化は Hal_Setup() 側で済んでいる前提
  Motor_Stop();
}

void Motor_Drive(float left, float right) {
  int pwmLeft = Motor_SpeedToPwm(left);
  int pwmRight = Motor_SpeedToPwm(right);
  if (pwmLeft == lastPwmLeft && pwmRight == lastPwmRight) {
    return;
  }
  lastPwmLeft = pwmLeft;
  lastPwmRight = pwmRight;
  SetWheel(PCA9685_CH_M1_IN1, PCA9685_CH_M1_IN2, MOTOR_1_DIRECTION * pwmLeft);
  SetWheel(PCA9685_CH_M2_IN1, PCA9685_CH_M2_IN2, MOTOR_2_DIRECTION * pwmLeft);
  SetWheel(PCA9685_CH_M3_IN1, PCA9685_CH_M3_IN2, MOTOR_3_DIRECTION * pwmRight);
  SetWheel(PCA9685_CH_M4_IN1, PCA9685_CH_M4_IN2, MOTOR_4_DIRECTION * pwmRight);
}

void Motor_Stop(void) {
  Motor_Drive(0.0f, 0.0f);
}
