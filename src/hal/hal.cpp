#include "hal.h"

#include "hal_pca9685.h"

bool Hal_Setup(void) {
  Pca9685_Setup();   // I2Cバスの初期化を含む。最初に呼ぶ
  Motor_Setup();     // 起動直後は必ず停止状態にする
  Servo_Setup();     // 首は正面・水平から始める
  Matrix_Setup();
  Ultrasonic_Setup();
  Buzzer_Setup();
  Light_Setup();
  Battery_Setup();
  return Track_Setup();
}
