#include "hal.h"

#include "hal_i2c.h"
#include "hal_pca9685.h"

bool Hal_Setup(void) {
  I2c_Setup();       // I2Cバスの初期化（速度・タイムアウト）。最初に呼ぶ
  Pca9685_Setup();
  Motor_Setup();     // 起動直後は必ず停止状態にする
  Servo_Setup();     // 首は正面・水平から始める
  Matrix_Setup();
  Ultrasonic_Setup();
  Ir_Setup();        // リモコン（GPIO0）。GPIO割り込みとタイマー3を使う
  Buzzer_Setup();
  Light_Setup();
  Ws2812_ClearAtBoot();   // 電源投入時に光ったままの WS2812 を消す。GPIO32 共用のため電池より前に1回だけ
  Battery_Setup();
  Storage_Setup();
  Reset_Setup();
  return Track_Setup();
}
