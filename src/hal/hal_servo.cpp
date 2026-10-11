#include "hal_servo.h"

#include "../config.h"
#include "hal_pca9685.h"
#include "hal_pins.h"

static int currentAngle[2] = {SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG};
static uint16_t lastPulseWidth[2] = {0, 0};   // 最後に書いたパルス幅（読み戻しと比べるために覚えておく）
// サーボ（軸）→ PCA9685 のチャンネル。角度の範囲は軸に付いたままで、ここで変わるのは出力先だけ
#if SERVO_SWAP_CHANNELS
static const uint8_t SERVO_CHANNEL[2] = {PCA9685_CH_SERVO2, PCA9685_CH_SERVO1};   // 切り分け用：左右＝ch1、上下＝ch0
#else
static const uint8_t SERVO_CHANNEL[2] = {PCA9685_CH_SERVO1, PCA9685_CH_SERVO2};   // 既定：左右＝ch0、上下＝ch1
#endif
static bool skipUnchangedTilt = false;        // 実験用。起動時は切
static uint32_t tiltWritesSkipped = 0;

// 角度（0〜180°）→パルス幅。公式サンプル 01.2_Servo と同じ map(0-180 -> 102-512)（50Hz動作）
static uint16_t AngleToPulseWidth(int angleDeg) {
  return (uint16_t)map(constrain(angleDeg, 0, 180), 0, 180, 102, 512);
}

void Servo_Setup(void) {
  Servo_Center();
}

int Servo_SetAngle(ServoId servo, int angleDeg) {
  if (servo == SERVO_PAN) {
    angleDeg = constrain(angleDeg, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
    Pca9685_SetPulseWidth(SERVO_CHANNEL[SERVO_PAN], AngleToPulseWidth(angleDeg));
  } else {
    angleDeg = constrain(angleDeg, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
    if (skipUnchangedTilt && angleDeg == currentAngle[SERVO_TILT]) {
      tiltWritesSkipped++;   // 実験用：同じ角度なら書かない（パルス幅も角度も変わらない）
      return angleDeg;
    }
    Pca9685_SetPulseWidth(SERVO_CHANNEL[SERVO_TILT], AngleToPulseWidth(angleDeg));
  }
  lastPulseWidth[servo] = AngleToPulseWidth(angleDeg);
  currentAngle[servo] = angleDeg;
  return angleDeg;
}

int Servo_GetAngle(ServoId servo) {
  return currentAngle[servo];
}

bool Servo_ReadBack(ServoId servo, uint16_t *written, uint16_t *readOn, uint16_t *readOff) {
  *written = lastPulseWidth[servo];
  return Pca9685_ReadOnOff(SERVO_CHANNEL[servo], readOn, readOff);
}

void Servo_SetSkipUnchangedTilt(bool enabled) {
  skipUnchangedTilt = enabled;
}

bool Servo_IsSkipUnchangedTilt(void) {
  return skipUnchangedTilt;
}

uint32_t Servo_TiltWritesSkipped(void) {
  return tiltWritesSkipped;
}

uint8_t Servo_Channel(ServoId servo) {
  return SERVO_CHANNEL[servo];
}

bool Servo_IsSwapped(void) {
  return SERVO_SWAP_CHANNELS != 0;
}

void Servo_Center(void) {
  Servo_SetAngle(SERVO_PAN, SERVO1_FRONT_DEG);
  Servo_SetAngle(SERVO_TILT, SERVO2_LEVEL_DEG);
}
