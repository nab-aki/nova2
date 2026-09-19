#include "hal_servo.h"

#include "../config.h"
#include "hal_pca9685.h"
#include "hal_pins.h"

static int currentAngle[2] = {SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG};

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
    Pca9685_SetPulseWidth(PCA9685_CH_SERVO1, AngleToPulseWidth(angleDeg));
  } else {
    angleDeg = constrain(angleDeg, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
    Pca9685_SetPulseWidth(PCA9685_CH_SERVO2, AngleToPulseWidth(angleDeg));
  }
  currentAngle[servo] = angleDeg;
  return angleDeg;
}

int Servo_GetAngle(ServoId servo) {
  return currentAngle[servo];
}

void Servo_Center(void) {
  Servo_SetAngle(SERVO_PAN, SERVO1_FRONT_DEG);
  Servo_SetAngle(SERVO_TILT, SERVO2_LEVEL_DEG);
}
