#include "hal_pca9685.h"

#include <Wire.h>
#include <PCA9685.h>

#include "../config.h"
#include "hal_pins.h"

static PCA9685 pca9685;

void Pca9685_Setup(void) {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  pca9685.setupSingleDevice(Wire, I2C_ADDR_PCA9685);

  // MODE1レジスタを初期化する（tools/pwm_test で動作確認済みの手順。公式サンプル 01.4 と同じ）
  Wire.beginTransmission(I2C_ADDR_PCA9685);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();

  pca9685.setToFrequency(PCA9685_FREQUENCY_HZ);
}

void Pca9685_SetPulseWidth(uint8_t channel, uint16_t pulseWidth) {
  pca9685.setChannelPulseWidth(channel, pulseWidth);
}
