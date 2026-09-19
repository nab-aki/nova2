#include "hal_track.h"

#include <PCF8574.h>

#include "hal_pins.h"

static PCF8574 trackSensor(I2C_ADDR_TRACK);

bool Track_Setup(void) {
  // I2Cバスは Pca9685_Setup() で初期化済み（同じSDA/SCLを共有）
  return trackSensor.begin();
}

uint8_t Track_Read(void) {
  return trackSensor.read8() & 0x07;
}
