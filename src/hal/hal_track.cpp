#include "hal_track.h"

#include <PCF8574.h>

#include "hal_i2c.h"
#include "hal_pins.h"

static PCF8574 trackSensor(I2C_ADDR_TRACK);
static bool lastReadOk = false;

bool Track_Setup(void) {
  // I2Cバスは I2c_Setup() で初期化済み（同じSDA/SCLを共有）
  // begin() は、応答の確認（送信1回）のあと、全ピンを入力（High）にする書き込み（送信1回）を行う
  bool ok = trackSensor.begin();
  I2c_AddStats(I2C_DEV_TRACK, 1, ok ? 0 : 1, 0);
  if (ok) {
    I2c_CountTx(I2C_DEV_TRACK, (uint8_t)trackSensor.lastError());
  }
  return ok;
}

uint8_t Track_Read(void) {
  // 読めなかったとき、ライブラリは前回の値をそのまま返す。lastError() で失敗を拾う（読むと消える）
  uint8_t value = trackSensor.read8() & 0x07;
  lastReadOk = I2c_CountRx(I2C_DEV_TRACK, trackSensor.lastError() == PCF8574_OK);
  return value;
}

bool Track_LastReadOk(void) {
  return lastReadOk;
}
