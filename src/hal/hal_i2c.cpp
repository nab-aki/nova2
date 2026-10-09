#include "hal_i2c.h"

#include <Wire.h>

#include "../config.h"
#include "hal_log.h"
#include "hal_pins.h"

static I2cStats stats[I2C_DEV_COUNT];

void I2c_Setup(void) {
  // ライブラリ（PCA9685・LEDマトリクス）も Wire.begin() を呼ぶが、初期化済みのバスはそのまま使われる
  // （Arduino-ESP32 2.x。ピン・速度・タイムアウトは変わらない）
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  Log_Printf("I2C", "バス：%luHz・タイムアウト %ums", (unsigned long)Wire.getClock(), (unsigned)Wire.getTimeOut());
}

void I2c_AddStats(I2cDevice device, uint32_t total, uint32_t fail, uint32_t timeout) {
  stats[device].total += total;
  stats[device].fail += fail;
  stats[device].timeout += timeout;
}

bool I2c_CountTx(I2cDevice device, uint8_t txCode) {
  stats[device].total++;
  if (txCode == I2C_TX_OK) {
    return true;
  }
  stats[device].fail++;
  if (txCode == I2C_TX_TIMEOUT) {
    stats[device].timeout++;
  }
  return false;
}

bool I2c_CountRx(I2cDevice device, bool ok) {
  stats[device].total++;
  if (!ok) {
    stats[device].fail++;
  }
  return ok;
}

const I2cStats &I2c_GetStats(I2cDevice device) {
  return stats[device];
}

const char *I2c_DeviceName(I2cDevice device) {
  switch (device) {
    case I2C_DEV_PCA9685: return "PCA9685";
    case I2C_DEV_TRACK:   return "PCF8574";
    case I2C_DEV_MATRIX:  return "マトリクス";
    default:              return "ジャイロ";
  }
}

bool I2c_WriteReg(I2cDevice device, uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return I2c_CountTx(device, Wire.endTransmission());
}

bool I2c_ReadRegs(I2cDevice device, uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  // リスタート条件で読みに移る。Arduino-ESP32 2.x では、実際の通信は次の requestFrom() でまとめて行われる
  // （この endTransmission(false) はバスに何も出さず、常に成功を返す）ので、数えるのは受信の1回だけ
  Wire.endTransmission(false);
  size_t received = Wire.requestFrom(address, length);
  if (!I2c_CountRx(device, received == length)) {
    while (Wire.available() > 0) {
      Wire.read();   // 中途半端に受け取った分は捨てる
    }
    return false;
  }
  for (uint8_t i = 0; i < length; i++) {
    buffer[i] = (uint8_t)Wire.read();
  }
  return true;
}

void I2c_PrintStats(void) {
  char line[256];
  int used = 0;
  for (int i = 0; i < I2C_DEV_COUNT; i++) {
    const I2cStats &s = stats[i];
    used += snprintf(line + used, sizeof(line) - used, "%s%s 失敗 %lu/%lu（タイムアウト %lu）",
                     (i == 0) ? "" : "  ", I2c_DeviceName((I2cDevice)i),
                     (unsigned long)s.fail, (unsigned long)s.total, (unsigned long)s.timeout);
    if (used >= (int)sizeof(line)) {
      break;
    }
  }
  Log_Printf("I2C", "%s", line);
}
