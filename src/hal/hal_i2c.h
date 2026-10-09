// I2Cバス（Wire、GPIO13/14）の窓口と、機器ごとの失敗の集計（共通部品。#0。docs/specs/common_gyro_checklist.md「共有バスの対策」）
// 同じバスに PCA9685（モーター・首）・PCF8574（ライン）・LEDマトリクス・ジャイロの4台がつながる。
//   ・速度は 100kHz（PCF8574 の上限）
//   ・タイムアウトは I2C_TIMEOUT_MS（既定の 50ms のままだと、バスの異常時にモーターの更新が長く止まる）
//   ・通信の失敗は捨てずに、機器ごとに数える（シリアルの t で表示）
// バスの復帰（SCL を9回振る）は ESP-IDF のドライバが自動で行う。
#ifndef NOVA_HAL_I2C_H
#define NOVA_HAL_I2C_H

#include <Arduino.h>

enum I2cDevice {
  I2C_DEV_PCA9685,
  I2C_DEV_TRACK,
  I2C_DEV_MATRIX,
  I2C_DEV_GYRO,
  I2C_DEV_COUNT
};

// 累計（起動から。RAM だけに持ち、保存しない）
struct I2cStats {
  uint32_t total;     // 通信の回数（送信1回・受信1回をそれぞれ1と数える）
  uint32_t fail;      // 失敗の回数（タイムアウトを含む）
  uint32_t timeout;   // そのうち、送信のタイムアウト（Wire.endTransmission() の戻り値 5）。
                      // 受信（requestFrom）は失敗の理由が分からないので、ここには入らない
};

// Wire.endTransmission() の戻り値（Arduino-ESP32 2.x）
#define I2C_TX_OK        0
#define I2C_TX_TIMEOUT   5

// バスを初期化する。ほかのハードウェアより先に呼ぶ
void I2c_Setup(void);

// 集計に足す（ライブラリ側で数えた分を、各 hal がここへ移す）
void I2c_AddStats(I2cDevice device, uint32_t total, uint32_t fail, uint32_t timeout);

// 送信1回の結果（Wire.endTransmission() の戻り値）を数える。成功なら true
bool I2c_CountTx(I2cDevice device, uint8_t txCode);

// 受信1回の結果を数える。ok：頼んだバイト数を受け取れたか
bool I2c_CountRx(I2cDevice device, bool ok);

const I2cStats &I2c_GetStats(I2cDevice device);
const char *I2c_DeviceName(I2cDevice device);

// レジスタへ1バイト書く／レジスタから連続して読む（失敗は device に数える）。成功なら true
bool I2c_WriteReg(I2cDevice device, uint8_t address, uint8_t reg, uint8_t value);
bool I2c_ReadRegs(I2cDevice device, uint8_t address, uint8_t reg, uint8_t *buffer, uint8_t length);

// 機器ごとの失敗の数を1行で出す（t キー用）
void I2c_PrintStats(void);

#endif // NOVA_HAL_I2C_H
