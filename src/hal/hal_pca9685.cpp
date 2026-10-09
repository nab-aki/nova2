#include "hal_pca9685.h"

#include <Wire.h>
#include <PCA9685.h>

#include "../config.h"
#include "hal_i2c.h"
#include "hal_pins.h"

// PCA9685 データシート（NXP Semiconductors, Rev. 4 - 7 April 2015）
// https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
// MODE1 レジスタ（アドレス 0x00）。Table 4「MODE1 bit description」より、bit0 = ALLCALL。
#define PCA9685_MODE1_REG         0x00
#define PCA9685_MODE1_ALLCALL     0x01   // bit0

static PCA9685 pca9685;

// ライブラリが数えた通信の回数・失敗（lib/PCA9685/NOVA_PATCH.md）のうち、前回までに集計へ移した分
static uint32_t syncedTotal = 0;
static uint32_t syncedFail = 0;
static uint32_t syncedTimeout = 0;

// ライブラリを呼んだあとに、増えた分を機器ごとの集計（hal_i2c）へ移す
static void SyncI2cStats(void) {
  I2c_AddStats(I2C_DEV_PCA9685,
               pca9685.nova_i2c_total - syncedTotal,
               pca9685.nova_i2c_fail - syncedFail,
               pca9685.nova_i2c_timeout - syncedTimeout);
  syncedTotal = pca9685.nova_i2c_total;
  syncedFail = pca9685.nova_i2c_fail;
  syncedTimeout = pca9685.nova_i2c_timeout;
}

// 起動直後の MODE1 は ALLCALL=1（データシート 7.3.1節、パワーオン時の初期値）。
// ALLCALL のままだと、個別アドレス（I2C_ADDR_PCA9685）に加えて All Call アドレスにも応答してしまい、
// 同じバスの他の機器（PCF8574・VK16K33）と紛れる余地が生まれるため、無効化する。
// 他のビット（AI・SLEEPなど）は変えたくないので、読んでから bit0 だけ落として書き戻す。
static void DisableAllCall(void) {
  uint8_t mode1 = 0;
  if (!I2c_ReadRegs(I2C_DEV_PCA9685, I2C_ADDR_PCA9685, PCA9685_MODE1_REG, &mode1, 1)) {
    return;   // 読めなければ触らない（起動時の一度きりの処理。次の電源投入時にやり直す）
  }
  mode1 &= ~PCA9685_MODE1_ALLCALL;
  I2c_WriteReg(I2C_DEV_PCA9685, I2C_ADDR_PCA9685, PCA9685_MODE1_REG, mode1);
}

void Pca9685_Setup(void) {
  // I2Cバスは I2c_Setup() で初期化済み（ライブラリも Wire.begin() を呼ぶが、初期化済みのバスはそのまま使われる）
  // setupSingleDevice() はソフトウェアリセット（General Call アドレスへ 0x06。データシート 7.6節
  // "Software reset"）のあと、MODE1 の SLEEP を 0・AI（オートインクリメント）を 1 に戻す。
  pca9685.setupSingleDevice(Wire, I2C_ADDR_PCA9685);

  DisableAllCall();

  // PWM周波数の変更手順（データシート 7.3.5節 "PRE_SCALE register"）：
  // SLEEP を 1 にして内蔵発振器を止める → PRE_SCALE レジスタ（0xFE）に値を書く →
  // SLEEP を 0 に戻し、発振器が安定するまで待つ（500µs）。この一連はライブラリの setToFrequency() が行う。
  pca9685.setToFrequency(PCA9685_FREQUENCY_HZ);
  SyncI2cStats();
}

void Pca9685_SetPulseWidth(uint8_t channel, uint16_t pulseWidth) {
  pca9685.setChannelPulseWidth(channel, pulseWidth);
  SyncI2cStats();
}
