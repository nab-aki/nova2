#include "hal_gyro.h"

#include "hal_i2c.h"
#include "hal_pins.h"

// ST データシート DS13510 Rev 4（LSM6DSV16X）のレジスタ番号と、使うビット。
// https://www.st.com/resource/en/datasheet/lsm6dsv16x.pdf
#define REG_IF_CFG              0x03   // 9.3節
#define REG_FIFO_CTRL3          0x09   // 9.7節
#define REG_FIFO_CTRL4          0x0A   // 9.8節
#define REG_WHO_AM_I            0x0F   // 9.13節
#define REG_CTRL1               0x10   // 9.14節（加速度）
#define REG_CTRL2               0x11   // 9.15節（ジャイロ）
#define REG_CTRL3               0x12   // 9.16節
#define REG_CTRL6               0x15   // 9.19節
#define REG_FIFO_STATUS1        0x1B   // 9.25節
#define REG_INTERNAL_FREQ_FINE  0x4F   // 9.52節
#define REG_FIFO_DATA_OUT_TAG   0x78   // 9.84節（続く 79h〜7Eh がデータ6バイト）

// IF_CFG（Table 30）：bit5 ASF_CTRL＝1 で、SCL/SDA のノイズ除去（アンチスパイク）フィルターを常に有効にする
#define IF_CFG_ASF_CTRL         0x20
// CTRL3（Table 57）：bit6 BDU（既定 1）、bit2 IF_INC（既定 1）、bit0 SW_RESET（自動で 0 に戻る）
#define CTRL3_BDU               0x40
#define CTRL3_IF_INC            0x04
#define CTRL3_SW_RESET          0x01
// CTRL1（Table 51・52）：0000＝加速度は停止（既定）
#define CTRL1_XL_POWER_DOWN     0x00
// CTRL2（Table 54・55）：OP_MODE_G＝000（高性能モード）、ODR_G＝0110（120Hz）
#define CTRL2_G_HP_120HZ        0x06
// CTRL6（Table 63）：LPF1_G_BW＝000、FS_G＝0010（±500dps。感度 17.50mdps/LSB は機械的特性の表の G_So）
#define CTRL6_FS_500DPS         0x02
// FIFO_CTRL3（Table 38）：BDR_GY＝0110（ジャイロを 120Hz で FIFO へ）、BDR_XL＝0000（加速度は入れない）
#define FIFO_CTRL3_GY_120HZ     0x60
// FIFO_CTRL4（Table 40）：FIFO_MODE＝000（バイパス。FIFO を止めて空にする）／110（連続モード）
#define FIFO_CTRL4_BYPASS       0x00
#define FIFO_CTRL4_CONTINUOUS   0x06
// FIFO_STATUS2（Table 79）：bit3 FIFO_OVR_LATCHED（あふれ。読むと消える）、bit0 DIFF_FIFO_8（件数の9ビット目）
#define FIFO_STATUS2_OVR_LATCHED  0x08
#define FIFO_STATUS2_DIFF_8       0x01

// 実際の ODR（9.52節）：ODR = 7680 × (1 + 0.0013 × FREQ_FINE) ÷ ODRcoeff。120Hz の ODRcoeff は 64（Table 146）
#define ODR_BASE_HZ             7680.0f
#define ODR_FINE_STEP           0.0013f
#define ODR_COEFF_120HZ         64.0f

static bool WriteReg(uint8_t reg, uint8_t value) {
  return I2c_WriteReg(I2C_DEV_GYRO, I2C_ADDR_GYRO, reg, value);
}

static bool ReadRegs(uint8_t reg, uint8_t *buffer, uint8_t length) {
  return I2c_ReadRegs(I2C_DEV_GYRO, I2C_ADDR_GYRO, reg, buffer, length);
}

bool GyroHal_ReadWhoAmI(uint8_t *value) {
  return ReadRegs(REG_WHO_AM_I, value, 1);
}

bool GyroHal_StartReset(void) {
  // 既定値（BDU＝1・IF_INC＝1）に SW_RESET を足して書く
  return WriteReg(REG_CTRL3, CTRL3_BDU | CTRL3_IF_INC | CTRL3_SW_RESET);
}

bool GyroHal_IsResetDone(bool *done) {
  uint8_t ctrl3 = 0;
  if (!ReadRegs(REG_CTRL3, &ctrl3, 1)) {
    return false;
  }
  *done = (ctrl3 & CTRL3_SW_RESET) == 0;
  return true;
}

bool GyroHal_Configure(void) {
  // リセット直後なので、ほかのビットは既定値（0）。そのまま値を書く。
  // ジャイロの ODR（CTRL2）を最後に書く。ここからジャイロが動き出す（立ち上がり 30ms。電気的特性の表の Ton）
  return WriteReg(REG_FIFO_CTRL4, FIFO_CTRL4_BYPASS) &&
         WriteReg(REG_IF_CFG, IF_CFG_ASF_CTRL) &&
         WriteReg(REG_CTRL6, CTRL6_FS_500DPS) &&
         WriteReg(REG_CTRL1, CTRL1_XL_POWER_DOWN) &&
         WriteReg(REG_FIFO_CTRL3, FIFO_CTRL3_GY_120HZ) &&
         WriteReg(REG_CTRL2, CTRL2_G_HP_120HZ);
}

bool GyroHal_StartFifo(void) {
  return WriteReg(REG_FIFO_CTRL4, FIFO_CTRL4_CONTINUOUS);
}

bool GyroHal_ReadSettings(GyroHalSettings *settings) {
  uint8_t fifo[2];    // FIFO_CTRL3・FIFO_CTRL4
  uint8_t ctrl[3];    // CTRL1・CTRL2・CTRL3
  uint8_t freqFine = 0;
  if (!ReadRegs(REG_IF_CFG, &settings->ifCfg, 1) ||
      !ReadRegs(REG_FIFO_CTRL3, fifo, 2) ||
      !ReadRegs(REG_CTRL1, ctrl, 3) ||
      !ReadRegs(REG_CTRL6, &settings->ctrl6, 1) ||
      !ReadRegs(REG_INTERNAL_FREQ_FINE, &freqFine, 1)) {
    return false;
  }
  settings->fifoCtrl3 = fifo[0];
  settings->fifoCtrl4 = fifo[1];
  settings->ctrl1 = ctrl[0];
  settings->ctrl2 = ctrl[1];
  settings->ctrl3 = ctrl[2];
  settings->freqFine = (int8_t)freqFine;
  return true;
}

bool GyroHal_SettingsMatch(const GyroHalSettings &settings) {
  return settings.ifCfg == IF_CFG_ASF_CTRL &&
         settings.fifoCtrl3 == FIFO_CTRL3_GY_120HZ &&
         settings.fifoCtrl4 == FIFO_CTRL4_CONTINUOUS &&
         settings.ctrl1 == CTRL1_XL_POWER_DOWN &&
         settings.ctrl2 == CTRL2_G_HP_120HZ &&
         settings.ctrl3 == (CTRL3_BDU | CTRL3_IF_INC) &&
         settings.ctrl6 == CTRL6_FS_500DPS;
}

float GyroHal_ActualOdrHz(int8_t freqFine) {
  return ODR_BASE_HZ * (1.0f + ODR_FINE_STEP * (float)freqFine) / ODR_COEFF_120HZ;
}

bool GyroHal_ReadFifoStatus(uint16_t *words, bool *overrun) {
  uint8_t status[2];   // FIFO_STATUS1（件数の下位8ビット）・FIFO_STATUS2
  if (!ReadRegs(REG_FIFO_STATUS1, status, 2)) {
    return false;
  }
  *words = (uint16_t)status[0] | ((status[1] & FIFO_STATUS2_DIFF_8) ? 0x0100 : 0);
  *overrun = (status[1] & FIFO_STATUS2_OVR_LATCHED) != 0;
  return true;
}

bool GyroHal_ReadFifoWord(uint8_t *tag, int16_t xyz[3]) {
  // 1回の通信で1件（78h〜7Eh の7バイト）だけ読む。複数件を続けて読めるか（7Eh の次に 78h へ戻るか）は
  // データシートに記載を見つけられなかったので、頼らない
  uint8_t word[7];
  if (!ReadRegs(REG_FIFO_DATA_OUT_TAG, word, 7)) {
    return false;
  }
  *tag = word[0] >> 3;   // 上位5ビットが TAG_SENSOR
  xyz[0] = (int16_t)((uint16_t)word[1] | ((uint16_t)word[2] << 8));
  xyz[1] = (int16_t)((uint16_t)word[3] | ((uint16_t)word[4] << 8));
  xyz[2] = (int16_t)((uint16_t)word[5] | ((uint16_t)word[6] << 8));
  return true;
}
