#include "hal_matrix.h"

#include <Wire.h>

#include "../config.h"
#include "hal_i2c.h"
#include "hal_pins.h"

// 公式ライブラリのヘッダは uint8_t / uint16_t をマクロで再定義するため、
// 他のヘッダの後に読み込み、直後に元へ戻す
#include <Freenove_VK16K33_Lib_For_ESP32.h>
#undef uint8_t
#undef uint16_t

static Freenove_ESP32_VK16K33 matrix;

// ライブラリが数えた送信の回数・失敗（lib/Freenove_VK16K33_Lib_For_ESP32/NOVA_PATCH.md）のうち、前回までに集計へ移した分
static uint32_t syncedTotal = 0;
static uint32_t syncedFail = 0;
static uint32_t syncedTimeout = 0;

// ライブラリを呼んだあとに、増えた分を機器ごとの集計（hal_i2c）へ移す
static void SyncI2cStats(void) {
  uint32_t total = (uint32_t)matrix.nova_i2c_total;
  uint32_t fail = (uint32_t)matrix.nova_i2c_fail;
  uint32_t timeout = (uint32_t)matrix.nova_i2c_timeout;
  I2c_AddStats(I2C_DEV_MATRIX, total - syncedTotal, fail - syncedFail, timeout - syncedTimeout);
  syncedTotal = total;
  syncedFail = fail;
  syncedTimeout = timeout;
}

void Matrix_Setup(void) {
  // I2Cバス（同じSDA/SCL）は I2c_Setup() で初期化済み。ライブラリ側も同じピンで Wire.begin() を呼ぶが、
  // 初期化済みのバスはそのまま使われる（速度・タイムアウトは変わらない）
  matrix.init(I2C_ADDR_MATRIX, PIN_I2C_SDA, PIN_I2C_SCL);
  matrix.setBlink(VK16K33_BLINK_OFF);
  matrix.setBrightness(MATRIX_BRIGHTNESS);
  SyncI2cStats();
}

void Matrix_Show(const uint8_t *left, const uint8_t *right) {
  // ライブラリの引数は非const（読み取りのみ）
  matrix.showStaticArray((byte *)left, (byte *)right);
  SyncI2cStats();
}

void Matrix_SetBrightness(uint8_t brightness) {
  matrix.setBrightness(constrain(brightness, 0, 15));
  SyncI2cStats();
}

void Matrix_Clear(void) {
  static const uint8_t blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  Matrix_Show(blank, blank);
}
