#include "hal_matrix.h"

#include <Wire.h>

#include "../config.h"
#include "hal_pins.h"

// 公式ライブラリのヘッダは uint8_t / uint16_t をマクロで再定義するため、
// 他のヘッダの後に読み込み、直後に元へ戻す
#include <Freenove_VK16K33_Lib_For_ESP32.h>
#undef uint8_t
#undef uint16_t

static Freenove_ESP32_VK16K33 matrix;

void Matrix_Setup(void) {
  // I2Cバス（同じSDA/SCL）は Pca9685_Setup() で初期化済み。ライブラリ側でも同じピンで再初期化される
  matrix.init(I2C_ADDR_MATRIX, PIN_I2C_SDA, PIN_I2C_SCL);
  matrix.setBlink(VK16K33_BLINK_OFF);
  matrix.setBrightness(MATRIX_BRIGHTNESS);
}

void Matrix_Show(const uint8_t *left, const uint8_t *right) {
  // ライブラリの引数は非const（読み取りのみ）
  matrix.showStaticArray((byte *)left, (byte *)right);
}

void Matrix_SetBrightness(uint8_t brightness) {
  matrix.setBrightness(constrain(brightness, 0, 15));
}

void Matrix_Clear(void) {
  static const uint8_t blank[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  Matrix_Show(blank, blank);
}
