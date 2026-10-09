// ピン番号・I2Cアドレス（Freenove 公式サンプルの定義に従う。独自に変更しない）
//   https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32
//   Sketches/06.3_Multi_Functional_Car, 01.5_Matrix, 01.6_WS2812, 02.1_Ultrasonic_Ranging, 05.1_IR_Receiver ほか
#ifndef NOVA_HAL_PINS_H
#define NOVA_HAL_PINS_H

// I2Cバス（PCA9685・LEDマトリクス・PCF8574・ジャイロが共有）
#define PIN_I2C_SDA          13
#define PIN_I2C_SCL          14

// PCA9685（モーター・首サーボ）
#define I2C_ADDR_PCA9685     0x5F
#define PCA9685_CH_SERVO1    0      // servo1（左右）
#define PCA9685_CH_SERVO2    1      // servo2（上下）
#define PCA9685_CH_M1_IN1    15
#define PCA9685_CH_M1_IN2    14
#define PCA9685_CH_M2_IN1    9
#define PCA9685_CH_M2_IN2    8
#define PCA9685_CH_M3_IN1    12
#define PCA9685_CH_M3_IN2    13
#define PCA9685_CH_M4_IN1    10
#define PCA9685_CH_M4_IN2    11
#define PCA9685_PWM_MAX      4095   // PCA9685のパルス幅の上限（公式サンプル準拠）

// モーターの向き（逆に回るときは 1 を -1 にする。公式サンプルと同じ）
#define MOTOR_1_DIRECTION    1
#define MOTOR_2_DIRECTION    1
#define MOTOR_3_DIRECTION    1
#define MOTOR_4_DIRECTION    1

// LEDマトリクス（VK16K33。左右2面）
#define I2C_ADDR_MATRIX      0x71

// ライントラッキング（PCF8574）
#define I2C_ADDR_TRACK       0x20

// ジャイロ（LSM6DSV16X。秋月電子 AE-LSM6DSV16X。キットにはない追加の部品）
// モジュールの J3 を短絡して SA0＝GND にしたので 0x6A（J2 側なら 0x6B。docs/specs/common_gyro_checklist.md）
#define I2C_ADDR_GYRO        0x6A

// 超音波（PCA9685を経由せずGPIOに直結）
#define PIN_SONIC_TRIG       12
#define PIN_SONIC_ECHO       15

// アナログ入力
#define PIN_BATTERY          32
#define PIN_LIGHT            33

// WS2812 ×12
// 【注意】公式定義では電池電圧（PIN_BATTERY）と同じ GPIO32。両方を同時に使う前に要確認（docs/decisions.md）。
#define PIN_WS2812           32
#define WS2812_COUNT         12
#define WS2812_RMT_CHANNEL   0

// IR受信（リモコン）。公式サンプル 05.1〜05.3 の RECV_PIN
// 【注意】GPIO0 は起動時のモード選択（strapping）のピン。受信モジュールは待機時に High なので通常は問題ないが、
// 起動の瞬間にリモコンを押すと起動を妨げるおそれがある（カメラのクロックと共用だが、カメラは未使用）。
#define PIN_IR_RECV          0

// パッシブブザー
#define PIN_BUZZER           2
#define BUZZER_LEDC_CHANNEL  0
#define BUZZER_LEDC_BITS     10

#endif // NOVA_HAL_PINS_H
