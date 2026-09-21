// IR受信（リモコン）のラッパー。ID20「呼びかけ」でも使う前提で、ボタンのコードを返すだけにしている。
// ピンは Freenove 公式サンプル（05.1〜05.3）と同じ GPIO0。ライブラリは IRremoteESP8266
// （公式の Freenove_IR_Lib は新しい RMT ドライバ用で、Arduino-ESP32 2.0.17 ではビルドできないため。
//   docs/specs/common_ir.md）。待たずに結果を返す。
//
// 押しっぱなしのリピート（NEC のリピート信号）と、同じコードの短い間隔の繰り返しは、呼ぶ側に返さない。
// 受け取ったボタンは、シリアルに1行出す（どのボタンが何のコードか確かめるため）。
#ifndef NOVA_HAL_IR_H
#define NOVA_HAL_IR_H

#include <Arduino.h>

void Ir_Setup(void);

// loop() から毎回呼ぶ。新しく押されたボタンがあれば true を返し、コードを code に入れる。
// 押しっぱなしのリピートは false。
bool Ir_Poll(uint32_t *code);

// Freenove のリモコンのボタン名（公式サンプル 05.2・05.3 のコード表）。未登録なら NULL
const char *Ir_ButtonName(uint32_t code);

#endif // NOVA_HAL_IR_H
