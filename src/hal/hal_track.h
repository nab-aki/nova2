// ライントラッキングセンサー（PCF8574経由）のラッパー
// このモジュールは比較器で判定したデジタル値（0/1）のみ返す。アナログ値は取得できない。
#ifndef NOVA_HAL_TRACK_H
#define NOVA_HAL_TRACK_H

#include <Arduino.h>

// 初期化。センサーが応答しなければ false
bool Track_Setup(void);

// 3ビットの値を返す：bit0=左、bit1=中央、bit2=右（公式サンプルと同じ並び）
uint8_t Track_Read(void);

// 直前の Track_Read() が読めたか。読めなかったときの戻り値は前回の値（信用しない）
bool Track_LastReadOk(void);

#endif // NOVA_HAL_TRACK_H
