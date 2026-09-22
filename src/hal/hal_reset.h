// 起動時のリセット理由（esp_reset_reason() のラッパー）
// ケーブルなしの試験のあとでも、電圧低下（ブラウンアウト）で再起動していないかを
// 確かめられるようにする。core/test_stats.* が試験の記録に残す。
#ifndef NOVA_HAL_RESET_H
#define NOVA_HAL_RESET_H

#include <Arduino.h>

// 今のところ何もしないが、他の hal ファイルと形をそろえる
void Reset_Setup(void);

// このブートのリセット理由（esp_reset_reason_t の値をそのまま返す。保存にも使うので固定サイズの数値）
uint8_t Reset_ReasonCode(void);

// コードから、表示用の日本語名を返す（不明な値は「不明」）
const char *Reset_ReasonName(uint8_t code);

// 電圧低下（ブラウンアウト）によるリセットか
bool Reset_IsBrownout(uint8_t code);

#endif // NOVA_HAL_RESET_H
