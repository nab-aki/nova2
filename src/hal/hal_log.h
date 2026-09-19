// シリアル出力（115200bps）の窓口
// センサー値と状態遷移は、必ずここ経由で出力する。書式：[起動からのms][タグ] メッセージ
#ifndef NOVA_HAL_LOG_H
#define NOVA_HAL_LOG_H

#include <Arduino.h>

void Log_Setup(void);

// printf形式でタグ付きの1行を出力する
void Log_Printf(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

#endif // NOVA_HAL_LOG_H
