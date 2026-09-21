// シリアル出力（115200bps）の窓口
// センサー値と状態遷移は、必ずここ経由で出力する。書式：[起動からのms][タグ] メッセージ
#ifndef NOVA_HAL_LOG_H
#define NOVA_HAL_LOG_H

#include <Arduino.h>

void Log_Setup(void);

// printf形式でタグ付きの1行を出力する
void Log_Printf(const char *tag, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

// 行頭の時刻・タグを付けずに1行出す。config.h や measurements.md に、そのまま貼れる行を出すときに使う
void Log_Raw(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif // NOVA_HAL_LOG_H
