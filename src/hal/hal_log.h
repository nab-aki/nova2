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

// 出力にかかった時間の記録（起動からの累計）。シリアルの送信バッファが 0 なので、
// 128 バイトを超える行は送り終わるまで戻らない。loop を止める時間の原因を切り分けるために測る
// （docs/specs/common_gyro_turn.md 段1）
struct LogStats {
  uint32_t count;       // 出力した行数
  uint32_t slowCount;   // LOG_SLOW_US を超えた回数
  uint32_t maxUs;       // 1行にかかった最大の時間
  uint32_t maxBytes;    // そのときのバイト数
  uint32_t totalUs;     // 合計の時間（平均を出すため）
  uint32_t totalBytes;
};
const LogStats &Log_GetStats(void);

#endif // NOVA_HAL_LOG_H
