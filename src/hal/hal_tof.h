// ToF（VL53L8CX。8×8）。専用のタスクが Wire1 で読み、最新の1枚を置く（docs/specs/common_tof_scan.md 6章）。
// センサーに触るのはこのタスクだけ。loop は Tof_GetLatest で受け取るだけで、I2C を待たない。
// 走行中に呼ぶのは check_data_ready と get_ranging_data だけ。init のやり直し・stop_ranging はしない
// （docs/specs/common_tof_checklist.md「本体に組み込むときの注意」）。
#ifndef NOVA_HAL_TOF_H
#define NOVA_HAL_TOF_H

#include <Arduino.h>

#define TOF_ZONES 64   // 区画の番号 z = 行 × 8 + 列（行＝上下 r0 上、列＝左右 c0 左。Nova の後ろから前を見て）

struct TofFrame {
  uint32_t seq;              // フレーム番号（1から。0 はまだ1枚も届いていない）
  uint32_t readyMs;          // 読み終えた時刻（millis）
  int16_t distanceMm[TOF_ZONES];
  uint8_t status[TOF_ZONES];
};

enum TofState {
  TOF_STATE_STARTING,   // 起動中（is_alive・init）
  TOF_STATE_RUNNING,    // 測距中
  TOF_STATE_FAILED,     // 起動に失敗。この起動中は使わない
  TOF_STATE_DISABLED    // TOF_ENABLE が 0
};

// 起動の結果と、測距の数（タスクが数え、loop が読む）
struct TofStats {
  TofState state;
  uint8_t aliveStatus;     // 最後の is_alive の戻り値（5＝タイムアウト）
  uint8_t alive;
  uint8_t aliveTries;      // is_alive を試した回数
  uint8_t failStep;        // 失敗した段階の戻り値（設定・start_ranging・init）
  const char *failName;    // 失敗した段階の名前（なければ NULL）
  int sdaLevel;            // is_alive が失敗したときの線のレベル（まだ見ていなければ -1）
  int sclLevel;
  uint32_t initMs;         // init にかかった時間
  uint32_t frames;         // get_ranging_data が成功した回数
  uint32_t readyFail;      // check_data_ready の失敗
  uint32_t dataFail;       // get_ranging_data の失敗
  uint32_t periodSumMs;    // 周期の合計（平均用）
  uint32_t periodCount;
  uint32_t periodMaxMs;
  uint32_t readSumUs;      // 読み出しの時間の合計（平均用）
  uint32_t readMaxUs;
  uint32_t stackFree;      // タスクのスタックの残りの最小（バイト）
};

// タスクを作るだけで、すぐ戻る。is_alive・init（約2.7秒）はタスクの中で行う
void Tof_Setup(void);

// lastSeq より新しい1枚があれば out に写して true
bool Tof_GetLatest(uint32_t lastSeq, TofFrame *out);

void Tof_GetStats(TofStats *out);

#endif // NOVA_HAL_TOF_H
