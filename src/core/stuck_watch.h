// 詰まりの見張り（ID26 の共通部品。docs/specs/26_stuck.md）
// ID25 うろうろから、巡航中の測距・巡航の終わり・見回しの結果を受け取り、
// 「前に進もうとしているのに進めていない」ことに気づく。気づいたら ID26 詰まり脱出が発動する。
//
//   信号A（押しても近づかない）：巡航中、正面の有効な測距が STUCK_PUSH_WINDOW_MS の間に、
//     期待した縮み（経過時間×CRUISE_CM_PER_S）の STUCK_UNCHANGED_RATIO 倍も縮まない。単独で確定する。
//   信号B（見回しが変わらない）：歩いたのに、見回し（正面・左右 ±50°）が前回と「同じ」が続く。
//     測れた方向だけを比べる。正面不変・横が接触は1回、横だけは2回で確定する。
//
// 数えのやり直しは ID25 が行う（交代したときと、持ち上げから戻ったとき）。
#ifndef NOVA_CORE_STUCK_WATCH_H
#define NOVA_CORE_STUCK_WATCH_H

#include <Arduino.h>

#include "sensors.h"

// 見回しの方向
enum StuckScanDir {
  STUCK_DIR_FRONT,
  STUCK_DIR_LEFT,
  STUCK_DIR_RIGHT,
  STUCK_DIR_COUNT
};

// 見回し1回の結果（NeckScan の値と有効回数。有効0回なら「測れず」）
struct StuckScanResult {
  float cm[STUCK_DIR_COUNT];
  int valid[STUCK_DIR_COUNT];
};

// 何で気づいたか
enum StuckReason {
  STUCK_REASON_PUSH,      // 信号A：巡航中に正面が縮まない
  STUCK_REASON_FRONT,     // 信号B：正面不変
  STUCK_REASON_CONTACT,   // 信号B：横が接触（WANDER_SIDE_VERY_NEAR_CM 未満）のまま同じ
  STUCK_REASON_SIDE       // 信号B：横だけ（25〜60cm）が同じ
};

// 壁の側（回る向きの手がかり）
enum StuckWallSide {
  STUCK_WALL_UNKNOWN,
  STUCK_WALL_LEFT,
  STUCK_WALL_RIGHT
};

struct StuckEvent {
  StuckReason reason;
  StuckWallSide wall;
};

// 数えをやり直す。dropDetection=true なら、まだ受け取られていない「気づいた」も捨てる
// （一時停止など。ID26 に交代するときは false にして、気づいた内容を残す）
void StuckWatch_Reset(bool dropDetection);

// ID25 の巡航が始まった（巡航速度に達した）とき
void StuckWatch_OnCruiseStart(void);

// ID25 の巡航中、毎ループ呼ぶ（信号A）
void StuckWatch_OnCruiseSample(const SensorData &sensors, unsigned long nowMs);

// ID25 の巡航が最後まで終わったとき。cruiseMs＝巡航の時間（加速・減速は含まない）
void StuckWatch_OnWalked(unsigned long cruiseMs);

// ID25 の見回しが終わったとき（信号B）
void StuckWatch_OnScan(const StuckScanResult &result, unsigned long nowMs);

// 次の巡航を短くしてほしいか（信号B で1回目の「同じ」が出て、確定しなかったとき）
bool StuckWatch_WantShortRun(void);

// 詰まりに気づいていて、まだ ID26 が受け取っていないか
bool StuckWatch_IsDetected(void);

// 気づいた内容を受け取る（受け取ったら「気づいた」は消える）
StuckEvent StuckWatch_Take(void);

const char *StuckWatch_ReasonName(StuckReason reason);
bool StuckWatch_IsSignalA(StuckReason reason);

#endif // NOVA_CORE_STUCK_WATCH_H
