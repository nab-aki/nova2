// デバッグ：止まったまま見回しをくり返す（#0。docs/notes.md 2章。キー o）
// うろうろ（ID25）と同じ見回し（ため → 正面 → 左 → 右 → 正面へ戻す → 測距の履歴がたまるのを待つ）を、
// 同じ部品（NeckScan）・同じ回数・同じ待ち時間で DEBUG_SCAN_COUNT 回くり返す。車体は動かさない。
//
// 目的：首が毎回左右に動くか、3方向の測距がばらつかないか、見回しが途中で打ち切られるかを、
// ケーブルをつないだまま確かめる。ID16「見回し探索」の土台にもする。
//
// 障害物の判定・安全層・気づく（ID9）・首の調停は、うろうろ中と同じように働く。
// うろうろと同じ優先度（PRIORITY_WANDER）で調停を通すので、ID9 などが割り込めば、うろうろと同じように打ち切られる。
// 一時停止中だけ受け付ける（うろうろ・困る・詰まり脱出は一時停止中は動かないので、割り込むのは ID9 と持ち上げ）。
#ifndef NOVA_BEHAVIORS_DEBUG_SCAN_H
#define NOVA_BEHAVIORS_DEBUG_SCAN_H

#include "../core/behavior.h"
#include "../core/scan.h"

class DebugScanBehavior : public Behavior {
 public:
  const char *name() const override { return "見回し試験"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 試験を予約する。次の調停で始まる（受け付けてよいかは main.cpp が見る）
  void request();

  // 予約済み、または実行中か
  bool isBusy() const { return busy_; }

  // 中断する（キー・リモコン）。ここまでのまとめを出す
  void abort(const char *reason);

 private:
  enum State {
    STATE_SETTLE,       // ため
    STATE_SCAN_FRONT,
    STATE_SCAN_LEFT,
    STATE_SCAN_RIGHT,
    STATE_FACE_FRONT,   // 首を正面へ戻して安定を待つ
    STATE_READY,        // 測距の履歴がたまるのを待つ
    STATE_COUNT
  };
  enum Dir { DIR_FRONT, DIR_LEFT, DIR_RIGHT, DIR_COUNT };
  enum CutReason { CUT_LIFT, CUT_NOTICE, CUT_OTHER, CUT_COUNT };

  void BeginIteration(unsigned long nowMs);
  void StoreScan(Dir dir, unsigned long nowMs);
  void FinishIteration(unsigned long nowMs);
  void NoteCut(CutReason reason, unsigned long nowMs);
  void PrintSummary(const char *why);
  static const char *StateName(State state);

  bool busy_ = false;
  bool fresh_ = false;          // 予約直後（集計をやり直す）
  bool inIteration_ = false;    // 1回の見回しの途中か
  bool lifted_ = false;
  State state_ = STATE_SETTLE;
  unsigned long stateStartMs_ = 0;
  int iteration_ = 0;           // 何回目か（打ち切られた回も数える）
  NeckScan scan_;

  // この回の結果
  float cm_[DIR_COUNT];
  int valid_[DIR_COUNT];
  int panDeg_[DIR_COUNT];           // 測り終えたときの首の角度（指示した値）
  unsigned long tookMs_[DIR_COUNT]; // その方向にかかった時間（首を向ける待ち＋測距）

  // まとめ
  int completed_ = 0;
  int measured_[DIR_COUNT];     // 有効な値が取れた回数
  int noEcho_[DIR_COUNT];       // 1回も測れなかった回数（80cm 扱い）
  float minCm_[DIR_COUNT];
  float maxCm_[DIR_COUNT];
  double sumCm_[DIR_COUNT];
  int cutByReason_[CUT_COUNT];
  int cutByState_[STATE_COUNT];
};

#endif // NOVA_BEHAVIORS_DEBUG_SCAN_H
