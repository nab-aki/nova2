// ID15 障害物で困る（最小版 v1）。docs/specs/15_trouble.md
// ID9 が気づいて止まり、反応（目を見開く・停止後の距離を出す）が終わってもまだ塞がっているとき、
// 後退して首で左右を確かめ、空いているほうへその場回転で向きを変える。
//
//   [少し後退] → [左] → [右] → [決める] → [少し回る] → [止めて正面を測る]
//                                              ↑              ↓（まだ塞がっている）
//                                              └──────────────┘
//                              （一周してもだめ）→ [休む] → 最初へ
//
// v1 は立て直すことだけを扱う。首かしげと「？」の目は v2。
#ifndef NOVA_BEHAVIORS_TROUBLE_H
#define NOVA_BEHAVIORS_TROUBLE_H

#include "../config.h"
#include "../core/behavior.h"
#include "../core/motion.h"
#include "../core/scan.h"
#include "notice.h"

class TroubleBehavior : public Behavior {
 public:
  explicit TroubleBehavior(const NoticeBehavior *notice) : notice_(notice) {}

  const char *name() const override { return "困る"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 立て直しの最中か（デバッグキーを受け付けてよいかの判断に使う）
  bool isBusy() const { return active_; }

  // t キー：直近 TROUBLE_LOG_COUNT 回の立て直しの経過（見回し・回転ごとの測り直し・終わり方）。
  // 記録するだけで、動きは変えない。RAM だけに持つ（電源を切ると消える）
  void printLog() const;

 private:
  enum State {
    STATE_BACK,        // 少し後退する
    STATE_SCAN_LEFT,   // 左を測る
    STATE_SCAN_RIGHT,  // 右を測る
    STATE_TURN,        // その場回転を1ステップ
    STATE_CHECK,       // 止めて正面を測り直す
    STATE_REST         // あきらめて休む
  };

  void StartSequence(unsigned long nowMs);
  void ChangeState(State next, unsigned long nowMs);
  void Decide(unsigned long nowMs);
  void StartTurnStep(unsigned long nowMs);
  static const char *StateName(State state);

  const NoticeBehavior *notice_;
  bool active_ = false;
  State state_ = STATE_BACK;
  unsigned long stateStartMs_ = 0;
  NeckScan scan_;

  float backStartCm_ = -1.0f;   // 後退する前の正面の距離
  bool backStopping_ = false;   // 後退の減速に入ったか

  float leftCm_ = 0.0f;
  float rightCm_ = 0.0f;
  int leftValid_ = 0;           // 左右それぞれ、測れた回数（0なら「測れず」。cm は ULTRASONIC_MAX_CM になっている）
  int rightValid_ = 0;
  bool lastTurnLeft_ = false;   // 前回どちらへ回ったか（差が小さいときは反対を選ぶ）
  TurnKind turnKind_ = TURN_ROTATE_LEFT;
  int steps_ = 0;               // この立て直しで回った回数

  bool checkReset_ = false;       // 測り直しの履歴をリセット済みか
  unsigned long checkResetMs_ = 0;
  float checkClosestCm_ = 0.0f;   // リセット後に測った中で最も近い有効値
  int checkValidCount_ = 0;

  bool lifted_ = false;

  // ------------------------ 経過の記録（原因調べ用）------------------------ //
  enum LogEnd { LOG_RUNNING, LOG_CLEARED, LOG_GIVEUP, LOG_ABORTED };
  struct StepLog {
    float closestCm;      // 測り直しで得た最も近い有効値（1回も測れなければ ULTRASONIC_MAX_CM のまま）
    uint8_t validCount;   // 有効な測距の数
    uint8_t nearCount;    // 共通部品の履歴のうち「近い」の数
    bool blocked;         // 共通部品の判定（あり＝true）
    float yawDeg;         // 判定したときのジャイロの向き（積算の角度）
    float yawMovedDeg;    // 測り直しの間（履歴を捨ててから判定まで）に動いた向き。0 に近ければ、止まって測れている
  };
  struct SeqLog {
    unsigned long startMs;
    bool scanned;         // 左右の見回しまで進んだか
    float leftCm;
    float rightCm;
    uint8_t leftValid;
    uint8_t rightValid;
    bool turnLeft;
    uint8_t steps;        // 回転の回数
    uint8_t end;          // LogEnd
    StepLog step[TROUBLE_MAX_STEPS];
  };
  void LogBegin(unsigned long nowMs);
  void LogFinish(LogEnd end);

  SeqLog logs_[TROUBLE_LOG_COUNT];
  int logHead_ = 0;             // 次に書く位置
  int logCount_ = 0;
  SeqLog *logNow_ = nullptr;    // いま書いている記録
  float checkYawStartDeg_ = 0.0f;
};

#endif // NOVA_BEHAVIORS_TROUBLE_H
