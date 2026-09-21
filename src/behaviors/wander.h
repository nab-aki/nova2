// ID25 うろうろ（v1）。docs/specs/25_wander.md
// 止まって首で見回し、空いていそうな向きへ歩き、また止まる、を繰り返す。
// 車体レイヤーの既定の振る舞い（優先度がいちばん低い）。
//
//   [ため] → [正面] → [左] → [右] → [決める] →（横が近ければ片側旋回）
//        → [正面へ戻して安定を待つ] → [加速] → [巡航（ランダム）] → [減速] → [ため]
//
// 走行中に正面の障害物に気づいたら ID9 が、そのあとの立て直しは ID15 が引き継ぐ。
#ifndef NOVA_BEHAVIORS_WANDER_H
#define NOVA_BEHAVIORS_WANDER_H

#include "../core/behavior.h"
#include "../core/motion.h"
#include "../core/scan.h"

class WanderBehavior : public Behavior {
 public:
  const char *name() const override { return "うろうろ"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

 private:
  enum State {
    STATE_SETTLE,       // 止まってからの「ため」
    STATE_SCAN_FRONT,   // 正面を測る
    STATE_SCAN_LEFT,    // 左を測る
    STATE_SCAN_RIGHT,   // 右を測る
    STATE_FACE_FRONT,   // 首を正面へ戻す
    STATE_AVOID,        // 横が近いので片側旋回で向きを変える
    STATE_READY,        // 測距の履歴がたまるのを待つ
    STATE_ACCEL,
    STATE_CRUISE,
    STATE_DECEL
  };

  void Restart(unsigned long nowMs);
  void ChangeState(State next, unsigned long nowMs);
  void Decide(unsigned long nowMs);
  static const char *StateName(State state);

  State state_ = STATE_SETTLE;
  unsigned long stateStartMs_ = 0;
  NeckScan scan_;
  float frontCm_ = 0.0f;
  float leftCm_ = 0.0f;
  float rightCm_ = 0.0f;
  bool pendingPivot_ = false;      // 歩き出す前に向きを変えるか
  TurnKind pivotKind_ = TURN_PIVOT_LEFT;
  unsigned long runMs_ = 0;        // この回の巡航時間（ランダム）
  bool lifted_ = false;            // 持ち上げられている間は状態を進めない
};

#endif // NOVA_BEHAVIORS_WANDER_H
