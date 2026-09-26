// ID25 うろうろ（v1）。docs/specs/25_wander.md
// 止まって首で見回し、空いていそうな向きへ歩き、また止まる、を繰り返す。
// 車体レイヤーの既定の振る舞い（優先度がいちばん低い）。
//
//   [ため] → [正面] → [左] → [右] → [決める] →（横がとても近ければ後退+その場回転／近ければその場回転で向き変え）
//        → [正面へ戻して安定を待つ] → [加速] → [巡航（ランダム）] → [減速] → [ため]
//
// 横が近いとき（WANDER_SIDE_NEAR_CM 未満、VERY_NEAR 以上）の向き変えは、2026-09-25 まで片側旋回を
// 使っていたが、その場回転（1ステップ・約30°）に置き換えた。片側旋回は止める側が惰性で1回に約20cm
// 前進しており、壁から離れたい場面でかえって近づいていたため（docs/decisions.md 2026-09-25）。
// 片側旋回のコード自体は消さず、広い場所での方向転換として将来使い直す（docs/specs/25_wander.md）。
//
// 横がとても近い（WANDER_SIDE_VERY_NEAR_CM 未満）ときの後退+その場回転は、
// 片側旋回が安全層に止められ続けて張りつく問題（2026-09-22 の5分間試験）への対策。
// 後退・その場回転の間は ID15 の立て直しと同じく、気づく（ID9）に割り込まれず最後までやり切る。
// 後退が止まってから回転を始めるまでに WANDER_RECOVER_PAUSE_MS の「ため」を挟む
// （なめらか加減速の出力が0になっても、車体は慣性で少し動き続けている可能性があり、
//  間を置かず逆向きの回転を始めると大電流になりブラウンアウトを起こすため。2026-09-22）。
//
// 走行中に正面の障害物に気づいたら ID9 が、そのあとの立て直しは ID15 が引き継ぐ。
// 巡航中の測距・巡航の終わり・見回しの結果は詰まりの見張り（core/stuck_watch.*）に渡し、
// 進めていないと分かったら ID26 詰まり脱出が引き継ぐ。
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
    STATE_SETTLE,        // 止まってからの「ため」
    STATE_SCAN_FRONT,    // 正面を測る
    STATE_SCAN_LEFT,     // 左を測る
    STATE_SCAN_RIGHT,    // 右を測る
    STATE_FACE_FRONT,    // 首を正面へ戻す
    STATE_AVOID,         // 横が近いのでその場回転で向きを変える（片側旋回は今は使わない）
    STATE_RECOVER_BACK,  // 横がとても近いので、少し後退する（張りつき対策）
    STATE_RECOVER_PAUSE, // 後退のあと、回転を始める前の「ため」（ブラウンアウト対策）
    STATE_RECOVER_TURN,  // 反対側へその場回転する
    STATE_READY,         // 測距の履歴がたまるのを待つ
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
  int frontValid_ = 0;             // 各方向、測れた回数（0なら「測れず」。ID26 の見張りに渡す）
  int leftValid_ = 0;
  int rightValid_ = 0;
  bool pendingAvoidTurn_ = false;  // 歩き出す前にその場回転で向きを変えるか（横が近いとき）
  TurnKind avoidTurnKind_ = TURN_ROTATE_LEFT;
  bool pendingRecover_ = false;    // 歩き出す前に後退+その場回転で避けるか（横がとても近いとき）
  TurnKind recoverKind_ = TURN_ROTATE_LEFT;
  bool recoverBackStopping_ = false;  // 後退の減速に入ったか（trouble.cpp の backStopping_ と同じ考え方）
  unsigned long runMs_ = 0;        // この回の巡航時間（ランダム）
  bool lifted_ = false;            // 持ち上げられている間は状態を進めない
};

#endif // NOVA_BEHAVIORS_WANDER_H
