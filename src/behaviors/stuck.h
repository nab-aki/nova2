// ID26 詰まり脱出（v1）。docs/specs/26_stuck.md
// 詰まりの見張り（core/stuck_watch.*）が「進めていない」ことに気づいたら、
// 止まって「？」の目で考えてから、後退とその場回転で抜け出す。
//
//   [止まる・首を正面へ] → [考える（？の目）] → [後退（長め）] → [ため] → [壁と反対へその場回転（ランダムな角度）]
//        → ID25 に戻る（見回しから）
//
// ID15 と違い、正面の測距は信用しない（浅い壁は返らない）。測り直さず、決めた角度だけ回る。
// 脱出してから STUCK_REPEAT_MS 以内にまた詰まったら、大きく回る（くり返し）。
// 優先度は ID15 より下、ID25 の後退+その場回転・ID9 より上（後退・回転は最後までやり切る）。
#ifndef NOVA_BEHAVIORS_STUCK_H
#define NOVA_BEHAVIORS_STUCK_H

#include "../core/behavior.h"
#include "../core/motion.h"
#include "../core/stuck_watch.h"

class StuckBehavior : public Behavior {
 public:
  const char *name() const override { return "詰まり脱出"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 脱出の最中か（デバッグキーを受け付けてよいかの判断に使う）
  bool isBusy() const { return active_; }

  // 「？」の目を出しているべきか（気づいてから後退を始めるまで）
  bool isThinking() const { return active_ && (state_ == STATE_SETTLE || state_ == STATE_THINK); }

 private:
  enum State {
    STATE_SETTLE,   // 止まって、首を正面へ戻す
    STATE_THINK,    // 「？」の目で考える
    STATE_BACK,     // 後退する
    STATE_PAUSE,    // 後退のあと、回転を始める前の「ため」
    STATE_TURN      // その場回転
  };

  void ChangeState(State next, unsigned long nowMs);
  void DecideTurn(unsigned long nowMs);
  void Finish(unsigned long nowMs);
  static const char *StateName(State state);

  bool active_ = false;
  State state_ = STATE_SETTLE;
  unsigned long stateStartMs_ = 0;
  StuckEvent event_ = {STUCK_REASON_PUSH, STUCK_WALL_UNKNOWN};
  bool repeat_ = false;              // くり返し詰まったか
  bool backStopping_ = false;        // 後退の減速に入ったか（trouble.cpp と同じ考え方）
  TurnKind turnKind_ = TURN_ROTATE_LEFT;
  int turnDeg_ = 0;
  unsigned long turnMs_ = 0;

  bool escapedOnce_ = false;         // 一度でも脱出したか
  unsigned long escapedMs_ = 0;      // 最後に脱出した時刻
  bool lastTurnLeft_ = false;        // 最後に脱出したときに回った向き
};

#endif // NOVA_BEHAVIORS_STUCK_H
