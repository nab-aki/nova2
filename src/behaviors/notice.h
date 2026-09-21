// ID9 気づく（v1）。docs/specs/09_notice.md
// 「前進 → 停止」の繰り返しの中で、正面の障害物に気づいたら即停止し、
// 目を見開いて首を正面に向けたまま待つ。なくなったら、ゆっくり前進を再開する。
// 方向転換はまだ行わない（ID15・ID25 としてスプリント2で扱う）。
//
// 車体と目の2つのレイヤーを使うため、このファイルに2つの振る舞いを置く。
#ifndef NOVA_BEHAVIORS_NOTICE_H
#define NOVA_BEHAVIORS_NOTICE_H

#include "../core/behavior.h"

class NoticeBehavior : public Behavior {
 public:
  const char *name() const override { return "気づく"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 目レイヤー側から参照する
  bool isNoticing() const { return state_ == STATE_NOTICED; }
  unsigned long noticedAtMs() const { return noticedAtMs_; }

 private:
  enum State {
    STATE_REST,      // 止まって待っている
    STATE_ACCEL,     // ゆっくり加速
    STATE_CRUISE,    // 巡航
    STATE_DECEL,     // ゆっくり減速
    STATE_NOTICED    // 気づいて止まり、空くのを待っている
  };

  void ChangeState(State next, unsigned long nowMs);
  void EnterNoticed(unsigned long nowMs);
  void ReportStop(unsigned long nowMs);
  static const char *StateName(State state);

  State state_ = STATE_REST;
  unsigned long stateStartMs_ = 0;
  unsigned long noticedAtMs_ = 0;
  unsigned long clearSinceMs_ = 0;
  bool clearTimerOn_ = false;    // 障害物がなくなってからの待ち時間を数えているか
  bool stopReported_ = false;    // 停止後の距離をシリアルに出したか
  float noticedCm_ = -1.0f;      // 気づいたときの距離
  float noticedSpeed_ = 0.0f;    // 気づいたときの接近速度（cm/s）
  bool noticedSpeedOk_ = false;
};

// 気づいた直後だけ目を見開く（まばたきより優先）。
// NOTICE_WIDE_MS が過ぎたら優先度を0にして、まばたきに戻す
// （止まって待っている間もまばたきするほうが、生き物らしく見えるため）。
class NoticeEyesBehavior : public Behavior {
 public:
  explicit NoticeEyesBehavior(const NoticeBehavior *body) : body_(body) {}

  const char *name() const override { return "気づく（見開く）"; }
  BehaviorLayer layer() const override { return LAYER_EYES; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

 private:
  const NoticeBehavior *body_;
};

#endif // NOVA_BEHAVIORS_NOTICE_H
