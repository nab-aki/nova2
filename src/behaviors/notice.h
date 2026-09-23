// ID9 気づく（v1）。docs/specs/09_notice.md
// 正面の障害物に気づいたら即停止し、目を見開いて首を正面に向けたまま待つ。
//
// スプリント2で走行ループ（停止→加速→巡航→減速）は ID25「うろうろ」に移し、
// この振る舞いは「気づいた瞬間の反応」だけを受け持つ（docs/specs/25_wander.md）。
//   ・障害物があるときだけ発動する（それ以外は優先度0で、ID25 が動く）
//   ・反応（NOTICE_REACT_MS）が終わってもまだ塞がっていれば、ID15「障害物で困る」が引き継ぐ
//   ・停止のたびに、停止後の距離と通算回数を出す（ID9 の完了条件の確認用）
//
// 車体と目の2つのレイヤーを使うため、このファイルに2つの振る舞いを置く。
#ifndef NOVA_BEHAVIORS_NOTICE_H
#define NOVA_BEHAVIORS_NOTICE_H

#include "../config.h"
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

  // 気づいた反応（目を見開く・停止後の距離を出す）が終わったか。
  // ID15「障害物で困る」は、これが終わってから引き継ぐ（設計原則1の順を守るため）
  bool reactionDone(unsigned long nowMs) const {
    return state_ == STATE_NOTICED && (long)(nowMs - noticedAtMs_) >= (long)NOTICE_REACT_MS;
  }

  int stopCount() const { return stopCount_; }

 private:
  enum State {
    STATE_IDLE,      // 何もしていない（ID25 が動いている）
    STATE_NOTICED    // 気づいて止まり、空くのを待っている
  };

  void EnterNoticed(unsigned long nowMs);
  void ReportStop(unsigned long nowMs);

  State state_ = STATE_IDLE;
  unsigned long noticedAtMs_ = 0;
  unsigned long clearSinceMs_ = 0;
  bool clearTimerOn_ = false;    // 障害物がなくなってからの待ち時間を数えているか
  bool stopReported_ = false;    // 停止後の距離をシリアルに出したか
  float noticedCm_ = -1.0f;      // 気づいたときの距離
  float noticedSpeed_ = 0.0f;    // 気づいたときの接近速度（cm/s）
  bool noticedSpeedOk_ = false;
  bool noticedWhileMoving_ = false;  // 走っているときに気づいたか（止まったまま気づいた分は数えない）
  bool noticedWhileTurning_ = false; // 回転中に気づいたか（Motion_StartTurn がスムーザーを0に戻すため、
                                      // noticedWhileMoving_ とは別に Motion_IsTurning() で見る）
  int stopCount_ = 0;            // 起動からの通算の停止回数（完了条件の確認用）
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
