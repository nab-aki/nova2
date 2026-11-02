// デバッグ：曲がり測定（ID27 段階1「曲がれる力を測る」。#27）
// 内側の車輪を ARC_INNER_PWM、外側を選んだ値にして前進し、ジャイロで曲がる速さ（°/秒）を測る。
// 合否の線はまだ決めていない（40°/秒などは【仮】の参考値。根拠は未確認。catalog.md「ID27 のメモ」）。
//
// 使い方（ケーブルなし。ケーブルが引っぱられると測定に混ざる）：
//   ・一時停止中（p）に、i＝左へ曲がる・k＝右へ曲がる・l＝外側の値を次へ、で準備する（シリアルに今の設定を出す）。
//   ・リモコンの ▶ で測定を始める。準備した測定があるときだけ、▶ は測定の開始に使う（なければ今までどおり再開・一時停止）。
//   ・測定中は、どのキーかリモコンの ▶ で中断できる。
//   ・結果は直近 ARC_RESULT_COUNT 回を RAM に残す（電源を切る・リセットで消える）。ケーブルをつないでから t で見る。
//
// 1回の流れ：首を走行姿勢にして待つ → 前が空いているか確かめる → 直進（ARC_LEAD_MS。加速 ARC_LEAD_ACCEL_MS）→ 差動の前進（ARC_RUN_MS）→ 減速して停止。
// 結果：平均の曲がる速さ・0.1秒ごと・差動の前進の 0.3/0.5/1.0 秒の時点の角度・前に進む速さ（ToF 中央の列の距離の減り方。壁は先 1.5m 以上）。
// 安全：ToF で止める判断と、持ち上げで止める安全層は働いたまま（外さない）。止まった回は「途中で止まった」と出す。
#ifndef NOVA_BEHAVIORS_DEBUG_ARC_H
#define NOVA_BEHAVIORS_DEBUG_ARC_H

#include "../config.h"
#include "../core/behavior.h"

class DebugArcBehavior : public Behavior {
 public:
  const char *name() const override { return "曲がり測定"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 準備（i・k・l）。設定をシリアルに出す。測定中は呼ばない
  void selectLeft();
  void selectRight();
  void nextOuter();
  void nextInner();   // 内側：675 → 0（フリー）→ ブレーキ → 最初に戻る（キー 9）
  bool isPrepared() const { return prepared_; }
  void cancelPrepare() { prepared_ = false; }

  // 測定を予約する（▶）。次の調停で始まる。準備した測定のときだけ呼ぶ
  void request();
  bool isBusy() const { return requested_ || phase_ != PHASE_IDLE; }

  // 中断する（キー・リモコン）。車体をすぐ止める
  void abort(const char *reason, unsigned long nowMs);

  // t キー：直近の結果を新しい順に出す
  static void PrintResults();
  static void OuterListText(char *buffer, size_t size);   // 「基準→800→…」

 private:
  enum Phase { PHASE_IDLE, PHASE_PREP, PHASE_LEAD, PHASE_ARC, PHASE_STOPPING };
  enum End { END_COMPLETE, END_TOF, END_OTHER, END_ABORT };

  void printSetting(const char *prefix) const;
  void begin(const SensorData &sensors, unsigned long nowMs);
  void startArc(unsigned long nowMs);
  void finishRecord(End end, const SensorData &sensors, unsigned long nowMs);
  void finish(const char *reason);
  void sampleSpeed();

  bool prepared_ = false;
  bool requested_ = false;
  int dir_ = 1;            // 1＝左へ曲がる、-1＝右へ曲がる
  int innerIndex_ = 0;     // ARC_INNER_LIST の番号（起動時は 675）
  int outerIndex_ = 0;     // 0＝直進の基準、1〜＝外側の値の一覧の番号（1 始まり）
  Phase phase_ = PHASE_IDLE;
  unsigned long phaseStartMs_ = 0;   // 現在の区間の始まり
  unsigned long prepStartMs_ = 0;
  unsigned long arcStartMs_ = 0;

  // この回の設定（始めた時点の値を固定する）
  int runDir_ = 1;
  int runOuterIndex_ = 0;
  int runInnerPwm_ = 0;
  int runLeftPwm_ = 0;
  int runRightPwm_ = 0;
  float batteryStartV_ = 0.0f;

  // 差動の前進の間の記録。index 0 が始まり、k は始まりから k×ARC_STEP_MS 以降の最初の loop
  unsigned long stepMs_[ARC_MAX_STEPS];
  float stepYaw_[ARC_MAX_STEPS];
  int stepCount_ = 0;
  unsigned long lastActiveMs_ = 0;   // 差動の前進がまだ続いていた最後の loop
  float lastActiveYaw_ = 0.0f;

  // 前に進む速さ：ToF 中央の列の距離が、1枚前から縮んだ分 ÷ 時間。時刻は2枚の読み終えた時刻の真ん中
  uint32_t speedSeq_ = 0;
  bool speedPrevValid_ = false;
  uint32_t speedPrevMs_ = 0;
  int speedPrevMm_ = 0;
  int speedCount_ = 0;
  unsigned long speedMs_[ARC_SPEED_MAX];
  int16_t speedMmS_[ARC_SPEED_MAX];
};

#endif // NOVA_BEHAVIORS_DEBUG_ARC_H
