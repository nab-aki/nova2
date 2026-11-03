// ID27 進みながらよける（v1。段階2 の最小の形）。docs/specs/27_steer_avoid.md
// うろうろの巡航中、端の列に写った物を、止まらずに反対側へ少し曲がってよける。
// 内側はフリー（PWM 0）、外側は STEER_OUTER_PWM。差動は前進の一種なので、止まる判断と安全層はそのまま働く。
// よけきれなかったときは、通路の中の物で止まる今の判断に任せる（ID27 から止める道は作らない）。
//
// 優先度は うろうろ より上、気づく より下。普通に曲がり終えたら、うろうろは止まらずに巡航へ戻る。
#ifndef NOVA_BEHAVIORS_STEER_AVOID_H
#define NOVA_BEHAVIORS_STEER_AVOID_H

#include "../config.h"
#include "../core/behavior.h"
#include "wander.h"

class SteerAvoidBehavior : public Behavior {
 public:
  explicit SteerAvoidBehavior(WanderBehavior *wander) : wander_(wander) {}

  const char *name() const override { return "進みながらよける"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // t キー：曲がった回ごとの記録を新しい順に出す
  static void PrintRecords();

 private:
  void takeView();
  bool sideTriggers(int side, unsigned long nowMs) const;
  void watchOutcome(unsigned long nowMs);
  void checkTrigger(unsigned long nowMs);
  void finishArc(const char *reason, unsigned long nowMs);
  void endBySafety(unsigned long nowMs);

  WanderBehavior *wander_;

  bool armed_ = false;       // きっかけを満たした。次の交代で曲がり始める
  bool arcing_ = false;      // 曲がっている
  int armedSide_ = 0;        // よける物の側（TOF_EDGE_LEFT・TOF_EDGE_RIGHT）
  unsigned long arcStartMs_ = 0;
  float arcStartYaw_ = 0.0f;
  int firstSeenMm_ = -1;     // きっかけのときに写っていた距離
  int lastSeenMm_ = -1;      // 曲がっている間に、最後に写った距離

  // 直近の枚（新しい順）
  uint32_t viewSeq_ = 0;
  int historyCount_ = 0;
  uint32_t historyMs_[STEER_HISTORY];
  int16_t historyMm_[STEER_HISTORY][2];
  int16_t oppositeMm_[2] = {-1, -1};   // 最新の枚の、その側の端の列の最も近い物
  int16_t centerMm_ = -1;              // 最新の枚の、中央の列の最も近い物
  bool newView_ = false;
  int8_t nearCol_[2] = {-1, -1};       // 最新の枚の、historyMm_[0] の列
  bool partnerWaiting_ = false;        // 交代した相手を、次の調停で記録する

  bool endedOnce_ = false;
  unsigned long lastEndMs_ = 0;     // 前回曲がり終えた時刻
  bool watching_ = false;           // 曲がり終えてから、次に止まるまでを見ている
  bool clearPending_ = false;       // 「よけきれた」をまだ決めていない
  bool accelMissCounted_ = false;   // この加速の間に、見送りを数えた
};

#endif // NOVA_BEHAVIORS_STEER_AVOID_H
