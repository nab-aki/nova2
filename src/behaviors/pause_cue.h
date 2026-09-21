// 一時停止・再開の合図（#0。docs/specs/common_ir.md）
// ケーブルなしで試験をするとき、切り替わったことを目で見て分かるようにする。
//   一時停止 ＝ 目を細める（PAUSE_CUE_PAUSE_MS）
//   再開     ＝ ゆっくり目を閉じて開く（細める→閉じる→細める。合計 約0.5秒。
//               普通のまばたき（約0.25秒）と区別できる。「見開く」は ID9 の「気づく」と紛らわしいので使わない）
// 目レイヤーの振る舞いで、まばたき（10）・気づく（見開く）（30）より優先する。
#ifndef NOVA_BEHAVIORS_PAUSE_CUE_H
#define NOVA_BEHAVIORS_PAUSE_CUE_H

#include "../core/behavior.h"

class PauseCueBehavior : public Behavior {
 public:
  const char *name() const override { return "合図（一時停止・再開）"; }
  BehaviorLayer layer() const override { return LAYER_EYES; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 合図を予約する（paused=true なら一時停止の合図、false なら再開の合図）。次の調停で始まる
  void trigger(bool paused);

 private:
  void begin(unsigned long nowMs);
  void finish(void);

  bool requested_ = false;
  bool running_ = false;
  bool paused_ = false;          // いま出している合図が一時停止か
  bool nextPaused_ = false;      // 予約された合図
  unsigned long startMs_ = 0;
};

#endif // NOVA_BEHAVIORS_PAUSE_CUE_H
