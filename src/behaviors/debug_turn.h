// デバッグ：回転角の測定（#0。docs/specs/25_wander.md「回転角を測るためのデバッグキー」）
// シリアルの 3〜6 で、その場回転・片側旋回を1ステップだけ実行する。
// 1ステップで何度回るかを実測して、WANDER_AVOID_PIVOT_MS と TROUBLE_MAX_STEPS を詰めるための道具。
// 1ステップの間の電池電圧の最低値も測る（大電流による実際の電圧降下かを見るため）。
//
// 車体レイヤーのいちばん高い優先度（PRIORITY_DEBUG_TURN）で、受け付けたときだけ発動する。
// 受け付けてよいかの判断（停止中か・ID15 が動いていないか・持ち上げられていないか）は
// キーを読む側（main.cpp）が行う。
#ifndef NOVA_BEHAVIORS_DEBUG_TURN_H
#define NOVA_BEHAVIORS_DEBUG_TURN_H

#include "../core/behavior.h"
#include "../core/motion.h"

class DebugTurnBehavior : public Behavior {
 public:
  const char *name() const override { return "デバッグ回転"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 1ステップ分を予約する。次の調停で実行される
  void request(TurnKind kind, unsigned long durationMs);

  // 予約済み、または実行中か。この間の新しいキーは受け付けない
  bool isBusy() const { return requested_ || running_; }

 private:
  void begin(unsigned long nowMs);
  void sampleBattery(unsigned long nowMs);
  void reportBattery();

  bool requested_ = false;
  bool running_ = false;
  TurnKind kind_ = TURN_ROTATE_LEFT;
  unsigned long durationMs_ = 0;
  unsigned long startMs_ = 0;

  // 1ステップの間の電池電圧（ADCを毎ループ読む。状態行の500ms間隔では、短いステップの底を逃すため）
  float batteryIdleV_ = 0.0f;     // 動き出す前
  float batteryMinV_ = 0.0f;      // ステップ中の最低
  unsigned long batteryMinAtMs_ = 0;   // その時刻（動き出しからの経過）
  int batterySamples_ = 0;
};

#endif // NOVA_BEHAVIORS_DEBUG_TURN_H
