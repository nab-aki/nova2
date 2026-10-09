// デバッグ：モーターの振動がジャイロに乗るかの測定（#0。n キー。docs/specs/common_gyro_checklist.md）
// 車輪を浮かせた状態で、「停止 → 前進（巡航と同じ PWM）→ その場回転（回転と同じ PWM）」を
// それぞれ GYRO_SPIN_SEGMENT_MS ずつ行い、3区間のノイズを並べて出す。
// ジャイロの取り付け位置が左前（M1 モーターの近く）で、振動を拾いやすいかを確かめるための道具。
//
// 安全：
//   ・一時停止中で、ライントラッキングが 111（浮いている）のときだけ受け付ける（床の上では動かない）
//   ・測定の間だけ、安全層の「持ち上げたら止める」を外す（Safety_SetSpinTestActive）。終了・中断で必ず元に戻す
//   ・超音波の非常停止（OBSTACLE_EMERGENCY_CM 未満）は外さない。その場回転の区間でも、近ければ中断する
//   ・途中でライントラッキングが 111 でなくなったら即停止。どのキーでも中断できる
//   ・止まった理由（時間切れ／非常停止／ライン変化／キー ほか）をログに出す
#ifndef NOVA_BEHAVIORS_DEBUG_GYRO_SPIN_H
#define NOVA_BEHAVIORS_DEBUG_GYRO_SPIN_H

#include "../core/behavior.h"
#include "../core/gyro_measure.h"

class DebugGyroSpinBehavior : public Behavior {
 public:
  const char *name() const override { return "ジャイロ振動測定"; }
  BehaviorLayer layer() const override { return LAYER_BODY; }
  int priority(const SensorData &sensors, unsigned long nowMs) override;
  void onStart(unsigned long nowMs) override;
  void onUpdate(const SensorData &sensors, unsigned long nowMs) override;
  void onStop(unsigned long nowMs) override;

  // 測定を予約する。次の調停で始まる（受け付けてよいかは main.cpp が見る）
  void request();

  // 予約済み、または実行中か
  bool isBusy() const { return requested_ || phase_ != PHASE_IDLE; }

  // 中断する（キー・リモコン）。モーターをすぐ止め、持ち上げ停止を元に戻す
  void abort(const char *reason, unsigned long nowMs);

 private:
  enum Phase {
    PHASE_IDLE,
    PHASE_STILL,      // モーター停止
    PHASE_FORWARD,    // 前進（巡航と同じ PWM）
    PHASE_GAP,        // 前進を止めて、その場回転までの「ため」
    PHASE_ROTATE      // その場回転（回転と同じ PWM）
  };

  void begin(unsigned long nowMs);
  void enter(Phase phase, unsigned long nowMs);
  void finish(const char *reason, bool emergencyStop, unsigned long nowMs);
  void report();

  bool requested_ = false;
  Phase phase_ = PHASE_IDLE;
  unsigned long phaseStartMs_ = 0;
  bool collecting_ = false;        // いまの区間の集計を始めたか

  GyroStats still_;
  GyroStats forward_;
  GyroStats rotate_;
  char forwardTitle_[32] = "";     // 区間の名前（実際の PWM を入れる）
  char rotateTitle_[40] = "";
  bool stillDone_ = false;
  bool forwardDone_ = false;
  bool rotateDone_ = false;
};

#endif // NOVA_BEHAVIORS_DEBUG_GYRO_SPIN_H
