#include "steer_avoid.h"

#include "../core/arbiter.h"
#include "../core/debug_pause.h"
#include "../core/gyro.h"
#include "../core/motion.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../core/test_stats.h"
#include "../core/tof_view.h"
#include "../hal/hal_log.h"

static const char *SideName(int side) {
  return side == TOF_EDGE_LEFT ? "左" : "右";
}

// 新しい枚があれば、直近の枚の記録へ足す
void SteerAvoidBehavior::takeView() {
  TofEdgeView view;
  if (!TofView_TakeEdgeView(&viewSeq_, &view)) {
    return;
  }
  for (int i = STEER_HISTORY - 1; i > 0; i--) {
    historyMs_[i] = historyMs_[i - 1];
    historyMm_[i][TOF_EDGE_LEFT] = historyMm_[i - 1][TOF_EDGE_LEFT];
    historyMm_[i][TOF_EDGE_RIGHT] = historyMm_[i - 1][TOF_EDGE_RIGHT];
  }
  historyMs_[0] = view.readyMs;
  historyMm_[0][TOF_EDGE_LEFT] = view.nearMm[TOF_EDGE_LEFT];
  historyMm_[0][TOF_EDGE_RIGHT] = view.nearMm[TOF_EDGE_RIGHT];
  oppositeMm_[TOF_EDGE_LEFT] = view.anyMm[TOF_EDGE_LEFT];
  oppositeMm_[TOF_EDGE_RIGHT] = view.anyMm[TOF_EDGE_RIGHT];
  centerMm_ = view.centerMm;
  if (historyCount_ < STEER_HISTORY) {
    historyCount_++;
  }
  newView_ = true;
}

// その側の物が、よけるきっかけを満たすか
bool SteerAvoidBehavior::sideTriggers(int side, unsigned long nowMs) const {
  (void)nowMs;
  if (historyCount_ < STEER_HISTORY) {
    return false;
  }
  // 止まる距離より遠く、ある距離より近い物が、続けて写っている
  for (int i = 0; i < STEER_NEAR_FRAMES; i++) {
    int mm = historyMm_[i][side];
    if (mm <= TOF_STOP_MM || mm >= STEER_TRIGGER_MM) {
      return false;
    }
  }
  // 縮んでいる（平行な壁と、1枚だけ飛んだ値で曲がらない）。枚が抜けていたら、時間が合わないので見ない
  int nowMm = historyMm_[0][side];
  int oldMm = historyMm_[STEER_HISTORY - 1][side];
  long spanMs = (long)(historyMs_[0] - historyMs_[STEER_HISTORY - 1]);
  if (oldMm < 0 || oldMm - nowMm < STEER_SHRINK_MM ||
      spanMs < STEER_SHRINK_WINDOW_MS - STEER_SHRINK_SLACK_MS || spanMs > STEER_SHRINK_WINDOW_MS + STEER_SHRINK_SLACK_MS) {
    return false;
  }
  // 中央の列より近いときだけ（正面の壁は今までどおり止まる）
  if (centerMm_ >= 0 && nowMm + STEER_EDGE_LEAD_MM > centerMm_) {
    return false;
  }
  // 反対側の横に余裕があるときだけ。写らないときは、余裕ありとしない
  int opposite = oppositeMm_[side == TOF_EDGE_LEFT ? TOF_EDGE_RIGHT : TOF_EDGE_LEFT];
  return opposite > STEER_OPPOSITE_CLEAR_MM;
}

// 曲がり終えたあとを見る。止まったら数え、止まらずに時間がたったら「よけきれた」
void SteerAvoidBehavior::watchOutcome(unsigned long nowMs) {
  if (!watching_) {
    return;
  }
  if (Pause_IsPaused() || Safety_IsLifted()) {
    watching_ = false;   // 試験の外の止まり方は数えない
    clearPending_ = false;
    return;
  }
  if (Obstacle_IsBlocked() && Motion_GetSpeed() >= MOTOR_SPEED_EPSILON) {
    unsigned long sinceEnd = nowMs - lastEndMs_;
    TestStats_RecordSteerNextStop(sinceEnd, clearPending_);
    Log_Printf("よける", "曲がり終えてから %lums で止まりました", sinceEnd);
    watching_ = false;
    clearPending_ = false;
    return;
  }
  if (clearPending_ && nowMs - lastEndMs_ >= STEER_CLEARED_MS) {
    clearPending_ = false;
    TestStats_RecordSteerCleared();
    Log_Printf("よける", "よけきれました（曲がり終えてから %dms 止まらなかった）", STEER_CLEARED_MS);
  }
}

// うろうろが歩いている間に、きっかけを見る。枚が届いたときだけ
void SteerAvoidBehavior::checkTrigger(unsigned long nowMs) {
  if (!newView_) {
    return;
  }
  newView_ = false;
  if (wander_->isAccelerating() || wander_->isCruising()) {
    if (!Gyro_IsTurnUsable() || Obstacle_IsBlocked() || !Obstacle_IsReady() || Motion_IsTurning()) {
      return;
    }
    if (endedOnce_ && nowMs - lastEndMs_ < STEER_COOLDOWN_MS) {
      return;
    }
    bool left = sideTriggers(TOF_EDGE_LEFT, nowMs);
    bool right = sideTriggers(TOF_EDGE_RIGHT, nowMs);
    if (!left && !right) {
      return;
    }
    if (wander_->isAccelerating()) {
      // 今は巡航だけで曲がる。加速中に満たした回は数えておく（機会がどれだけ減っているかを見る）
      if (!accelMissCounted_) {
        accelMissCounted_ = true;
        TestStats_RecordSteerAccelMiss();
      }
      return;
    }
    armedSide_ = (left && right)
                     ? (historyMm_[0][TOF_EDGE_LEFT] <= historyMm_[0][TOF_EDGE_RIGHT] ? TOF_EDGE_LEFT : TOF_EDGE_RIGHT)
                     : (left ? TOF_EDGE_LEFT : TOF_EDGE_RIGHT);
    armed_ = true;
  }
}

int SteerAvoidBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  takeView();
  watchOutcome(nowMs);
  if (Pause_IsPaused() || Safety_IsLifted()) {
    armed_ = false;
    historyCount_ = 0;
    return 0;
  }
  if (arcing_) {
    return PRIORITY_STEER_AVOID;
  }
  // うろうろが動いているときだけ見る（交代中のうろうろの状態は古い）
  if (Arbiter_ActiveName(LAYER_BODY) != wander_->name() || !(wander_->isAccelerating() || wander_->isCruising())) {
    armed_ = false;
    historyCount_ = 0;
    accelMissCounted_ = false;
    return 0;
  }
  if (!armed_) {
    checkTrigger(nowMs);
  }
  return armed_ ? PRIORITY_STEER_AVOID : 0;
}

void SteerAvoidBehavior::onStart(unsigned long nowMs) {
  armed_ = false;
  arcing_ = true;
  arcStartMs_ = nowMs;
  arcStartYaw_ = Gyro_YawDeg();
  firstSeenMm_ = historyMm_[0][armedSide_];
  lastSeenMm_ = firstSeenMm_;
  // 前回の結果がまだ決まっていなければ、続けて曲がった回として数える
  if (clearPending_) {
    TestStats_RecordSteerChained();
  }
  watching_ = false;
  clearPending_ = false;
  // よける物が左なら右へ曲がる。右へ曲がるときの内側は右の車輪
  bool turnRight = armedSide_ == TOF_EDGE_LEFT;
  Motion_StartArc(turnRight ? STEER_OUTER_PWM : STEER_INNER_PWM, turnRight ? STEER_INNER_PWM : STEER_OUTER_PWM, nowMs);
  TestStats_RecordSteerStart(!turnRight);
  Log_Printf("よける", "%sの端の列 %dmm（中央 %dmm・反対側 %dmm）。%sへ進みながら曲がります",
             SideName(armedSide_), firstSeenMm_, centerMm_,
             oppositeMm_[armedSide_ == TOF_EDGE_LEFT ? TOF_EDGE_RIGHT : TOF_EDGE_LEFT], turnRight ? "右" : "左");
}

// 普通に曲がり終えた。止まらずに巡航へ戻す
void SteerAvoidBehavior::finishArc(const char *reason, unsigned long nowMs) {
  float angle = (Gyro_YawDeg() - arcStartYaw_) * (armedSide_ == TOF_EDGE_LEFT ? -1.0f : 1.0f);
  arcing_ = false;
  endedOnce_ = true;
  lastEndMs_ = nowMs;
  watching_ = true;
  clearPending_ = true;
  historyCount_ = 0;   // 向きが変わったので、縮みは見直す
  Motion_SetSpeed(CRUISE_SPEED, 0, nowMs);
  wander_->requestCruiseResume();
  Log_Printf("よける", "曲がり終えました（%s。%lums で %.1f°。最初 %dmm・最後に写った %dmm）",
             reason, nowMs - arcStartMs_, angle, firstSeenMm_, lastSeenMm_);
}

void SteerAvoidBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (!arcing_) {
    return;
  }
  if (!Motion_IsArcing()) {
    return;   // 安全層が止めた。優先度の見直しで交代する（onStop で数える）
  }
  // 視野の外へ出たことは「よけきれた」の根拠にしない。記録として最後に写った距離だけ残す
  if (historyMm_[0][armedSide_] >= 0) {
    lastSeenMm_ = historyMm_[0][armedSide_];
  }
  float angle = (Gyro_YawDeg() - arcStartYaw_) * (armedSide_ == TOF_EDGE_LEFT ? -1.0f : 1.0f);
  if (angle >= STEER_MAX_DEG) {
    finishArc("角度の上限", nowMs);
  } else if (nowMs - arcStartMs_ >= STEER_MAX_MS) {
    finishArc("時間の上限", nowMs);
  } else if (!Gyro_IsTurnUsable()) {
    finishArc("ジャイロが使えない", nowMs);
  }
}

void SteerAvoidBehavior::onStop(unsigned long nowMs) {
  if (!arcing_) {
    return;
  }
  // 曲がっている途中で交代した（止まる判断・持ち上げ・一時停止など）。次の振る舞いに任せる
  arcing_ = false;
  endedOnce_ = true;
  lastEndMs_ = nowMs;
  historyCount_ = 0;
  TestStats_RecordSteerInterrupted();
  Log_Printf("よける", "曲がっている途中で交代しました（%lums。%s）", nowMs - arcStartMs_,
             Obstacle_IsBlocked() ? "障害物あり" : "ほかの理由");
  Motion_Stop(nowMs);
}
