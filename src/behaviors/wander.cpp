#include "wander.h"

#include "../config.h"
#include "../core/debug_pause.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../hal/hal_log.h"

const char *WanderBehavior::StateName(State state) {
  switch (state) {
    case STATE_SETTLE:     return "ため";
    case STATE_SCAN_FRONT: return "正面を測る";
    case STATE_SCAN_LEFT:  return "左を測る";
    case STATE_SCAN_RIGHT: return "右を測る";
    case STATE_FACE_FRONT: return "正面へ戻す";
    case STATE_AVOID:      return "片側旋回";
    case STATE_READY:      return "歩き出す準備";
    case STATE_ACCEL:      return "加速";
    case STATE_CRUISE:     return "巡航";
    default:               return "減速";
  }
}

void WanderBehavior::ChangeState(State next, unsigned long nowMs) {
  Log_Printf("うろうろ", "%s→%s", StateName(state_), StateName(next));
  state_ = next;
  stateStartMs_ = nowMs;
}

int WanderBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  if (Pause_IsPaused()) {
    return 0;               // デバッグの一時停止中は動かない
  }
  return PRIORITY_WANDER;   // 既定の振る舞い。ほかに何もなければ常にこれ
}

// 最初（ため）からやり直す。持ち上げから戻ったときもここを通る
void WanderBehavior::Restart(unsigned long nowMs) {
  pendingPivot_ = false;
  Motion_Stop(nowMs);
  ChangeState(STATE_SETTLE, nowMs);
}

void WanderBehavior::onStart(unsigned long nowMs) {
  Restart(nowMs);
}

void WanderBehavior::onStop(unsigned long nowMs) {
  Motion_Stop(nowMs);            // 回転中ならここで取り消される
  Neck_Release(NECK_OWNER_RANGE);
}

// 見回しの結果から、歩き出す前に向きを変えるかを決める
void WanderBehavior::Decide(unsigned long nowMs) {
  (void)nowMs;
  pendingPivot_ = false;

  Log_Printf("うろうろ", "見回し 正面 %.1fcm／左 %.1fcm／右 %.1fcm", frontCm_, leftCm_, rightCm_);

  if (leftCm_ < WANDER_SIDE_NEAR_CM && rightCm_ > leftCm_) {
    pendingPivot_ = true;
    pivotKind_ = TURN_PIVOT_RIGHT;
    Log_Printf("うろうろ", "左が近い（%.1fcm < %.0fcm）ので右へ片側旋回 %lums",
               leftCm_, WANDER_SIDE_NEAR_CM, (unsigned long)WANDER_AVOID_PIVOT_MS);
  } else if (rightCm_ < WANDER_SIDE_NEAR_CM && leftCm_ > rightCm_) {
    pendingPivot_ = true;
    pivotKind_ = TURN_PIVOT_LEFT;
    Log_Printf("うろうろ", "右が近い（%.1fcm < %.0fcm）ので左へ片側旋回 %lums",
               rightCm_, WANDER_SIDE_NEAR_CM, (unsigned long)WANDER_AVOID_PIVOT_MS);
  } else if (leftCm_ < WANDER_SIDE_NEAR_CM && rightCm_ < WANDER_SIDE_NEAR_CM) {
    Log_Printf("うろうろ", "左右とも近い（左%.1fcm 右%.1fcm）。避ける先がないのでそのまま前進",
               leftCm_, rightCm_);
  } else {
    Log_Printf("うろうろ", "左右とも空いている（左%.1fcm 右%.1fcm）のでそのまま前進", leftCm_, rightCm_);
  }
}

void WanderBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  // 持ち上げられている間は状態を進めない。床に戻ったら「ため」からやり直す
  if (Safety_IsLifted()) {
    if (!lifted_) {
      lifted_ = true;
      Log_Printf("うろうろ", "持ち上げられました。床に戻ったら見回しからやり直します");
    }
    return;
  }
  if (lifted_) {
    lifted_ = false;
    Restart(nowMs);
    return;
  }

  switch (state_) {
    case STATE_SETTLE:
      if (nowMs - stateStartMs_ < WANDER_REST_MS) {
        return;
      }
      scan_.begin(SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      ChangeState(STATE_SCAN_FRONT, nowMs);
      return;

    case STATE_SCAN_FRONT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      frontCm_ = scan_.cm();
      scan_.begin(SERVO1_FRONT_DEG - WANDER_SCAN_PAN_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      ChangeState(STATE_SCAN_LEFT, nowMs);
      return;

    case STATE_SCAN_LEFT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      leftCm_ = scan_.cm();
      scan_.begin(SERVO1_FRONT_DEG + WANDER_SCAN_PAN_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      ChangeState(STATE_SCAN_RIGHT, nowMs);
      return;

    case STATE_SCAN_RIGHT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      rightCm_ = scan_.cm();
      Decide(nowMs);
      ChangeState(STATE_FACE_FRONT, nowMs);
      return;

    case STATE_FACE_FRONT:
      // 首を正面・水平に戻してから動き出す（設計原則2）
      if (!Neck_Request(NECK_OWNER_RANGE, SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, nowMs)) {
        return;
      }
      if (!Neck_IsFront() || !Neck_IsSteady(nowMs)) {
        return;
      }
      Neck_Release(NECK_OWNER_RANGE);
      if (pendingPivot_) {
        Motion_StartTurn(pivotKind_, nowMs);
        ChangeState(STATE_AVOID, nowMs);
      } else {
        ChangeState(STATE_READY, nowMs);
      }
      return;

    case STATE_AVOID:
      if (nowMs - stateStartMs_ < WANDER_AVOID_PIVOT_MS) {
        return;
      }
      Motion_StopTurn(nowMs);
      pendingPivot_ = false;
      ChangeState(STATE_READY, nowMs);
      return;

    case STATE_READY:
      // 首が正面で安定し、測距の履歴がたまり、正面が空いてから歩き出す
      if (Obstacle_IsBlocked() || !Obstacle_IsReady()) {
        return;   // 塞がっていれば ID9 が引き継ぐので、ここで待つ
      }
      if (!Neck_IsFront() || !Neck_IsSteady(nowMs)) {
        return;
      }
      runMs_ = (unsigned long)random(WANDER_RUN_MIN_MS, WANDER_RUN_MAX_MS + 1);
      Log_Printf("うろうろ", "前進 %lums（%d〜%dms から）",
                 runMs_, WANDER_RUN_MIN_MS, WANDER_RUN_MAX_MS);
      Motion_SetSpeed(CRUISE_SPEED, MOTION_ACCEL_MS, nowMs);
      ChangeState(STATE_ACCEL, nowMs);
      return;

    case STATE_ACCEL:
      if (Motion_IsAtTarget()) {
        ChangeState(STATE_CRUISE, nowMs);
      }
      return;

    case STATE_CRUISE:
      if (nowMs - stateStartMs_ >= runMs_) {
        Motion_Stop(nowMs);
        ChangeState(STATE_DECEL, nowMs);
      }
      return;

    case STATE_DECEL:
      if (Motion_IsAtTarget()) {
        Log_Printf("うろうろ", "止まりました。%lums ためてから見回します", (unsigned long)WANDER_REST_MS);
        ChangeState(STATE_SETTLE, nowMs);
      }
      return;
  }
}
