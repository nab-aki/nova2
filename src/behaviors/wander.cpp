#include "wander.h"

#include "../config.h"
#include "../core/debug_pause.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../core/test_stats.h"
#include "../core/turn_tuning.h"
#include "../hal/hal_log.h"

const char *WanderBehavior::StateName(State state) {
  switch (state) {
    case STATE_SETTLE:        return "ため";
    case STATE_SCAN_FRONT:    return "正面を測る";
    case STATE_SCAN_LEFT:     return "左を測る";
    case STATE_SCAN_RIGHT:    return "右を測る";
    case STATE_FACE_FRONT:    return "正面へ戻す";
    case STATE_AVOID:         return "片側旋回";
    case STATE_RECOVER_BACK:  return "後退";
    case STATE_RECOVER_TURN:  return "その場回転";
    case STATE_READY:         return "歩き出す準備";
    case STATE_ACCEL:         return "加速";
    case STATE_CRUISE:        return "巡航";
    default:                  return "減速";
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
  if (state_ == STATE_RECOVER_BACK || state_ == STATE_RECOVER_TURN) {
    // 横がとても近いときの後退+その場回転は、ID15 の立て直しと同じく
    // 気づく（ID9、優先度30）に割り込まれず最後までやり切る（非常停止と持ち上げは安全層が別途扱う）
    return PRIORITY_WANDER_RECOVER;
  }
  return PRIORITY_WANDER;   // 既定の振る舞い。ほかに何もなければ常にこれ
}

// 最初（ため）からやり直す。持ち上げから戻ったときもここを通る
void WanderBehavior::Restart(unsigned long nowMs) {
  pendingPivot_ = false;
  pendingRecover_ = false;
  Motion_Stop(nowMs);
  ChangeState(STATE_SETTLE, nowMs);
}

void WanderBehavior::onStart(unsigned long nowMs) {
  Restart(nowMs);
}

void WanderBehavior::onStop(unsigned long nowMs) {
  // 片側旋回の途中で手放されたら、理由を1行残す。旋回中に障害物「あり」に変わると、
  // ID9（優先度30）が割り込み、安全層も旋回を止める。そのあとは ID9・ID15 が引き継ぐ
  if (state_ == STATE_AVOID) {
    Log_Printf("うろうろ", "片側旋回の途中で交代しました（%lums で中断、%s。距離 %.1fcm）。向きは変わりきっていません",
               nowMs - stateStartMs_,
               Obstacle_IsBlocked() ? "障害物ありのため ID9 に" : "ほかの振る舞い（デバッグ回転・一時停止など）に",
               Obstacle_LastCm());
    TestStats_RecordPivotInterrupted();
  } else if (state_ == STATE_RECOVER_BACK || state_ == STATE_RECOVER_TURN) {
    // 優先度 PRIORITY_WANDER_RECOVER（35）で気づく（30）には割り込まれないので、
    // ここに来るのは一時停止など、優先度が0になる場合だけのはず
    Log_Printf("うろうろ", "後退+その場回転の途中で交代しました（%lums で中断、%s）。向きは変わりきっていません",
               nowMs - stateStartMs_, Pause_IsPaused() ? "一時停止のため" : "ほかの振る舞いに");
    TestStats_RecordRecoverInterrupted();
  }
  Motion_Stop(nowMs);            // 回転中ならここで取り消される
  Neck_Release(NECK_OWNER_RANGE);
}

// 見回しの結果から、歩き出す前に向きを変えるかを決める
void WanderBehavior::Decide(unsigned long nowMs) {
  (void)nowMs;
  pendingPivot_ = false;
  pendingRecover_ = false;

  Log_Printf("うろうろ", "見回し 正面 %.1fcm／左 %.1fcm／右 %.1fcm", frontCm_, leftCm_, rightCm_);

  bool leftVeryNear = leftCm_ < WANDER_SIDE_VERY_NEAR_CM;
  bool rightVeryNear = rightCm_ < WANDER_SIDE_VERY_NEAR_CM;

  if (leftVeryNear || rightVeryNear) {
    // 片側旋回は車体が前へふくらむので、とても近いと安全層に止められ続けて張りつく（2026-09-22 実測）。
    // より近い側と反対へ、後退してからその場回転で離れる（どちらも安全層に止められない）
    bool nearIsLeft = leftVeryNear && (!rightVeryNear || leftCm_ <= rightCm_);
    pendingRecover_ = true;
    recoverKind_ = nearIsLeft ? TURN_ROTATE_RIGHT : TURN_ROTATE_LEFT;
    Log_Printf("うろうろ", "%sがとても近い（%.1fcm < %.0fcm）ので、後退してから%sへその場回転",
               nearIsLeft ? "左" : "右", nearIsLeft ? leftCm_ : rightCm_, WANDER_SIDE_VERY_NEAR_CM,
               nearIsLeft ? "右" : "左");
  } else if (leftCm_ < WANDER_SIDE_NEAR_CM && rightCm_ > leftCm_) {
    pendingPivot_ = true;
    pivotKind_ = TURN_PIVOT_RIGHT;
    Log_Printf("うろうろ", "左が近い（%.1fcm < %.0fcm）ので右へ片側旋回 %lums",
               leftCm_, WANDER_SIDE_NEAR_CM, (unsigned long)TurnTuning_StepMs(TURN_PIVOT_RIGHT));
  } else if (rightCm_ < WANDER_SIDE_NEAR_CM && leftCm_ > rightCm_) {
    pendingPivot_ = true;
    pivotKind_ = TURN_PIVOT_LEFT;
    Log_Printf("うろうろ", "右が近い（%.1fcm < %.0fcm）ので左へ片側旋回 %lums",
               rightCm_, WANDER_SIDE_NEAR_CM, (unsigned long)TurnTuning_StepMs(TURN_PIVOT_LEFT));
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
      if (pendingRecover_) {
        recoverBackStopping_ = false;
        TestStats_RecordRecoverPerformed();
        Motion_SetSpeed(TROUBLE_BACK_SPEED, TROUBLE_BACK_RAMP_MS, nowMs);
        ChangeState(STATE_RECOVER_BACK, nowMs);
      } else if (pendingPivot_) {
        TestStats_RecordPivotPerformed();
        Motion_StartTurn(pivotKind_, nowMs);
        ChangeState(STATE_AVOID, nowMs);
      } else {
        ChangeState(STATE_READY, nowMs);
      }
      return;

    case STATE_AVOID:
      // 安全層に旋回を止められた（非常停止距離未満など）。時間を待たずに旋回を終えて次へ進む。
      // まだ「あり」なら、次のループで ID9 が引き継ぐ
      if (!Motion_IsTurning()) {
        Log_Printf("うろうろ", "片側旋回が安全層に止められました（%lums で中断、距離 %.1fcm）。向きは変わりきっていません",
                   nowMs - stateStartMs_, Obstacle_LastCm());
        TestStats_RecordPivotInterrupted();
        pendingPivot_ = false;
        ChangeState(STATE_READY, nowMs);
        return;
      }
      if (nowMs - stateStartMs_ < (unsigned long)TurnTuning_StepMs(pivotKind_)) {
        return;
      }
      Motion_StopTurn(nowMs);
      pendingPivot_ = false;
      ChangeState(STATE_READY, nowMs);
      return;

    case STATE_RECOVER_BACK:
      // trouble.cpp の STATE_BACK と同じ2段階（後退→減速→静止を待つ）。値も ID15 と共通のものを使う
      if (!recoverBackStopping_) {
        if (nowMs - stateStartMs_ < TROUBLE_BACK_MS) {
          return;
        }
        recoverBackStopping_ = true;
        Motion_SetSpeed(0.0f, TROUBLE_BACK_RAMP_MS, nowMs);
        return;
      }
      if (!Motion_IsAtTarget()) {
        return;
      }
      Log_Printf("うろうろ", "後退 %lums：%sへその場回転します",
                 (unsigned long)TROUBLE_BACK_MS, Motion_TurnName(recoverKind_));
      Motion_StartTurn(recoverKind_, nowMs);
      ChangeState(STATE_RECOVER_TURN, nowMs);
      return;

    case STATE_RECOVER_TURN:
      // 1ステップ（その場回転。約30°）で終える。ID15 のような再判定ループはしない
      // （このあと STATE_READY が障害物の有無を確かめてから歩き出す）
      if (nowMs - stateStartMs_ < (unsigned long)TurnTuning_StepMs(recoverKind_)) {
        return;
      }
      Motion_StopTurn(nowMs);
      pendingRecover_ = false;
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
