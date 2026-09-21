#include "trouble.h"

#include "../config.h"
#include "../core/debug_pause.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../core/turn_tuning.h"
#include "../hal/hal_log.h"

const char *TroubleBehavior::StateName(State state) {
  switch (state) {
    case STATE_BACK:       return "後退";
    case STATE_SCAN_LEFT:  return "左を測る";
    case STATE_SCAN_RIGHT: return "右を測る";
    case STATE_TURN:       return "回る";
    case STATE_CHECK:      return "正面を測り直す";
    default:               return "休む";
  }
}

void TroubleBehavior::ChangeState(State next, unsigned long nowMs) {
  Log_Printf("困る", "%s→%s", StateName(state_), StateName(next));
  state_ = next;
  stateStartMs_ = nowMs;
}

// ID9 の反応が終わっても塞がったままなら引き継ぐ。いちど始めたら、
// 空くかあきらめるまで手放さない（途中で ID25 に戻さない）
int TroubleBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (Pause_IsPaused()) {
    return 0;   // デバッグの一時停止中は動かない（動いていたら止まり、再開すると最初からやり直す）
  }
  if (active_) {
    return PRIORITY_TROUBLE;
  }
  if (Obstacle_IsBlocked() && notice_->reactionDone(nowMs)) {
    return PRIORITY_TROUBLE;
  }
  return 0;
}

// 最初（後退）から始める。持ち上げから戻ったときもここを通る
void TroubleBehavior::StartSequence(unsigned long nowMs) {
  steps_ = 0;
  backStopping_ = false;
  backStartCm_ = Obstacle_LastCm();
  state_ = STATE_BACK;
  stateStartMs_ = nowMs;
  Log_Printf("困る", "正面が塞がったまま（%.1fcm）。立て直します", backStartCm_);
  Motion_SetSpeed(TROUBLE_BACK_SPEED, TROUBLE_BACK_RAMP_MS, nowMs);
}

void TroubleBehavior::onStart(unsigned long nowMs) {
  active_ = true;
  StartSequence(nowMs);
}

void TroubleBehavior::onStop(unsigned long nowMs) {
  active_ = false;
  Motion_StopTurn(nowMs);
  Motion_Stop(nowMs);
  Neck_Release(NECK_OWNER_RANGE);
}

// 左右のうち遠いほうへ回る。差が小さいときは前回と反対側にする
// （行き止まりで同じ動きを繰り返さないため）
void TroubleBehavior::Decide(unsigned long nowMs) {
  (void)nowMs;
  bool turnLeft;
  if (fabsf(leftCm_ - rightCm_) < TROUBLE_SIDE_DIFF_CM) {
    turnLeft = !lastTurnLeft_;
    Log_Printf("困る", "左右の差が小さい（左%.1fcm 右%.1fcm、差%.1fcm）ので前回と反対の%sへ",
               leftCm_, rightCm_, fabsf(leftCm_ - rightCm_), turnLeft ? "左" : "右");
  } else {
    turnLeft = leftCm_ > rightCm_;
    Log_Printf("困る", "%sが遠い（%.1fcm > %.1fcm）ので%sへその場回転",
               turnLeft ? "左" : "右",
               turnLeft ? leftCm_ : rightCm_, turnLeft ? rightCm_ : leftCm_,
               turnLeft ? "左" : "右");
  }
  lastTurnLeft_ = turnLeft;
  turnKind_ = turnLeft ? TURN_ROTATE_LEFT : TURN_ROTATE_RIGHT;
}

void TroubleBehavior::StartTurnStep(unsigned long nowMs) {
  ChangeState(STATE_TURN, nowMs);
  Motion_StartTurn(turnKind_, nowMs);
}

void TroubleBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  // 持ち上げられている間は状態を進めない。床に戻ったら最初（後退）からやり直す
  if (Safety_IsLifted()) {
    if (!lifted_) {
      lifted_ = true;
      Log_Printf("困る", "持ち上げられました。床に戻ったら後退からやり直します");
    }
    return;
  }
  if (lifted_) {
    lifted_ = false;
    StartSequence(nowMs);
    return;
  }

  switch (state_) {
    case STATE_BACK:
      if (!backStopping_) {
        if (nowMs - stateStartMs_ < TROUBLE_BACK_MS) {
          return;
        }
        backStopping_ = true;
        Motion_SetSpeed(0.0f, TROUBLE_BACK_RAMP_MS, nowMs);
        return;
      }
      if (!Motion_IsAtTarget()) {
        return;
      }
      {
        float nowCm = Obstacle_LastCm();
        if (backStartCm_ >= 0.0f && nowCm >= 0.0f) {
          Log_Printf("困る", "後退 %lums：前 %.1fcm → %.1fcm（%.1fcm 下がった）",
                     (unsigned long)TROUBLE_BACK_MS, backStartCm_, nowCm, nowCm - backStartCm_);
        } else {
          Log_Printf("困る", "後退 %lums（前後の距離が測れませんでした）", (unsigned long)TROUBLE_BACK_MS);
        }
      }
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
      Log_Printf("困る", "左 %.1fcm／右 %.1fcm", leftCm_, rightCm_);
      Decide(nowMs);
      Neck_Release(NECK_OWNER_RANGE);   // 回る間は安全層が首を正面に固定する
      StartTurnStep(nowMs);
      return;

    case STATE_TURN:
      if (nowMs - stateStartMs_ < (unsigned long)TurnTuning_StepMs(turnKind_)) {
        return;
      }
      Motion_StopTurn(nowMs);
      steps_++;
      checkReset_ = false;
      ChangeState(STATE_CHECK, nowMs);
      return;

    case STATE_CHECK:
      // 車体の揺れが収まるまで待ってから、履歴を捨てて測り直す
      if (nowMs - stateStartMs_ < TROUBLE_CHECK_SETTLE_MS) {
        return;
      }
      if (!checkReset_) {
        checkReset_ = true;
        checkResetMs_ = nowMs;
        checkClosestCm_ = ULTRASONIC_MAX_CM;
        checkValidCount_ = 0;
        Obstacle_Reset();
        return;
      }
      // リセット後に完了した測距のうち、最も近い有効値を覚える
      if (sensors.distanceUpdated && sensors.distanceNeckSteady &&
          (long)(sensors.distanceMs - checkResetMs_) > 0 && sensors.distanceValid) {
        if (checkValidCount_ == 0 || sensors.distanceCm < checkClosestCm_) {
          checkClosestCm_ = sensors.distanceCm;
        }
        checkValidCount_++;
      }
      if (!Obstacle_IsReady()) {
        return;   // 5回たまるまで判定しない
      }
      // 「空いた」＝ 自分の基準（最も近い値が TROUBLE_CLEAR_CM 以上）と
      //   共通部品の判定（あり→なしに戻った）の両方（docs/specs/15_trouble.md）
      if (checkClosestCm_ >= TROUBLE_CLEAR_CM && !Obstacle_IsBlocked()) {
        Log_Printf("困る", "正面が空いた（%.1fcm、回転%d回）。うろうろに戻ります", checkClosestCm_, steps_);
        active_ = false;   // 優先度が0になり、調停が ID25 に戻す
        return;
      }
      Log_Printf("困る", "%d回目の回転のあと 正面 %.1fcm（まだ塞がっている）", steps_, checkClosestCm_);
      if (steps_ >= TROUBLE_MAX_STEPS) {
        Log_Printf("困る", "%d回まわっても空きません。%lums 休みます",
                   steps_, (unsigned long)TROUBLE_GIVEUP_REST_MS);
        ChangeState(STATE_REST, nowMs);
        return;
      }
      StartTurnStep(nowMs);
      return;

    case STATE_REST:
      if (nowMs - stateStartMs_ >= TROUBLE_GIVEUP_REST_MS) {
        StartSequence(nowMs);   // 後退からやり直す
      }
      return;
  }
}
