#include "trouble.h"

#include "../config.h"
#include "../core/debug_pause.h"
#include "../core/gyro.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../core/test_stats.h"
#include "../core/trace.h"
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
  Trace_Mark(name(), StateName(next), nowMs);   // 足あと（再起動しても残る）
}

// ------------------------ 経過の記録（原因調べ用。動きは変えない）------------------------ //

void TroubleBehavior::LogBegin(unsigned long nowMs) {
  LogFinish(LOG_ABORTED);   // 前の記録が締まっていなければ、中断として締める
  logNow_ = &logs_[logHead_];
  logHead_ = (logHead_ + 1) % TROUBLE_LOG_COUNT;
  if (logCount_ < TROUBLE_LOG_COUNT) {
    logCount_++;
  }
  memset(logNow_, 0, sizeof(*logNow_));
  logNow_->startMs = nowMs;
  logNow_->end = LOG_RUNNING;
}

void TroubleBehavior::LogFinish(LogEnd end) {
  if (logNow_ != nullptr && logNow_->end == LOG_RUNNING) {
    logNow_->end = (uint8_t)end;   // 回転の回数は、測り直しのたびに書いてある
  }
}

void TroubleBehavior::printLog() const {
  if (logCount_ == 0) {
    Log_Printf("困る", "経過の記録：まだありません");
    return;
  }
  Log_Printf("困る", "経過の記録（直近%d回。新しい順。「空いた」＝最も近い値が %.0fcm 以上 かつ 共通部品が「なし」。○＝満たした ×＝満たさない）：",
             logCount_, TROUBLE_CLEAR_CM);
  for (int i = 0; i < logCount_; i++) {
    const SeqLog &q = logs_[(logHead_ - 1 - i + 2 * TROUBLE_LOG_COUNT) % TROUBLE_LOG_COUNT];
    const char *end = (q.end == LOG_CLEARED) ? "空いた" : (q.end == LOG_GIVEUP) ? "あきらめ"
                    : (q.end == LOG_ABORTED) ? "中断" : "進行中";
    if (q.scanned) {
      Log_Printf("困る", "%d) %lums 開始：左 %.1fcm（有効%d/%d）／右 %.1fcm（有効%d/%d）→ %sへ。終わり：%s（回転%d回）",
                 i + 1, q.startMs, q.leftCm, q.leftValid, WANDER_SCAN_SAMPLES, q.rightCm, q.rightValid, WANDER_SCAN_SAMPLES,
                 q.turnLeft ? "左" : "右", end, q.steps);
    } else {
      Log_Printf("困る", "%d) %lums 開始：見回しの前に終わった。終わり：%s", i + 1, q.startMs, end);
    }
    int shown = (q.steps < TROUBLE_MAX_STEPS) ? q.steps : TROUBLE_MAX_STEPS;
    for (int k = 0; k < shown; k++) {
      const StepLog &t = q.step[k];
      Log_Printf("困る", "   回転%d：最も近い値 %.1fcm %s（有効%d回%s）／共通部品 %s %s（近い %d/%d）／向き %+.1f°（測っている間に %+.1f°）",
                 k + 1, t.closestCm, t.closestCm >= TROUBLE_CLEAR_CM ? "○" : "×", t.validCount,
                 t.validCount == 0 ? "＝測れた値なし" : "",
                 t.blocked ? "あり" : "なし", t.blocked ? "×" : "○", t.nearCount, OBSTACLE_HISTORY,
                 t.yawDeg, t.yawMovedDeg);
    }
  }
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
  Trace_Mark(name(), StateName(STATE_BACK), nowMs);   // ChangeState を通らないので、ここで足あとに残す
  Log_Printf("困る", "正面が塞がったまま（%.1fcm）。立て直します", backStartCm_);
  LogBegin(nowMs);
  TestStats_RecordTroubleStart();
  Motion_SetSpeed(TROUBLE_BACK_SPEED, TROUBLE_BACK_RAMP_MS, nowMs);
}

void TroubleBehavior::onStart(unsigned long nowMs) {
  active_ = true;
  StartSequence(nowMs);
}

void TroubleBehavior::onStop(unsigned long nowMs) {
  LogFinish(LOG_ABORTED);   // 空いた・あきらめで締めたあとなら、何もしない
  active_ = false;
  Motion_StopTurn(nowMs);
  Motion_Stop(nowMs);
  Neck_Release(NECK_OWNER_RANGE);
}

// 左右のうち遠いほうへ回る。差が小さいときは前回と反対側にする
// （行き止まりで同じ動きを繰り返さないため）。
// 測れなかった側は「遠い」とみなさない（浅い角度の壁はエコーが返らず、壁側を選んでしまうため）
void TroubleBehavior::Decide(unsigned long nowMs) {
  (void)nowMs;
  bool turnLeft;
  bool leftKnown = leftValid_ > 0;
  bool rightKnown = rightValid_ > 0;
  if (!leftKnown && !rightKnown) {
    turnLeft = !lastTurnLeft_;
    Log_Printf("困る", "左右とも測れなかったので前回と反対の%sへ", turnLeft ? "左" : "右");
  } else if (leftKnown != rightKnown) {
    float knownCm = leftKnown ? leftCm_ : rightCm_;
    bool knownFar = knownCm >= TROUBLE_KNOWN_FAR_CM;
    turnLeft = (leftKnown == knownFar);   // 測れた側が十分遠ければ測れた側、近ければ測れなかった側
    Log_Printf("困る", "%sは測れず、%sは %.1fcm（%s %.0fcm）なので%sへその場回転",
               leftKnown ? "右" : "左", leftKnown ? "左" : "右", knownCm,
               knownFar ? "≧" : "<", TROUBLE_KNOWN_FAR_CM, turnLeft ? "左" : "右");
  } else if (fabsf(leftCm_ - rightCm_) < TROUBLE_SIDE_DIFF_CM) {
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
  // 角度を指示して回る。止めるのは motion（角度・時間の上限・回っていない。docs/specs/common_gyro_turn.md）
  Motion_StartTurnDeg(turnKind_, TROUBLE_TURN_STEP_DEG, (unsigned long)TurnTuning_StepMs(turnKind_), nowMs);
}

void TroubleBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  // 持ち上げられている間は状態を進めない。床に戻ったら最初（後退）からやり直す
  if (Safety_IsLifted()) {
    if (!lifted_) {
      lifted_ = true;
      LogFinish(LOG_ABORTED);
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
      scan_.begin(SCAN_LEFT_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      ChangeState(STATE_SCAN_LEFT, nowMs);
      return;

    case STATE_SCAN_LEFT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      leftCm_ = scan_.cm();
      leftValid_ = scan_.validCount();
      scan_.begin(SCAN_RIGHT_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      ChangeState(STATE_SCAN_RIGHT, nowMs);
      return;

    case STATE_SCAN_RIGHT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      rightCm_ = scan_.cm();
      rightValid_ = scan_.validCount();
      Log_Printf("困る", "左 %.1fcm（有効%d/%d）／右 %.1fcm（有効%d/%d）",
                 leftCm_, leftValid_, WANDER_SCAN_SAMPLES, rightCm_, rightValid_, WANDER_SCAN_SAMPLES);
      Decide(nowMs);
      if (logNow_ != nullptr) {
        logNow_->scanned = true;
        logNow_->leftCm = leftCm_;
        logNow_->rightCm = rightCm_;
        logNow_->leftValid = (uint8_t)leftValid_;
        logNow_->rightValid = (uint8_t)rightValid_;
        logNow_->turnLeft = (turnKind_ == TURN_ROTATE_LEFT);
      }
      Neck_Release(NECK_OWNER_RANGE);   // 回る間は安全層が首を正面に固定する
      StartTurnStep(nowMs);
      return;

    case STATE_TURN:
      // motion が止めるのを待つ。どの止まり方でも、1回の回転が終わったものとして測り直しへ進む
      if (Motion_IsTurning()) {
        return;
      }
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
        checkYawStartDeg_ = Gyro_YawDeg();   // 記録用：測り直しの間に向きが動いたかを見る
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
      // 経過の記録（判定に使った値を、そのまま残す）
      if (logNow_ != nullptr && steps_ >= 1 && steps_ <= TROUBLE_MAX_STEPS) {
        StepLog &t = logNow_->step[steps_ - 1];
        t.closestCm = checkClosestCm_;
        t.validCount = (uint8_t)checkValidCount_;
        t.nearCount = (uint8_t)Obstacle_NearCount();
        t.blocked = Obstacle_IsBlocked();
        t.yawDeg = Gyro_YawDeg();
        t.yawMovedDeg = t.yawDeg - checkYawStartDeg_;
        logNow_->steps = (uint8_t)steps_;
      }
      // 「空いた」＝ 自分の基準（最も近い値が TROUBLE_CLEAR_CM 以上）と
      //   共通部品の判定（あり→なしに戻った）の両方（docs/specs/15_trouble.md）
      if (checkClosestCm_ >= TROUBLE_CLEAR_CM && !Obstacle_IsBlocked()) {
        Log_Printf("困る", "正面が空いた（%.1fcm、回転%d回）。うろうろに戻ります", checkClosestCm_, steps_);
        LogFinish(LOG_CLEARED);
        TestStats_RecordTroubleCleared();
        active_ = false;   // 優先度が0になり、調停が ID25 に戻す
        return;
      }
      Log_Printf("困る", "%d回目の回転のあと 正面 %.1fcm（まだ塞がっている：最も近い値が %.0fcm %s／共通部品は「%s」。有効%d回）",
                 steps_, checkClosestCm_, TROUBLE_CLEAR_CM, checkClosestCm_ >= TROUBLE_CLEAR_CM ? "以上 ○" : "未満 ×",
                 Obstacle_IsBlocked() ? "あり ×" : "なし ○", checkValidCount_);
      if (steps_ >= TROUBLE_MAX_STEPS) {
        Log_Printf("困る", "%d回まわっても空きません。%lums 休みます",
                   steps_, (unsigned long)TROUBLE_GIVEUP_REST_MS);
        LogFinish(LOG_GIVEUP);
        TestStats_RecordTroubleGiveup();
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
