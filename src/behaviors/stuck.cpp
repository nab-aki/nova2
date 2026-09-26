#include "stuck.h"

#include "../config.h"
#include "../core/debug_pause.h"
#include "../core/neck.h"
#include "../core/safety.h"
#include "../core/test_stats.h"
#include "../core/trace.h"
#include "../hal/hal_log.h"

const char *StuckBehavior::StateName(State state) {
  switch (state) {
    case STATE_SETTLE: return "止まる";
    case STATE_THINK:  return "考える";
    case STATE_BACK:   return "後退";
    case STATE_PAUSE:  return "ため（回転前）";
    default:           return "その場回転";
  }
}

void StuckBehavior::ChangeState(State next, unsigned long nowMs) {
  Log_Printf("詰まり", "%s→%s", StateName(state_), StateName(next));
  state_ = next;
  stateStartMs_ = nowMs;
  Trace_Mark(name(), StateName(next), nowMs);   // 足あと（再起動しても残る）
}

// 見張りが気づいたら発動し、回り終えるまで手放さない
int StuckBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  if (Pause_IsPaused()) {
    return 0;   // デバッグの一時停止中は動かない（途中で止まり、再開してもやり直さない）
  }
  if (active_ || StuckWatch_IsDetected()) {
    return PRIORITY_STUCK;
  }
  return 0;
}

// 壁の反対へ回る。壁の側が分からなければ、くり返しなら前回と同じ向き、それ以外はランダム
void StuckBehavior::DecideTurn(unsigned long nowMs) {
  (void)nowMs;
  bool turnLeft;
  const char *basis;
  if (event_.wall == STUCK_WALL_LEFT) {
    turnLeft = false;
    basis = "壁は左（見回しから）";
  } else if (event_.wall == STUCK_WALL_RIGHT) {
    turnLeft = true;
    basis = "壁は右（見回しから）";
  } else if (repeat_) {
    turnLeft = lastTurnLeft_;
    basis = "壁は分からない。くり返しなので前回と同じ向き";
  } else {
    turnLeft = random(2) == 0;
    basis = "壁は分からない（ランダム）";
  }
  turnKind_ = turnLeft ? TURN_ROTATE_LEFT : TURN_ROTATE_RIGHT;
  if (repeat_) {
    turnDeg_ = (int)random(STUCK_TURN_REPEAT_MIN_DEG, STUCK_TURN_REPEAT_MAX_DEG + 1);
  } else {
    turnDeg_ = (int)random(STUCK_TURN_MIN_DEG, STUCK_TURN_MAX_DEG + 1);
  }
  turnMs_ = (unsigned long)(turnDeg_ * ROTATE_CONT_MS_PER_DEG + 0.5f);
  Log_Printf("詰まり", "%s。考える %lums →後退 %lums →ため %lums →%sへ %d°（%lums%s）",
             basis, (unsigned long)STUCK_THINK_MS, (unsigned long)STUCK_BACK_MS,
             (unsigned long)WANDER_RECOVER_PAUSE_MS, turnLeft ? "左" : "右", turnDeg_, turnMs_,
             repeat_ ? "。くり返し" : "");
}

void StuckBehavior::onStart(unsigned long nowMs) {
  active_ = true;
  event_ = StuckWatch_Take();
  repeat_ = escapedOnce_ && (nowMs - escapedMs_ < STUCK_REPEAT_MS);
  if (repeat_) {
    Log_Printf("詰まり", "脱出してから %lums でまた詰まりました（くり返し）", nowMs - escapedMs_);
    TestStats_RecordStuckRepeat();
  }
  DecideTurn(nowMs);
  backStopping_ = false;
  state_ = STATE_SETTLE;
  stateStartMs_ = nowMs;
  Trace_Mark(name(), StateName(STATE_SETTLE), nowMs);   // ChangeState を通らないので、ここで足あとに残す
  Motion_Stop(nowMs);   // 巡航中（信号A）ならなめらかに減速する
}

void StuckBehavior::onStop(unsigned long nowMs) {
  if (active_) {
    Log_Printf("詰まり", "脱出の途中（%s）で交代しました（%s）", StateName(state_),
               Pause_IsPaused() ? "一時停止のため" : "ほかの振る舞いに");
  }
  active_ = false;
  Motion_StopTurn(nowMs);
  Motion_Stop(nowMs);
  Neck_Release(NECK_OWNER_RANGE);
}

void StuckBehavior::Finish(unsigned long nowMs) {
  Log_Printf("詰まり", "脱出しました。うろうろに戻ります");
  TestStats_RecordStuckEscaped(StuckWatch_IsSignalA(event_.reason));
  escapedOnce_ = true;
  escapedMs_ = nowMs;
  lastTurnLeft_ = (turnKind_ == TURN_ROTATE_LEFT);
  active_ = false;   // 優先度が0になり、調停が ID25 に戻す
}

void StuckBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (!active_) {
    return;
  }
  // 持ち上げられたら、ここで終える（床に戻ったら ID25 が見回しからやり直す）
  if (Safety_IsLifted()) {
    Log_Printf("詰まり", "持ち上げられたので、脱出をやめます（%s の途中）", StateName(state_));
    active_ = false;
    return;
  }

  switch (state_) {
    case STATE_SETTLE:
      // 車体が止まり、首が正面・水平で安定してから考え始める（見回しの途中で気づいたときは首を戻す）
      if (!Motion_IsStill()) {
        return;
      }
      if (!Neck_Request(NECK_OWNER_RANGE, SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, nowMs)) {
        return;
      }
      if (!Neck_IsFront() || !Neck_IsSteady(nowMs)) {
        return;
      }
      Neck_Release(NECK_OWNER_RANGE);   // 動く間は安全層が首を正面に固定する
      ChangeState(STATE_THINK, nowMs);
      return;

    case STATE_THINK:
      if (nowMs - stateStartMs_ < STUCK_THINK_MS) {
        return;
      }
      backStopping_ = false;
      Motion_SetSpeed(TROUBLE_BACK_SPEED, TROUBLE_BACK_RAMP_MS, nowMs);
      ChangeState(STATE_BACK, nowMs);   // ここで「？」の目が消える（isThinking）
      return;

    case STATE_BACK:
      // trouble.cpp の STATE_BACK と同じ2段階（後退→減速→静止を待つ）
      if (!backStopping_) {
        if (nowMs - stateStartMs_ < STUCK_BACK_MS) {
          return;
        }
        backStopping_ = true;
        Motion_SetSpeed(0.0f, TROUBLE_BACK_RAMP_MS, nowMs);
        return;
      }
      if (!Motion_IsAtTarget()) {
        return;
      }
      ChangeState(STATE_PAUSE, nowMs);
      return;

    case STATE_PAUSE:
      // 後退の直後に逆向きの回転を重ねない（ブラウンアウト対策。2026-09-22）
      if (nowMs - stateStartMs_ < WANDER_RECOVER_PAUSE_MS) {
        return;
      }
      Motion_StartTurn(turnKind_, nowMs);
      ChangeState(STATE_TURN, nowMs);
      return;

    case STATE_TURN:
      if (nowMs - stateStartMs_ < turnMs_) {
        return;
      }
      Motion_StopTurn(nowMs);
      Finish(nowMs);
      return;
  }
}
