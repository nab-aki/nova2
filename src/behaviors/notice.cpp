#include "notice.h"

#include "../config.h"
#include "../core/eyes.h"
#include "../core/motion.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../hal/hal_log.h"

// ------------------------ 車体 ------------------------ //

int NoticeBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return PRIORITY_NOTICE;
}

const char *NoticeBehavior::StateName(State state) {
  switch (state) {
    case STATE_REST:    return "停止";
    case STATE_ACCEL:   return "加速";
    case STATE_CRUISE:  return "巡航";
    case STATE_DECEL:   return "減速";
    default:            return "気づく";
  }
}

void NoticeBehavior::ChangeState(State next, unsigned long nowMs) {
  Log_Printf("気づく", "%s→%s", StateName(state_), StateName(next));
  state_ = next;
  stateStartMs_ = nowMs;
}

void NoticeBehavior::onStart(unsigned long nowMs) {
  // 起動直後にいきなり走らないよう、停止から始める
  state_ = STATE_REST;
  stateStartMs_ = nowMs;
  clearTimerOn_ = false;
  stopReported_ = false;
  Motion_Stop(nowMs);
}

void NoticeBehavior::onStop(unsigned long nowMs) {
  Motion_Stop(nowMs);
}

// 障害物に気づいた。即停止は安全層が行うので、ここでは目標速度を0にして記録を残す
void NoticeBehavior::EnterNoticed(unsigned long nowMs) {
  noticedCm_ = Obstacle_LastCm();
  noticedSpeedOk_ = Obstacle_ApproachSpeed(&noticedSpeed_);
  ChangeState(STATE_NOTICED, nowMs);
  noticedAtMs_ = nowMs;
  clearTimerOn_ = false;
  stopReported_ = false;
  Motion_SetSpeed(0.0f, 0, nowMs);

  if (noticedSpeedOk_) {
    Log_Printf("気づく", "正面 %.1fcm に気づいた（接近速度 %.1fcm/s）。即停止して目を見開く",
               noticedCm_, noticedSpeed_);
  } else {
    Log_Printf("気づく", "正面 %.1fcm に気づいた（接近速度は不明）。即停止して目を見開く", noticedCm_);
  }
}

// 止まりきったころに、停止後の距離を1回だけ出す（完了条件の確認と config.h の見直しに使う）
void NoticeBehavior::ReportStop(unsigned long nowMs) {
  (void)nowMs;
  stopReported_ = true;
  float restCm = Obstacle_LastCm();
  float slide = (noticedCm_ >= 0.0f && restCm >= 0.0f) ? (noticedCm_ - restCm) : -1.0f;
  if (slide >= 0.0f) {
    Log_Printf("気づく", "停止後の距離 %.1fcm（気づいたとき %.1fcm、滑走 %.1fcm、閾値 %.0fcm）",
               restCm, noticedCm_, slide, OBSTACLE_STOP_CM);
  } else {
    Log_Printf("気づく", "停止後の距離 %.1fcm（気づいたとき %.1fcm、閾値 %.0fcm）",
               restCm, noticedCm_, OBSTACLE_STOP_CM);
  }
}

void NoticeBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  bool blocked = Obstacle_IsBlocked();
  unsigned long elapsed = nowMs - stateStartMs_;

  // 走っている最中に障害物を見つけたら、どの状態からでも「気づく」へ
  if (blocked && state_ != STATE_NOTICED && state_ != STATE_REST) {
    EnterNoticed(nowMs);
    return;
  }

  switch (state_) {
    case STATE_REST:
      if (blocked) {
        return;   // 障害物があるうちは走り出さない
      }
      if (elapsed < NOTICE_REST_MS) {
        return;
      }
      if (!Obstacle_IsReady()) {
        return;   // 測距の履歴がたまるまで待つ
      }
      if (!Neck_IsFront() || !Neck_IsSteady(nowMs)) {
        return;   // 首が正面・水平で安定してから走り出す
      }
      ChangeState(STATE_ACCEL, nowMs);
      Motion_SetSpeed(CRUISE_SPEED, MOTION_ACCEL_MS, nowMs);
      break;

    case STATE_ACCEL:
      if (Motion_IsAtTarget()) {
        ChangeState(STATE_CRUISE, nowMs);
      }
      break;

    case STATE_CRUISE:
      if (elapsed >= NOTICE_CRUISE_MS) {
        ChangeState(STATE_DECEL, nowMs);
        Motion_Stop(nowMs);
      }
      break;

    case STATE_DECEL:
      if (Motion_IsAtTarget()) {
        ChangeState(STATE_REST, nowMs);
      }
      break;

    case STATE_NOTICED:
      if (!stopReported_ && nowMs - noticedAtMs_ >= NOTICE_STOP_REPORT_MS) {
        ReportStop(nowMs);
      }
      if (blocked) {
        clearTimerOn_ = false;   // まだ空いていない
        return;
      }
      if (!clearTimerOn_) {
        clearTimerOn_ = true;
        clearSinceMs_ = nowMs;
        Log_Printf("気づく", "正面が空いた（%lums 待ってから前進を考える）", (unsigned long)NOTICE_HOLD_MS);
        return;
      }
      if (nowMs - clearSinceMs_ >= NOTICE_HOLD_MS) {
        ChangeState(STATE_REST, nowMs);
      }
      break;
  }
}

// ------------------------ 目 ------------------------ //

int NoticeEyesBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (!body_->isNoticing()) {
    return 0;
  }
  if ((long)(nowMs - body_->noticedAtMs()) >= (long)NOTICE_WIDE_MS) {
    return 0;   // 見開く時間が過ぎたら、まばたきに戻す
  }
  return PRIORITY_NOTICE_EYES;
}

void NoticeEyesBehavior::onStart(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_WIDE);
}

void NoticeEyesBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
}

void NoticeEyesBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_NORMAL);
}
