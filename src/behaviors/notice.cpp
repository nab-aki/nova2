#include "notice.h"

#include "../config.h"
#include "../core/eyes.h"
#include "../core/motion.h"
#include "../core/obstacle.h"
#include "../core/test_stats.h"
#include "../hal/hal_log.h"

// ------------------------ 車体 ------------------------ //

// 障害物があるときだけ発動する。空いたあとも、見開き・停止後の報告・その後の間合いが
// 終わるまでは手放さない（終わったら優先度0になり、ID25「うろうろ」に戻る）
int NoticeBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (Obstacle_IsBlocked()) {
    return PRIORITY_NOTICE;
  }
  if (state_ != STATE_NOTICED) {
    return 0;
  }
  if ((long)(nowMs - noticedAtMs_) < (long)NOTICE_REACT_MS) {
    return PRIORITY_NOTICE;   // 反応の途中
  }
  if (!clearTimerOn_ || nowMs - clearSinceMs_ < NOTICE_HOLD_MS) {
    return PRIORITY_NOTICE;   // 空いてからの間合い
  }
  return 0;
}

void NoticeBehavior::onStart(unsigned long nowMs) {
  EnterNoticed(nowMs);
}

void NoticeBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  state_ = STATE_IDLE;
}

// 障害物に気づいた。即停止は安全層が行うので、ここでは目標速度を0にして記録を残す
void NoticeBehavior::EnterNoticed(unsigned long nowMs) {
  noticedCm_ = Obstacle_LastCm();
  noticedSpeedOk_ = Obstacle_ApproachSpeed(&noticedSpeed_);
  // 安全層が止めるのはこの後（loop の順番）なので、ここでは気づいた瞬間の速度が読める。
  // 止まっているときに気づいた（目の前に物を置かれた）ぶんは、完了条件の回数に数えない。
  // 回転中は Motion_StartTurn() が速度スムーザーを0に戻しているため noticedWhileMoving_ では
  // 拾えない。下の Motion_SetSpeed() が回転を取り消す前に Motion_IsTurning() を見て別に記録する
  noticedWhileMoving_ = fabsf(Motion_GetSpeed()) >= MOTOR_SPEED_EPSILON;
  noticedWhileTurning_ = Motion_IsTurning();
  state_ = STATE_NOTICED;
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

// 止まりきったころに、停止後の距離を1回だけ出す（ID9 の完了条件の確認と config.h の見直しに使う）
void NoticeBehavior::ReportStop(unsigned long nowMs) {
  (void)nowMs;
  stopReported_ = true;
  float restCm = Obstacle_LastCm();
  float slide = (noticedCm_ >= 0.0f && restCm >= 0.0f) ? (noticedCm_ - restCm) : -1.0f;

  bool countable = noticedWhileMoving_ || noticedWhileTurning_;
  if (!countable) {
    Log_Printf("気づく", "停止後の距離 %.1fcm（止まっているときに気づいたので、通算には数えません）", restCm);
    return;
  }

  stopCount_++;
  TestStats_RecordNoticeStop(restCm, noticedWhileTurning_);
  const char *turnNote = noticedWhileTurning_ ? "、回転中の検知として別途カウント" : "";
  if (slide >= 0.0f) {
    Log_Printf("気づく", "停止後の距離 %.1fcm（気づいたとき %.1fcm、滑走 %.1fcm、閾値 %.0fcm%s）通算%d回目",
               restCm, noticedCm_, slide, OBSTACLE_STOP_CM, turnNote, stopCount_);
  } else {
    Log_Printf("気づく", "停止後の距離 %.1fcm（気づいたとき %.1fcm、閾値 %.0fcm%s）通算%d回目",
               restCm, noticedCm_, OBSTACLE_STOP_CM, turnNote, stopCount_);
  }
  const float contactWarnCm = 10.0f;   // 非常停止の 12cm より下。ここまで近いと接触を疑う
  if (restCm >= 0.0f && restCm < contactWarnCm) {
    Log_Printf("気づく", "※停止後の距離が%.0fcm未満です。接触したおそれがあります", contactWarnCm);
  }
}

void NoticeBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (state_ != STATE_NOTICED) {
    return;
  }

  if (!stopReported_ && nowMs - noticedAtMs_ >= NOTICE_STOP_REPORT_MS) {
    ReportStop(nowMs);
  }

  if (Obstacle_IsBlocked()) {
    clearTimerOn_ = false;   // まだ空いていない
    return;
  }
  if (!clearTimerOn_) {
    clearTimerOn_ = true;
    clearSinceMs_ = nowMs;
    Log_Printf("気づく", "正面が空いた（%lums 待ってから、うろうろに戻る）", (unsigned long)NOTICE_HOLD_MS);
  }
  // 間合いが過ぎると priority() が0を返し、調停が ID25「うろうろ」に戻す
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
