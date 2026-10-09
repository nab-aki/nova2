#include "debug_gyro_spin.h"

#include "../config.h"
#include "../core/debug_pause.h"
#include "../core/gyro.h"
#include "../core/motion.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../core/turn_tuning.h"
#include "../hal/hal_log.h"
#include "../hal/hal_motor.h"

void DebugGyroSpinBehavior::request() {
  requested_ = true;
}

int DebugGyroSpinBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return isBusy() ? PRIORITY_DEBUG_GYRO_SPIN : 0;
}

void DebugGyroSpinBehavior::begin(unsigned long nowMs) {
  requested_ = false;
  stillDone_ = false;
  forwardDone_ = false;
  rotateDone_ = false;
  snprintf(forwardTitle_, sizeof(forwardTitle_), "前進 PWM%d", Motor_SpeedToPwm(CRUISE_SPEED));
  snprintf(rotateTitle_, sizeof(rotateTitle_), "その場回転 PWM%d", TurnTuning_Get(TURN_ROTATE_LEFT).holdPwm);
  Safety_SetSpinTestActive(true);    // この測定の間だけ、持ち上げによる停止を外す
  Gyro_SetTurnRecording(false);      // 浮かせて回すので、回転の記録（回った角度）は取らない
  Log_Printf("測定", "モーターの振動の測定を始めます：停止 %d秒 → 前進 PWM%d %d秒 → その場回転 PWM%d %d秒",
             GYRO_SPIN_SEGMENT_MS / 1000, Motor_SpeedToPwm(CRUISE_SPEED), GYRO_SPIN_SEGMENT_MS / 1000,
             TurnTuning_Get(TURN_ROTATE_LEFT).holdPwm, GYRO_SPIN_SEGMENT_MS / 1000);
  Log_Printf("測定", "車輪を浮かせたまま、前を30cm以上あけてください。どのキーでも中断できます"
             "（%.0fcm 未満の非常停止と、ライントラッキングが 111 でなくなったときも止まります）",
             OBSTACLE_EMERGENCY_CM);
  enter(PHASE_STILL, nowMs);
}

void DebugGyroSpinBehavior::enter(Phase phase, unsigned long nowMs) {
  phase_ = phase;
  phaseStartMs_ = nowMs;
  collecting_ = false;
  switch (phase) {
    case PHASE_STILL:
      Log_Printf("測定", "区間1：停止");
      break;
    case PHASE_FORWARD:
      Log_Printf("測定", "区間2：前進 PWM%d（最初の %dms は集計しません）",
                 Motor_SpeedToPwm(CRUISE_SPEED), GYRO_SPIN_SKIP_MS);
      Motion_SetSpeed(CRUISE_SPEED, GYRO_SPIN_RAMP_MS, nowMs);
      break;
    case PHASE_GAP:
      Motion_SetSpeed(0.0f, GYRO_SPIN_RAMP_MS, nowMs);
      break;
    case PHASE_ROTATE:
      Log_Printf("測定", "区間3：その場回転 PWM%d（最初の %dms は集計しません）",
                 TurnTuning_Get(TURN_ROTATE_LEFT).holdPwm, GYRO_SPIN_SKIP_MS);
      Motion_StartTurn(TURN_ROTATE_LEFT, nowMs);
      break;
    default:
      break;
  }
}

void DebugGyroSpinBehavior::report() {
  if (!stillDone_ && !forwardDone_ && !rotateDone_) {
    Log_Printf("測定", "最後まで終わった区間がないので、結果はありません");
    return;
  }
  bool header = true;
  if (stillDone_) {
    GyroMeasure_Report(still_, "停止", header);
    header = false;
  }
  if (forwardDone_) {
    GyroMeasure_Report(forward_, forwardTitle_, header);
    header = false;
  }
  if (rotateDone_) {
    GyroMeasure_Report(rotate_, rotateTitle_, header);
  }
}

// 測定を終える。理由をログに出し、持ち上げ停止を必ず元に戻す
void DebugGyroSpinBehavior::finish(const char *reason, bool emergencyStop, unsigned long nowMs) {
  bool wasActive = (phase_ != PHASE_IDLE);
  GyroMeasure_End();
  if (emergencyStop) {
    if (!Motion_IsStill()) {
      Motion_EmergencyStop();
    }
  } else if (Motion_IsTurning()) {
    Motion_StopTurn(nowMs);
  } else if (!Motion_IsStill()) {
    Motion_SetSpeed(0.0f, GYRO_SPIN_RAMP_MS, nowMs);
  }
  phase_ = PHASE_IDLE;
  requested_ = false;
  Safety_SetSpinTestActive(false);
  Gyro_SetTurnRecording(true);
  if (!wasActive) {
    return;
  }
  Log_Printf("測定", "モーターの振動の測定を終えました。止まった理由：%s（持ち上げによる停止を元に戻しました）", reason);
  report();
}

void DebugGyroSpinBehavior::abort(const char *reason, unsigned long nowMs) {
  if (!isBusy()) {
    return;
  }
  finish(reason, true, nowMs);
}

void DebugGyroSpinBehavior::onStart(unsigned long nowMs) {
  begin(nowMs);
}

void DebugGyroSpinBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  if (phase_ == PHASE_IDLE) {
    if (requested_) {
      begin(nowMs);   // 終わってから調停が手放すまでの間に、次の予約が入った場合
    }
    return;
  }

  // 中断の条件。どれも、モーターをすぐ止める
  if (Safety_IsTrackLost()) {
    finish("ライントラッキングが読めない", true, nowMs);
    return;
  }
  if (sensors.trackUpdated && sensors.trackReadOk && sensors.track != SAFETY_LIFT_TRACK) {
    finish("ライン変化（111 でなくなった）", true, nowMs);
    return;
  }
  if (Obstacle_IsEmergency()) {
    finish("非常停止（前が近すぎる）", true, nowMs);
    return;
  }
  if (!Pause_IsPaused()) {
    finish("一時停止が解除された", true, nowMs);
    return;
  }
  if (!Gyro_IsReading()) {
    finish("ジャイロが読めなくなった", true, nowMs);
    return;
  }

  unsigned long elapsed = nowMs - phaseStartMs_;
  switch (phase_) {
    case PHASE_STILL:
      if (!collecting_) {
        collecting_ = true;
        GyroMeasure_Begin(&still_);
      }
      if (elapsed >= GYRO_SPIN_SEGMENT_MS) {
        GyroMeasure_End();
        stillDone_ = true;
        enter(PHASE_FORWARD, nowMs);
      }
      break;

    case PHASE_FORWARD:
      if (!collecting_ && elapsed >= GYRO_SPIN_SKIP_MS) {
        collecting_ = true;
        GyroMeasure_Begin(&forward_);
      }
      if (elapsed >= GYRO_SPIN_SEGMENT_MS) {
        GyroMeasure_End();
        forwardDone_ = true;
        enter(PHASE_GAP, nowMs);
      }
      break;

    case PHASE_GAP:
      // 出力が0になってから「ため」を置いて、その場回転へ（逆向きの大きな電流を重ねない）
      if (elapsed >= GYRO_SPIN_RAMP_MS + GYRO_SPIN_GAP_MS && Motion_IsStill()) {
        enter(PHASE_ROTATE, nowMs);
      }
      break;

    case PHASE_ROTATE:
      if (!collecting_ && elapsed >= GYRO_SPIN_SKIP_MS) {
        collecting_ = true;
        GyroMeasure_Begin(&rotate_);
      }
      if (elapsed >= GYRO_SPIN_SEGMENT_MS) {
        GyroMeasure_End();
        rotateDone_ = true;
        finish("時間切れ（予定どおり最後まで測った）", false, nowMs);
      }
      break;

    default:
      break;
  }
}

void DebugGyroSpinBehavior::onStop(unsigned long nowMs) {
  // ほかの振る舞いに交代されたとき（通常は起きない）も、必ず元に戻す
  if (phase_ != PHASE_IDLE) {
    finish("ほかの振る舞いに交代された", true, nowMs);
  }
  requested_ = false;
}
