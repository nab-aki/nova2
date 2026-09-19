#include "demo_drive.h"

#include "../config.h"
#include "../core/motion.h"
#include "../hal/hal_log.h"

int DemoDriveBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return PRIORITY_DEMO_DRIVE;
}

const char *DemoDriveBehavior::StateName(State state) {
  switch (state) {
    case STATE_REST:   return "停止";
    case STATE_ACCEL:  return "加速";
    case STATE_CRUISE: return "巡航";
    default:           return "減速";
  }
}

void DemoDriveBehavior::ChangeState(State next, unsigned long nowMs) {
  Log_Printf("デモ", "%s→%s", StateName(state_), StateName(next));
  state_ = next;
  stateStartMs_ = nowMs;
}

void DemoDriveBehavior::onStart(unsigned long nowMs) {
  // 起動直後にいきなり走らないよう、最初は停止から始める
  state_ = STATE_REST;
  stateStartMs_ = nowMs;
  blocked_ = false;
  Motion_Stop(nowMs);
}

void DemoDriveBehavior::onStop(unsigned long nowMs) {
  Motion_Stop(nowMs);
}

void DemoDriveBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  bool blocked = sensors.distanceValid && sensors.distanceCm < DEMO_STOP_DISTANCE_CM;
  if (blocked != blocked_) {
    blocked_ = blocked;
    if (blocked) {
      Log_Printf("デモ", "正面 %.1fcm に何かある（走らない）", sensors.distanceCm);
    } else {
      Log_Printf("デモ", "正面が空いた");
    }
  }

  unsigned long elapsed = nowMs - stateStartMs_;

  switch (state_) {
    case STATE_REST:
      if (elapsed >= DEMO_REST_MS && !blocked) {
        ChangeState(STATE_ACCEL, nowMs);
        Motion_SetSpeed(DEMO_CRUISE_SPEED, MOTION_ACCEL_MS, nowMs);
      }
      break;

    case STATE_ACCEL:
      if (blocked) {
        ChangeState(STATE_DECEL, nowMs);
        Motion_Stop(nowMs);
      } else if (Motion_IsAtTarget()) {
        ChangeState(STATE_CRUISE, nowMs);
      }
      break;

    case STATE_CRUISE:
      if (blocked || elapsed >= DEMO_CRUISE_MS) {
        ChangeState(STATE_DECEL, nowMs);
        Motion_Stop(nowMs);
      }
      break;

    case STATE_DECEL:
      if (Motion_IsAtTarget()) {
        ChangeState(STATE_REST, nowMs);
      }
      break;
  }
}
