#include "safety.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "motion.h"
#include "neck.h"
#include "obstacle.h"

static bool stopping = false;

void Safety_Setup(void) {
  stopping = false;
}

// 車体が動いている（または動こうとしている）か
static bool BodyIsMoving(void) {
  return fabsf(Motion_GetSpeed()) >= MOTOR_SPEED_EPSILON ||
         fabsf(Motion_GetTarget()) >= MOTOR_SPEED_EPSILON;
}

void Safety_Update(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  bool moving = BodyIsMoving();

  // 1. 走行中は首を正面・水平に固定する。止まったら使用権を返す（停止中は測距・表情が使える）
  if (moving) {
    Neck_Request(NECK_OWNER_SAFETY, SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, nowMs);
  } else if (Neck_Owner() == NECK_OWNER_SAFETY) {
    Neck_Release(NECK_OWNER_SAFETY);
  }

  // 2. 障害物があれば即停止する
  bool danger = Obstacle_IsBlocked() || Obstacle_IsEmergency();
  if (danger && moving) {
    const char *reason = Obstacle_IsEmergency() ? "非常停止（近すぎる）" : "障害物あり";
    Log_Printf("安全", "%s のため即停止（距離 %.1fcm）", reason, Obstacle_LastCm());
    Motion_EmergencyStop();
  }
  stopping = danger;
}

bool Safety_IsStopping(void) {
  return stopping;
}
