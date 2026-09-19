#include "motion.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_motor.h"
#include "smoother.h"

static Smoother speedSmoother(0.0f);
static unsigned long lastUpdateMs = 0;

// 直進中の速度の揺らぎ。周期の違う2つの波を重ねて、機械的に見えないようにする
static float Wobble(unsigned long nowMs, float speed) {
  float magnitude = fabsf(speed);
  // 低速では入れない（最低PWMを割らないため）。しきい値付近は徐々に効かせる
  float fade = constrain((magnitude - MOTION_WOBBLE_MIN_SPEED) / MOTION_WOBBLE_MIN_SPEED, 0.0f, 1.0f);
  if (fade <= 0.0f) {
    return 0.0f;
  }
  float phase1 = TWO_PI * (float)(nowMs % MOTION_WOBBLE_PERIOD_MS) / MOTION_WOBBLE_PERIOD_MS;
  float phase2 = TWO_PI * (float)(nowMs % MOTION_WOBBLE_PERIOD2_MS) / MOTION_WOBBLE_PERIOD2_MS;
  float wave = 0.6f * sinf(phase1) + 0.4f * sinf(phase2);
  return MOTION_WOBBLE_AMPLITUDE * wave * fade * ((speed >= 0) ? 1.0f : -1.0f);
}

void Motion_Setup(void) {
  speedSmoother.reset(0.0f);
  Motor_Stop();
}

void Motion_SetSpeed(float target, unsigned long rampMs, unsigned long nowMs) {
  target = constrain(target, -1.0f, 1.0f);
  if (target != speedSmoother.target()) {
    Log_Printf("動き", "目標速度 %.2f→%.2f（%lums かけて）", speedSmoother.target(), target, rampMs);
  }
  speedSmoother.setTarget(target, rampMs, nowMs);
}

void Motion_Stop(unsigned long nowMs) {
  Motion_SetSpeed(0.0f, MOTION_DECEL_MS, nowMs);
}

void Motion_EmergencyStop(void) {
  speedSmoother.reset(0.0f);
  Motor_Stop();
  Log_Printf("動き", "非常停止");
}

void Motion_Update(unsigned long nowMs) {
  if (nowMs - lastUpdateMs < MOTION_UPDATE_INTERVAL_MS) {
    return;
  }
  lastUpdateMs = nowMs;

  speedSmoother.update(nowMs);
  float speed = speedSmoother.value();
  float output = speed + Wobble(nowMs, speed);
  Motor_Drive(output, output);
}

float Motion_GetSpeed(void) {
  return speedSmoother.value();
}

float Motion_GetTarget(void) {
  return speedSmoother.target();
}

bool Motion_IsAtTarget(void) {
  return speedSmoother.done();
}
