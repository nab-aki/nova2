#include "motion.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_motor.h"
#include "obstacle.h"
#include "smoother.h"
#include "turn_tuning.h"

static Smoother speedSmoother(0.0f);
static unsigned long lastUpdateMs = 0;

// 回転（その場回転・片側旋回）。直進とは同時に使わない
static bool turning = false;
static TurnKind turnKind = TURN_ROTATE_LEFT;
static bool turnKicking = false;      // キック（動き出し）の最中か
static unsigned long turnStartMs = 0;

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

const char *Motion_TurnName(TurnKind kind) {
  switch (kind) {
    case TURN_ROTATE_LEFT:  return "その場回転 左";
    case TURN_ROTATE_RIGHT: return "その場回転 右";
    case TURN_PIVOT_LEFT:   return "片側旋回 左";
    default:                return "片側旋回 右";
  }
}

// 回転の種類とPWMから、左右それぞれの出力を決めて出す。
// 左＝M1・M2、右＝M3・M4。左回転は左輪を後退・右輪を前進、左片側旋回は右輪だけ前進。
static void ApplyTurn(TurnKind kind, int pwm) {
  switch (kind) {
    case TURN_ROTATE_LEFT:  Motor_DrivePwm(-pwm, pwm); break;
    case TURN_ROTATE_RIGHT: Motor_DrivePwm(pwm, -pwm); break;
    case TURN_PIVOT_LEFT:   Motor_DrivePwm(0, pwm);    break;
    default:                Motor_DrivePwm(pwm, 0);    break;
  }
}

static bool TurnIsPivot(TurnKind kind) {
  return kind == TURN_PIVOT_LEFT || kind == TURN_PIVOT_RIGHT;
}

// キック・保持のPWMとキックの時間は、実行時の調整値（core/turn_tuning.*）を読む
static int TurnKickPwm(TurnKind kind) {
  return TurnTuning_Get(kind).kickPwm;
}

static int TurnHoldPwm(TurnKind kind) {
  return TurnTuning_Get(kind).holdPwm;
}

static unsigned long TurnKickMs(TurnKind kind) {
  return (unsigned long)TurnTuning_Get(kind).kickMs;
}

// 回転をやめる（内部用。停止の記録を出すかどうかを選べる）
static void CancelTurn(unsigned long nowMs, const char *reason) {
  if (!turning) {
    return;
  }
  turning = false;
  Motor_Stop();
  Obstacle_Reset();   // 回っている間の測距は別の方向を見ている
  if (reason != NULL) {
    Log_Printf("動き", "%s %s（合計 %lums）", Motion_TurnName(turnKind), reason, nowMs - turnStartMs);
  }
}

void Motion_Setup(void) {
  speedSmoother.reset(0.0f);
  turning = false;
  Motor_Stop();
}

void Motion_SetSpeed(float target, unsigned long rampMs, unsigned long nowMs) {
  CancelTurn(nowMs, "取り消し（直進の指示が来た）");   // 直進と回転は同時に使わない
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
  turning = false;          // 回転中でも確実に止める（記録は下の1行にまとめる）
  speedSmoother.reset(0.0f);
  Motor_Stop();
  Log_Printf("動き", "非常停止");
}

void Motion_StartTurn(TurnKind kind, unsigned long nowMs) {
  speedSmoother.reset(0.0f);   // 直進の目標は捨てる
  turning = true;
  turnKind = kind;
  turnKicking = TurnKickMs(kind) > 0;   // キックの時間が0なら、保持のPWMから始める
  turnStartMs = nowMs;
  Obstacle_Reset();            // 回り始める前の測距は、別の方向を向いていたときの値
  if (turnKicking) {
    ApplyTurn(kind, TurnKickPwm(kind));
    Log_Printf("動き", "%s キック PWM%d（%lums）",
               Motion_TurnName(kind), TurnKickPwm(kind), TurnKickMs(kind));
  } else {
    ApplyTurn(kind, TurnHoldPwm(kind));
    Log_Printf("動き", "%s キックなし 保持 PWM%d", Motion_TurnName(kind), TurnHoldPwm(kind));
  }
}

void Motion_StopTurn(unsigned long nowMs) {
  CancelTurn(nowMs, "停止");
}

bool Motion_IsTurning(void) {
  return turning;
}

bool Motion_IsPivoting(void) {
  return turning && TurnIsPivot(turnKind);
}

TurnKind Motion_GetTurnKind(void) {
  return turnKind;
}

void Motion_Update(unsigned long nowMs) {
  // 回転中は直進の出力をしない。キック→保持の切り替えは、間隔を待たずに毎ループ見る
  if (turning) {
    if (turnKicking && nowMs - turnStartMs >= TurnKickMs(turnKind)) {
      turnKicking = false;
      ApplyTurn(turnKind, TurnHoldPwm(turnKind));
      Log_Printf("動き", "%s 保持 PWM%d", Motion_TurnName(turnKind), TurnHoldPwm(turnKind));
    }
    return;
  }

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
