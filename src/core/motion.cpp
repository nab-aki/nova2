#include "motion.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_motor.h"
#include "gyro.h"
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

// 直進・停止の指示で回転が取り消された時刻。調停の交代では、古い振る舞いの onStop() が
// 回転を取り消してから新しい振る舞いの onStart() が呼ばれるので、「交代の直前まで回っていたか」を後から読めるように残す
static bool turnCanceledValid = false;
static unsigned long turnCanceledMs = 0;

// 角度を指示した回転（Motion_StartTurnDeg。docs/specs/common_gyro_turn.md）
enum TurnStopMode {
  TURN_STOP_NONE,    // 呼ぶ側が Motion_StopTurn() で止める（今までの回転）
  TURN_STOP_ANGLE,   // ジャイロの角度で止める
  TURN_STOP_TIME     // ジャイロが使えないので、時間で止める
};
enum TurnEnd {
  TURN_END_ANGLE,     // 角度に届いた
  TURN_END_LIMIT,     // 時間の上限
  TURN_END_STALL,     // 回っていない
  TURN_END_TIME,      // 時間ベースで回した
  TURN_END_LOST,      // 途中でジャイロを失った
  TURN_END_ABORT,     // 中断（持ち上げ・非常停止・振る舞いの交代・直進の指示）
  TURN_END_COUNT
};
static TurnStopMode turnStopMode = TURN_STOP_NONE;
static unsigned long turnLimitMs = 0;
static float turnStopDeg = 0.0f;         // この角度に届いたら止める（目標 − 惰性の分）
static float turnStartYawDeg = 0.0f;     // 回し始めたときの向き
static uint32_t turnStartOverruns = 0;   // 回し始めたときの FIFOあふれの回数
static unsigned long stallCheckMs = 0;   // 「回っていない」の区切りの始まり
static float stallCheckDeg = 0.0f;       // そのときまでに回った角度
static uint32_t turnEndCounts[TURN_END_COUNT];

// 角度を指示した回転が、外から止められた（中断）ときに数える
static void NoteTurnAborted(void) {
  if (turnStopMode != TURN_STOP_NONE) {
    turnEndCounts[TURN_END_ABORT]++;
    turnStopMode = TURN_STOP_NONE;
  }
}

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
  NoteTurnAborted();        // 角度・時間で自分から止めたときは、先に TURN_STOP_NONE にしてある
  Gyro_OnTurnStop(nowMs);   // 実際に回った角度の記録
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
  if (turning) {
    turnCanceledValid = true;
    turnCanceledMs = nowMs;
  }
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
  bool wasTurning = turning;
  turning = false;          // 回転中でも確実に止める（記録は下の1行にまとめる）
  speedSmoother.reset(0.0f);
  Motor_Stop();
  NoteTurnAborted();
  if (wasTurning) {
    Gyro_OnTurnStop(millis());   // 実際に回った角度の記録（測るだけ）
  }
  Log_Printf("動き", "非常停止");
}

void Motion_StartTurn(TurnKind kind, unsigned long nowMs) {
  NoteTurnAborted();           // 前の回転が残っていたら中断として数え、止め方を「呼ぶ側が止める」に戻す
  speedSmoother.reset(0.0f);   // 直進の目標は捨てる
  turning = true;
  turnKind = kind;
  turnKicking = TurnKickMs(kind) > 0;   // キックの時間が0なら、保持のPWMから始める
  turnStartMs = nowMs;
  Obstacle_Reset();            // 回り始める前の測距は、別の方向を向いていたときの値
  Gyro_OnTurnStart(Motion_TurnName(kind), nowMs);   // 実際に回った角度の記録（測るだけ。動きは変えない）
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

// ------------------------ 角度を指示した回転 ------------------------ //

void Motion_StartTurnDeg(TurnKind kind, float targetDeg, unsigned long limitMs, unsigned long nowMs) {
  Motion_StartTurn(kind, nowMs);
  turnLimitMs = limitMs;
  if (!TurnIsPivot(kind) && Gyro_IsTurnUsable()) {
    turnStopMode = TURN_STOP_ANGLE;
    turnStopDeg = max(targetDeg - GYRO_TURN_COAST_DEG, 0.0f);
    turnStartYawDeg = Gyro_YawDeg();
    turnStartOverruns = Gyro_GetCounters().overruns;
    stallCheckMs = nowMs;
    stallCheckDeg = 0.0f;
    Log_Printf("動き", "%s %.0f°：ジャイロの角度で止めます（%.0f°で止める。時間の上限 %lums）",
               Motion_TurnName(kind), targetDeg, turnStopDeg, limitMs);
  } else {
    turnStopMode = TURN_STOP_TIME;
    Log_Printf("動き", "%s %.0f°：時間ベースで回します（%lums。ジャイロ：%s）",
               Motion_TurnName(kind), targetDeg, limitMs, TurnIsPivot(kind) ? "片側旋回には使わない" : Gyro_StateName());
  }
}

// 角度・時間で自分から止める。止め方を数え、理由を1行出す
static void EndTurnDeg(TurnEnd end, const char *reason, unsigned long nowMs) {
  turnEndCounts[end]++;
  turnStopMode = TURN_STOP_NONE;   // CancelTurn が「中断」として数えないように、先に戻す
  CancelTurn(nowMs, reason);
}

// 回し始めてから回った角度（指示した向きを正とする）
static float TurnedDeg(void) {
  float delta = Gyro_YawDeg() - turnStartYawDeg;
  return (turnKind == TURN_ROTATE_LEFT) ? delta : -delta;
}

static void UpdateTurnDeg(unsigned long nowMs) {
  unsigned long elapsedMs = nowMs - turnStartMs;
  char reason[96];
  if (turnStopMode == TURN_STOP_TIME) {
    if (elapsedMs >= turnLimitMs) {
      EndTurnDeg(TURN_END_TIME, "停止（時間ベース）", nowMs);
    }
    return;
  }
  // 途中でジャイロを失ったら、その場で止める（回り足りない側に倒す）
  if (!Gyro_IsTurnUsable() || Gyro_GetCounters().overruns != turnStartOverruns) {
    snprintf(reason, sizeof(reason), "停止（途中でジャイロを失った。ジャイロ：%s）", Gyro_StateName());
    EndTurnDeg(TURN_END_LOST, reason, nowMs);
    return;
  }
  float turnedDeg = TurnedDeg();
  if (turnedDeg >= turnStopDeg) {
    snprintf(reason, sizeof(reason), "停止（角度に届いた：%.1f°）", turnedDeg);
    EndTurnDeg(TURN_END_ANGLE, reason, nowMs);
    return;
  }
  if (elapsedMs >= turnLimitMs) {
    snprintf(reason, sizeof(reason), "停止（時間の上限。%.1f°で、%.0f°に届かなかった）", turnedDeg, turnStopDeg);
    EndTurnDeg(TURN_END_LIMIT, reason, nowMs);
    return;
  }
  // 回っていない：GYRO_TURN_STALL_MS ごとに区切り、その間に進んだ角度が小さければ止める
  if (nowMs - stallCheckMs >= GYRO_TURN_STALL_MS) {
    float advancedDeg = turnedDeg - stallCheckDeg;
    if (advancedDeg < GYRO_TURN_STALL_DEG) {
      snprintf(reason, sizeof(reason), "停止（回っていない：%dms で %.1f°）", GYRO_TURN_STALL_MS, advancedDeg);
      EndTurnDeg(TURN_END_STALL, reason, nowMs);
      return;
    }
    stallCheckMs = nowMs;
    stallCheckDeg = turnedDeg;
  }
}

void Motion_PrintTurnStats(void) {
  char now[64];
  if (Gyro_IsTurnUsable()) {
    snprintf(now, sizeof(now), "ジャイロの角度で止める");
  } else {
    snprintf(now, sizeof(now), "時間ベース（ジャイロ：%s）", Gyro_StateName());
  }
  Log_Printf("動き", "回転の止め方（起動から。角度を指示した回転だけ）：角度 %lu／時間の上限 %lu／回っていない %lu／時間ベース %lu／"
             "途中で失った %lu／中断 %lu（いま回せば：%s）",
             (unsigned long)turnEndCounts[TURN_END_ANGLE], (unsigned long)turnEndCounts[TURN_END_LIMIT],
             (unsigned long)turnEndCounts[TURN_END_STALL], (unsigned long)turnEndCounts[TURN_END_TIME],
             (unsigned long)turnEndCounts[TURN_END_LOST], (unsigned long)turnEndCounts[TURN_END_ABORT], now);
}

bool Motion_IsTurning(void) {
  return turning;
}

bool Motion_TurnCanceledAt(unsigned long nowMs) {
  return turnCanceledValid && turnCanceledMs == nowMs;
}

bool Motion_IsPivoting(void) {
  return turning && TurnIsPivot(turnKind);
}

TurnKind Motion_GetTurnKind(void) {
  return turnKind;
}

int Motion_GetTurnPwm(void) {
  if (!turning) {
    return 0;
  }
  return turnKicking ? TurnKickPwm(turnKind) : TurnHoldPwm(turnKind);
}

void Motion_Update(unsigned long nowMs) {
  // 回転中は直進の出力をしない。キック→保持の切り替えは、間隔を待たずに毎ループ見る
  if (turning) {
    if (turnKicking && nowMs - turnStartMs >= TurnKickMs(turnKind)) {
      turnKicking = false;
      ApplyTurn(turnKind, TurnHoldPwm(turnKind));
      Log_Printf("動き", "%s 保持 PWM%d", Motion_TurnName(turnKind), TurnHoldPwm(turnKind));
    }
    if (turnStopMode != TURN_STOP_NONE) {
      UpdateTurnDeg(nowMs);   // 角度を指示した回転は、ここで止める判断をする
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

bool Motion_IsStill(void) {
  return fabsf(Motion_GetSpeed()) < MOTOR_SPEED_EPSILON &&
         fabsf(Motion_GetTarget()) < MOTOR_SPEED_EPSILON &&
         !Motion_IsTurning();
}
