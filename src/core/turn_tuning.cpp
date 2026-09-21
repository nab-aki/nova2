#include "turn_tuning.h"

#include "../config.h"
#include "../hal/hal_log.h"

// 起動時の値は config.h。書き込み直すと元に戻る
static TurnParams rotateParams = {TROUBLE_TURN_STEP_MS, MOTOR_ROTATE_KICK_MS,
                                  MOTOR_ROTATE_KICK_PWM, MOTOR_ROTATE_HOLD_PWM};
static TurnParams pivotParams = {WANDER_AVOID_PIVOT_MS, MOTOR_PIVOT_KICK_MS,
                                 MOTOR_PIVOT_KICK_PWM, MOTOR_PIVOT_HOLD_PWM};

static bool KindIsPivot(TurnKind kind) {
  return kind == TURN_PIVOT_LEFT || kind == TURN_PIVOT_RIGHT;
}

const TurnParams &TurnTuning_Get(TurnKind kind) {
  return KindIsPivot(kind) ? pivotParams : rotateParams;
}

int TurnTuning_StepMs(TurnKind kind) {
  return TurnTuning_Get(kind).stepMs;
}

// 項目ごとの範囲と刻み（config.h の DEBUG_TUNE_*）
struct Range {
  int lo;
  int hi;
  int step;
};

static Range RangeOf(TurnTuneItem item) {
  switch (item) {
    case TUNE_STEP_MS:
      return {DEBUG_TUNE_STEP_MS_MIN, DEBUG_TUNE_STEP_MS_MAX, DEBUG_TUNE_STEP_MS_STEP};
    case TUNE_KICK_MS:
      return {DEBUG_TUNE_KICK_MS_MIN, DEBUG_TUNE_KICK_MS_MAX, DEBUG_TUNE_KICK_MS_STEP};
    default:
      return {DEBUG_TUNE_PWM_MIN, DEBUG_TUNE_PWM_MAX, DEBUG_TUNE_PWM_STEP};
  }
}

static int *FieldOf(TurnParams &params, TurnTuneItem item) {
  switch (item) {
    case TUNE_STEP_MS:  return &params.stepMs;
    case TUNE_KICK_MS:  return &params.kickMs;
    case TUNE_KICK_PWM: return &params.kickPwm;
    default:            return &params.holdPwm;
  }
}

bool TurnTuning_Adjust(bool pivot, TurnTuneItem item, int direction) {
  int *field = FieldOf(pivot ? pivotParams : rotateParams, item);
  Range range = RangeOf(item);
  int next = constrain(*field + direction * range.step, range.lo, range.hi);
  if (next == *field) {
    return false;   // 範囲の端
  }
  *field = next;
  return true;
}

void TurnTuning_Print(bool pivot, bool withPaste) {
  const TurnParams &p = pivot ? pivotParams : rotateParams;
  const char *name = pivot ? "片側旋回" : "その場回転";
  Log_Printf("調整", "%s：1ステップ %dms／キック %dms・PWM %d／保持 PWM %d%s", name,
             p.stepMs, p.kickMs, p.kickPwm, p.holdPwm,
             p.kickMs >= p.stepMs ? "（キックが1ステップ以上なので、保持は出ません）" : "");
  if (!withPaste) {
    return;
  }
  // config.h にそのまま貼れる形（行頭の時刻・タグを付けない）
  Log_Raw("#define %-28s %d", pivot ? "WANDER_AVOID_PIVOT_MS" : "TROUBLE_TURN_STEP_MS", p.stepMs);
  Log_Raw("#define %-28s %d", pivot ? "MOTOR_PIVOT_KICK_MS" : "MOTOR_ROTATE_KICK_MS", p.kickMs);
  Log_Raw("#define %-28s %d", pivot ? "MOTOR_PIVOT_KICK_PWM" : "MOTOR_ROTATE_KICK_PWM", p.kickPwm);
  Log_Raw("#define %-28s %d", pivot ? "MOTOR_PIVOT_HOLD_PWM" : "MOTOR_ROTATE_HOLD_PWM", p.holdPwm);
}
