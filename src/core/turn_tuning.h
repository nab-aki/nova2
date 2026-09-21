// 回転の調整値（実行時。デバッグ用）
// その場回転と片側旋回それぞれの「1ステップの時間・キックの時間・キックのPWM・保持のPWM」を、
// 床の上でキーを押して増減しながら探すための入れ物（docs/specs/25_wander.md「回転の調整キー」）。
//
// 起動時の値は config.h の定数。ここで変えた値は書き込み直すと元に戻る（config.h は別途直す）。
// 回転を実行する motion.*、立て直しの ID15、向き変えの ID25、デバッグキーは、すべてこの値を読む。
#ifndef NOVA_CORE_TURN_TUNING_H
#define NOVA_CORE_TURN_TUNING_H

#include <Arduino.h>

#include "motion.h"

struct TurnParams {
  int stepMs;    // 1ステップの時間（ID15 の1回の回転／ID25 の向き変え／デバッグキー）
  int kickMs;    // キック（動き出し）を出す時間
  int kickPwm;   // キックのPWM
  int holdPwm;   // 保持（回り続ける）PWM
};

enum TurnTuneItem {
  TUNE_STEP_MS,
  TUNE_KICK_MS,
  TUNE_KICK_PWM,
  TUNE_HOLD_PWM
};

// 回転の種類から、その場回転（左右共通）／片側旋回（左右共通）の値を返す
const TurnParams &TurnTuning_Get(TurnKind kind);
int TurnTuning_StepMs(TurnKind kind);

// 1項目を、刻みぶん増やす（direction=+1）／減らす（-1）。範囲の端で動かなければ false
bool TurnTuning_Adjust(bool pivot, TurnTuneItem item, int direction);

// 現在の値を1行で表示する。withPaste が true なら、config.h に貼れる #define も出す
void TurnTuning_Print(bool pivot, bool withPaste);

#endif // NOVA_CORE_TURN_TUNING_H
