// 目の表情セット（ID5 の最小版）と、まばたきの描画（ID1 の土台）
// 表情は8×8ドットのパターン（左右各8バイト）。表示はこのモジュールだけが行う。
// まばたきは「いま選ばれている表情の上に、短時間だけ半目→閉じ目→半目を重ねる」方式。
#ifndef NOVA_CORE_EYES_H
#define NOVA_CORE_EYES_H

#include <Arduino.h>

enum EyeExpression {
  EYE_NORMAL,     // 通常
  EYE_WIDE,       // 見開く
  EYE_NARROW,     // 細める
  EYE_DROOP,      // 伏せる
  EYE_QUESTION,   // ？
  EYE_CLOSED,     // 閉じ目
  EYE_EXPRESSION_COUNT
};

void Eyes_Setup(void);

// 表情を切り替える（表示の更新は Eyes_Update() で行う）
void Eyes_Set(EyeExpression expression);
EyeExpression Eyes_Get(void);
const char *Eyes_Name(EyeExpression expression);

// まばたきを始める。閉じ目のときや、まばたき中は始めずに false を返す
bool Eyes_StartBlink(unsigned long nowMs);
bool Eyes_IsBlinking(void);

// まばたき1回にかかる時間（ms）
unsigned long Eyes_BlinkDurationMs(void);

// loop() から毎回呼ぶ。まばたきの進行と、表示の更新（変化したときだけ）を行う
void Eyes_Update(unsigned long nowMs);

#endif // NOVA_CORE_EYES_H
