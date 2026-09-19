#include "eyes.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_matrix.h"

// 1面のパターン：8バイト=8行（上から順）、各バイトの最上位ビットが左端。
// 左右の目で同じ形を使う（ハイライトは左上寄りで、両目とも同じ側に置く）。
// 実機の向きや見え方は未確認。上下左右が反転して見える場合は、パターン側ではなく
// hal_matrix 側（ライブラリの reverse/flip）で吸収する。
static const uint8_t PATTERN_NORMAL[8] = {
    0x00,   // ........
    0x3C,   // ..####..
    0x5E,   // .#.####.   ← ハイライト
    0x7E,   // .######.
    0x7E,   // .######.
    0x7E,   // .######.
    0x3C,   // ..####..
    0x00};  // ........

static const uint8_t PATTERN_WIDE[8] = {
    0x3C,   // ..####..
    0x7E,   // .######.
    0xBF,   // #.######   ← ハイライト
    0xFF,   // ########
    0xFF,   // ########
    0xFF,   // ########
    0x7E,   // .######.
    0x3C};  // ..####..

static const uint8_t PATTERN_NARROW[8] = {
    0x00,   // ........
    0x00,   // ........
    0x00,   // ........
    0x7E,   // .######.
    0x7E,   // .######.
    0x3C,   // ..####..
    0x00,   // ........
    0x00};  // ........

static const uint8_t PATTERN_DROOP[8] = {
    0x00,   // ........
    0x00,   // ........
    0x00,   // ........
    0x00,   // ........
    0x3C,   // ..####..
    0x7E,   // .######.
    0x7E,   // .######.
    0x3C};  // ..####..

static const uint8_t PATTERN_QUESTION[8] = {
    0x3C,   // ..####..
    0x66,   // .##..##.
    0x06,   // .....##.
    0x0C,   // ....##..
    0x18,   // ...##...
    0x18,   // ...##...
    0x00,   // ........
    0x18};  // ...##...

static const uint8_t PATTERN_CLOSED[8] = {
    0x00,   // ........
    0x00,   // ........
    0x00,   // ........
    0x00,   // ........
    0x7E,   // .######.   ← 閉じたまぶたの線
    0x00,   // ........
    0x00,   // ........
    0x00};  // ........

// まばたきの途中の「半目」（表情には含めない内部用）
static const uint8_t PATTERN_HALF[8] = {
    0x00,   // ........
    0x00,   // ........
    0x00,   // ........
    0x7E,   // .######.
    0x7E,   // .######.
    0x3C,   // ..####..
    0x00,   // ........
    0x00};  // ........

static const uint8_t *const EXPRESSION_PATTERNS[EYE_EXPRESSION_COUNT] = {
    PATTERN_NORMAL, PATTERN_WIDE, PATTERN_NARROW, PATTERN_DROOP, PATTERN_QUESTION, PATTERN_CLOSED};

static const char *const EXPRESSION_NAMES[EYE_EXPRESSION_COUNT] = {
    "通常", "見開く", "細める", "伏せる", "？", "閉じ目"};

// 表示中の絵の識別（表情の番号、または半目）。変わったときだけマトリクスへ送る
#define FRAME_HALF   EYE_EXPRESSION_COUNT
#define FRAME_NONE   (-1)

static EyeExpression expression = EYE_NORMAL;
static int shownFrame = FRAME_NONE;

// まばたきの進行：0=していない、1=閉じかけ（半目）、2=閉じ目、3=開きかけ（半目）
static uint8_t blinkStep = 0;
static unsigned long blinkStepEndMs = 0;

void Eyes_Setup(void) {
  Matrix_SetBrightness(MATRIX_BRIGHTNESS);
  expression = EYE_NORMAL;
  blinkStep = 0;
  shownFrame = FRAME_NONE;
  Eyes_Update(millis());
}

void Eyes_Set(EyeExpression next) {
  if (next >= EYE_EXPRESSION_COUNT || next == expression) {
    return;
  }
  Log_Printf("目", "表情 %s→%s", EXPRESSION_NAMES[expression], EXPRESSION_NAMES[next]);
  expression = next;
}

EyeExpression Eyes_Get(void) {
  return expression;
}

const char *Eyes_Name(EyeExpression e) {
  return (e < EYE_EXPRESSION_COUNT) ? EXPRESSION_NAMES[e] : "?";
}

unsigned long Eyes_BlinkDurationMs(void) {
  return BLINK_FRAME_HALF_MS + BLINK_FRAME_CLOSED_MS + BLINK_FRAME_HALF_MS;
}

bool Eyes_StartBlink(unsigned long nowMs) {
  if (blinkStep != 0 || expression == EYE_CLOSED) {
    return false;
  }
  blinkStep = 1;
  blinkStepEndMs = nowMs + BLINK_FRAME_HALF_MS;
  return true;
}

bool Eyes_IsBlinking(void) {
  return blinkStep != 0;
}

void Eyes_Update(unsigned long nowMs) {
  // まばたきの進行（各段階の時間が来たら次へ）
  if (blinkStep != 0 && (long)(nowMs - blinkStepEndMs) >= 0) {
    if (blinkStep == 1) {
      blinkStep = 2;
      blinkStepEndMs = nowMs + BLINK_FRAME_CLOSED_MS;
    } else if (blinkStep == 2) {
      blinkStep = 3;
      blinkStepEndMs = nowMs + BLINK_FRAME_HALF_MS;
    } else {
      blinkStep = 0;
    }
  }

  int frame;
  if (blinkStep == 1 || blinkStep == 3) {
    frame = FRAME_HALF;
  } else if (blinkStep == 2) {
    frame = EYE_CLOSED;
  } else {
    frame = expression;
  }

  if (frame == shownFrame) {
    return;
  }
  shownFrame = frame;
  const uint8_t *pattern = (frame == FRAME_HALF) ? PATTERN_HALF : EXPRESSION_PATTERNS[frame];
  Matrix_Show(pattern, pattern);
}
