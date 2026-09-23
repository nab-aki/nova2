// 足あと：直前の動作の記録を RTCメモリ に残す（#0。docs/specs/common_trace.md）
// RTC_NOINIT_ATTR に置いた変数は RTC スロースピードメモリ（0x50000000〜）に入り、
// 電源が入ったままの再起動（ソフトリセット・ウォッチドッグ・ブラウンアウト）では消えない。
// 電源を切ると中身は不定になるので、magic で「意味のあるデータか」を見分ける。
//
// このファイルは記録の置き場所だけを受け持ち、何を記録するか（モーターの出力の読み方・
// 表示の仕方）は core/trace.* が決める。
#ifndef NOVA_HAL_TRACE_H
#define NOVA_HAL_TRACE_H

#include <Arduino.h>

#include "../config.h"

// 回転していないことを表す turnKind の値（core/motion.h の TurnKind とぶつからない値）
#define TRACE_TURN_NONE  0xFF

// 1件分の記録。RTCメモリにそのまま置くので固定長にする
struct TraceRecord {
  char behavior[TRACE_TEXT_LEN];  // 振る舞い名（UTF-8。入りきらなければ文字の切れ目で詰める）
  char state[TRACE_TEXT_LEN];     // 状態名
  uint32_t enteredMs;   // その状態に入った時刻（millis）
  uint32_t aliveMs;     // その状態で最後に生きていたことを確かめた時刻（millis）
  float speed;          // そのときの直進の速度（なめらか化後）
  float target;         // そのときの直進の目標速度
  int16_t turnPwm;      // 回転中ならそのPWM（キックか保持）。回転していなければ0
  uint8_t turnKind;     // 回転の種類（TurnKind の値。回転していなければ TRACE_TURN_NONE）
  uint8_t reserved;     // 詰め物（4バイト境界に合わせる）
};

// 前回のブートぶんを控えに取り出し、今回のブートの記録を空にする。
// setup() の早い段階で、Trace_Mark が呼ばれるより前に1回だけ呼ぶ
void RtcTrace_Setup(void);

// 1件記録する（モーターの出力は直後に RtcTrace_Motor で入れる）
void RtcTrace_Add(const char *behavior, const char *state, unsigned long nowMs);

// いちばん新しい記録に、そのときのモーターの出力と「ここまで生きていた時刻」を刻む
void RtcTrace_Motor(float speed, float target, int turnPwm, uint8_t turnKind, unsigned long nowMs);

// 前回のブートから引き継いだ記録の件数（0〜TRACE_MAX_RECORDS）
int RtcTrace_PrevCount(void);

// 前回のブートの記録を古い順に取り出す（0が最も古い）。範囲外は NULL
const TraceRecord *RtcTrace_Prev(int index);

#endif // NOVA_HAL_TRACE_H
