// 足あと：直前の動作の記録（共通部品。#0。docs/specs/common_trace.md）
// 振る舞い・状態が変わるたびに、「振る舞い名・状態名・時刻・そのときのモーターの出力」を
// RTCメモリ（hal/hal_trace.*）へ残す。ブラウンアウトで落ちても消えないので、
// 次の起動で「何の動作中に落ちたか」が分かる。
//
// RTCメモリの控えは RAM に置くため、次のリセット（シリアルモニタを開いたときの DTR/RTS など）で
// 失われる。そこで、リセット理由がブラウンアウトのときは起動直後（モーターが止まっている間）に
// NVS へも写し、`x` で消すまで `t` で見られるようにする。
#ifndef NOVA_CORE_TRACE_H
#define NOVA_CORE_TRACE_H

#include <Arduino.h>

// 前回のブートぶんを取り出し、今回の記録を空にする（setup() で1回。Trace_Mark より前に呼ぶ）。
// resetCode がブラウンアウトなら、取り出した足あとを NVS へ写す
void Trace_Setup(uint8_t resetCode, unsigned long nowMs);

// 振る舞い・状態が変わったときに呼ぶ。モーターの出力はこの中で core/motion.* から読む
void Trace_Mark(const char *behavior, const char *state, unsigned long nowMs);

// loop() から毎回呼ぶ。TRACE_ALIVE_INTERVAL_MS ごとに、いちばん新しい記録へ
// 「ここまで生きていた時刻」と、そのときのモーターの出力を刻み直す
void Trace_Update(unsigned long nowMs);

// 前回のブートの足あとをシリアルに出す（記録がなければ、その旨を1行だけ出す）
void Trace_PrintPrevious(const char *title);

// t キー用。NVS に保存してある「落ちる直前の流れ」があればそれを、
// なければ前回のブートの足あと（RAM の控え）を出す
void Trace_Print(void);

// x キーで消す（試験の集計といっしょに消える）
void Trace_ClearSaved(void);

#endif // NOVA_CORE_TRACE_H
