// loop() 1周の時間の記録（共通部品。#0。docs/specs/common_gyro_turn.md 段1）
// 「1回の読み取りで loop を止めた時間」の最大が、I2C だけでは説明できない大きさ（16.94ms・18.03ms）だったため、
// どこで時間を使っているかを切り分ける。測るだけで、動きは変えない。起動からの累計で、保存はしない。
//
// 使い方：loop() の最初で LoopStats_Begin()、区間の終わりごとに LoopStats_Mark(区間)。
//   周の時間 ＝ 前の周の始まりから今の周の始まりまで（loop の外＝yield・割り込み・別タスクの時間も含む）。
//   区間の時間 ＝ 前の Mark（または周の始まり）から、その Mark まで。
#ifndef NOVA_CORE_LOOP_STATS_H
#define NOVA_CORE_LOOP_STATS_H

#include <Arduino.h>

enum LoopSection {
  LOOP_SEC_SENSORS,    // キー・リモコン・ブザー・センサー
  LOOP_SEC_GYRO,       // ジャイロ（FIFO を読む）と、その集計
  LOOP_SEC_NECK,       // 首・障害物の判定
  LOOP_SEC_BEHAVIOR,   // 調停と振る舞い
  LOOP_SEC_SAFETY,     // 安全層
  LOOP_SEC_BODY,       // 目・モーター・足あと・試験の集計
  LOOP_SEC_LOG,        // ジャイロのログの出力・状態行
  LOOP_SEC_COUNT
};

void LoopStats_Begin(void);
void LoopStats_Mark(LoopSection section);
void LoopStats_Print(void);   // t キー
// この周は記録に入れない（キー操作の表示や測定の結果表で長くなる周。通常の動きの時間ではないため）
void LoopStats_SkipThisLoop(void);

#endif // NOVA_CORE_LOOP_STATS_H
