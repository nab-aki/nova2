// ToF の1枚の解釈（docs/specs/common_tof_scan.md 3章）。
// 段0：通路の判定を「影」で行って記録するだけで、動きには使わない（超音波で走る）。
// 記録するもの：超音波の判定との比較、データの途切れ、床のマス（r5〜r7）の値の分布。
#ifndef NOVA_CORE_TOF_VIEW_H
#define NOVA_CORE_TOF_VIEW_H

#include <Arduino.h>

#include "../hal/hal_tof.h"

// 試験の集計に残す ToF の数（起動からの累計）
struct TofCounts {
  TofState state;
  uint32_t frames;
  uint32_t readyFail;
  uint32_t dataFail;
  uint32_t staleEvents;
  uint32_t staleMaxMs;
  uint32_t periodMaxMs;
};

void TofView_Setup(void);

void TofView_GetCounts(TofCounts *out);

// loop() から毎回呼ぶ。Obstacle_Update のあとに呼ぶ（超音波の判定と比べるため）
void TofView_Update(unsigned long nowMs);

// 影の判定（段0 は記録だけ）
bool TofView_ShadowBlocked(void);

// 通路の中の最も近い距離（mm）。なければ負
int TofView_NearestMm(void);

// 1行の要約（状態行用）
void TofView_FormatStatus(char *buffer, size_t size, unsigned long nowMs);

// t：起動・測距の数と、影の比較、床の値の分布を出す
void TofView_Print(void);

// y：最新の1枚を表で出す（距離mm:状態。* は通路の中で止まる距離より近いマス）
void TofView_PrintFrame(void);

// c：食い違った回の 8×8 の表を出す
void TofView_PrintMismatchFrames(void);

#endif // NOVA_CORE_TOF_VIEW_H
