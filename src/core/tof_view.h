// ToF の1枚の解釈（docs/specs/common_tof_scan.md 3章）。
// 段1：通路の判定を止まる判断に使う（obstacle.* から呼ぶ）。見回しは超音波のまま。
// 記録するもの：超音波の判定との比較、データの途切れ、床のマス（r5〜r7）の値の分布（床では止めない）。
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

// 判定を捨てて「不明」に戻す。このあと動きが終わってから取った枚だけで判定し直す
void TofView_Reset(void);

// 判定が決まっているか（不明なら false）
bool TofView_IsKnown(void);

// 通路の中に障害物がある（不明なら false）
bool TofView_IsBlocked(void);

// 通路の中で非常停止の距離より近い（1枚で決める）
bool TofView_IsEmergency(void);

// 通路の中の最も近い距離の最新の値（cm）。まだなければ負
float TofView_LastCm(void);

// 走ってよいか（ToF が測距中で、データが途切れていない）。TOF_ENABLE が 0 なら常に true
bool TofView_CanDrive(void);

// 走れない理由（1行）
const char *TofView_StopReason(void);

// b：TOF_DEBUG_HOLD_MS の間、1枚を届けない（途切れで止まる試験）
void TofView_HoldFrames(unsigned long nowMs);

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
