// ジャイロの測定（#0。スプリント3.5 の第1段階。docs/specs/common_gyro_checklist.md「まず測るだけの段階で記録すること」）
// ある区間の角速度を集計する：各軸の平均（ゼロ点）・標準偏差・最大・最小・飛びの回数、積算した角度のずれ。
//   ・静止測定（m キー）：床に置いたまま GYRO_MEASURE_STATIC_MS のあいだ集計する
//   ・モーターの振動の測定（n キー）：behaviors/debug_gyro_spin.* が、区間ごとの集計にこの部品を使う
// 結果は、docs/measurements.md にそのまま貼れる表の行でも出す。
#ifndef NOVA_CORE_GYRO_MEASURE_H
#define NOVA_CORE_GYRO_MEASURE_H

#include <Arduino.h>

#include "../hal/hal_i2c.h"
#include "gyro.h"

// 1区間の集計
struct GyroStats {
  uint32_t count;
  double sum[3];           // 補正前の角速度（dps）の和
  double sumSq[3];
  float minDps[3];
  float maxDps[3];
  // 飛びの基準：区間の最初の GYRO_MEASURE_BASE_SAMPLES 件の平均と標準偏差
  bool baseReady;
  float baseMean[3];
  float baseLimit[3];      // 基準の平均から、これより離れたら「飛び」
  uint32_t spikes[3];
  double yawDeg;           // ゼロ点を引いた角速度を積算した角度
  uint32_t saturated;
  uint32_t firstUs;
  uint32_t lastUs;
  // 区間の最初と最後の累計（差が、この区間で起きた分）
  I2cStats i2cBegin;
  I2cStats i2cEnd;
  GyroCounters countersBegin;
  GyroCounters countersEnd;
};

// 集計を始める／終える。集計先は同時に1つだけ（始めると、前の集計先は外れる）
void GyroMeasure_Begin(GyroStats *stats);
void GyroMeasure_End(void);

// 集計の結果を出す。title は区間の名前（例：「静止」「前進 PWM726」）。
// header が true なら、表の見出し行も出す
void GyroMeasure_Report(const GyroStats &stats, const char *title, bool header);

// ------------------------ 静止測定（m キー）------------------------ //

// 始める。受け付けたら true（条件は呼ぶ側＝main.cpp が見る）
void GyroMeasure_StartStatic(unsigned long nowMs);
bool GyroMeasure_IsStaticRunning(void);

// loop() から毎回呼ぶ。時間が来たら結果を出す。車体が動いたら中止する
void GyroMeasure_Update(unsigned long nowMs);

#endif // NOVA_CORE_GYRO_MEASURE_H
