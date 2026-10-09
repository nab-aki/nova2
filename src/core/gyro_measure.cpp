#include "gyro_measure.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "motion.h"

static GyroStats *target = NULL;

static void OnSample(const GyroSample &s) {
  GyroStats *st = target;
  if (st == NULL) {
    return;
  }
  if (st->count == 0) {
    st->firstUs = s.timeUs;
  }
  st->lastUs = s.timeUs;
  for (int a = 0; a < 3; a++) {
    float v = s.rawDps[a];
    st->sum[a] += v;
    st->sumSq[a] += (double)v * v;
    if (st->count == 0 || v < st->minDps[a]) st->minDps[a] = v;
    if (st->count == 0 || v > st->maxDps[a]) st->maxDps[a] = v;
    if (st->baseReady && fabsf(v - st->baseMean[a]) > st->baseLimit[a]) {
      st->spikes[a]++;
    }
  }
  st->count++;
  st->yawDeg += (double)s.yawRateDps * s.dtS;
  if (s.saturated) {
    st->saturated++;
  }
  // 最初の GYRO_MEASURE_BASE_SAMPLES 件で、飛びを数える基準を決める
  if (!st->baseReady && st->count >= GYRO_MEASURE_BASE_SAMPLES) {
    for (int a = 0; a < 3; a++) {
      double mean = st->sum[a] / st->count;
      double var = st->sumSq[a] / st->count - mean * mean;
      float sd = (var > 0.0) ? (float)sqrt(var) : 0.0f;
      st->baseMean[a] = (float)mean;
      st->baseLimit[a] = max(GYRO_SPIKE_SIGMA * sd, GYRO_SPIKE_MIN_DPS);
    }
    st->baseReady = true;
  }
}

void GyroMeasure_Begin(GyroStats *stats) {
  memset(stats, 0, sizeof(*stats));
  stats->i2cBegin = I2c_GetStats(I2C_DEV_GYRO);
  stats->countersBegin = Gyro_GetCounters();
  target = stats;
  Gyro_SetListener(OnSample);
}

void GyroMeasure_End(void) {
  if (target != NULL) {
    target->i2cEnd = I2c_GetStats(I2C_DEV_GYRO);
    target->countersEnd = Gyro_GetCounters();
  }
  target = NULL;
  Gyro_SetListener(NULL);
}

void GyroMeasure_Report(const GyroStats &st, const char *title, bool header) {
  static const char *const AXIS[3] = {"X", "Y", "Z"};
  if (st.count < 2) {
    Log_Printf("測定", "%s：データがありません（%lu件）", title, (unsigned long)st.count);
    return;
  }
  float durationS = (float)(st.lastUs - st.firstUs) / 1000000.0f;
  float measuredHz = (durationS > 0.0f) ? (float)(st.count - 1) / durationS : 0.0f;
  float driftPerMin = (durationS > 0.0f) ? (float)(st.yawDeg / durationS * 60.0) : 0.0f;
  uint32_t i2cFail = st.i2cEnd.fail - st.i2cBegin.fail;
  uint32_t i2cTotal = st.i2cEnd.total - st.i2cBegin.total;
  uint32_t overruns = st.countersEnd.overruns - st.countersBegin.overruns;
  bool calibrated = Gyro_IsCalibrated();

  Log_Printf("測定", "%s：%lu件・%.1f秒（実測 %.2fHz／補正値 %.2fHz） 角度のずれ %+.2f°（1分あたり %+.2f°）"
             " I2C の失敗 %lu/%lu FIFOあふれ %lu 頭打ち %lu%s",
             title, (unsigned long)st.count, durationS, measuredHz, Gyro_OdrHz(),
             (float)st.yawDeg, driftPerMin, (unsigned long)i2cFail, (unsigned long)i2cTotal,
             (unsigned long)overruns, (unsigned long)st.saturated,
             calibrated ? "" : "【ゼロ点 未補正】");

  // docs/measurements.md に貼れる表の行（時刻・タグなし）
  if (header) {
    Log_Raw("| 区間 | 軸 | 件数 | 平均（補正前）dps | 平均（補正後）dps | 標準偏差 dps | 最小 dps | 最大 dps | 飛び |");
    Log_Raw("|---|---|---|---|---|---|---|---|---|");
  }
  for (int a = 0; a < 3; a++) {
    double mean = st.sum[a] / st.count;
    double var = st.sumSq[a] / st.count - mean * mean;
    float sd = (var > 0.0) ? (float)sqrt(var) : 0.0f;
    char corrected[24];
    if (calibrated) {
      snprintf(corrected, sizeof(corrected), "%+.3f", (float)mean - Gyro_ZeroDps(a));
    } else {
      snprintf(corrected, sizeof(corrected), "未補正");
    }
    Log_Raw("| %s | %s | %lu | %+.3f | %s | %.3f | %+.2f | %+.2f | %lu |",
            title, AXIS[a], (unsigned long)st.count, (float)mean, corrected, sd,
            st.minDps[a], st.maxDps[a], (unsigned long)st.spikes[a]);
  }
  Log_Raw("| %s | まとめ | 実測 %.2fHz | 角度のずれ %+.2f°/分 | I2C の失敗 %lu/%lu | FIFOあふれ %lu | 頭打ち %lu | 飛びの基準 ±%.2f/±%.2f/±%.2f dps | |",
          title, measuredHz, driftPerMin, (unsigned long)i2cFail, (unsigned long)i2cTotal,
          (unsigned long)overruns, (unsigned long)st.saturated,
          st.baseLimit[0], st.baseLimit[1], st.baseLimit[2]);
}

// ------------------------ 静止測定（m キー）------------------------ //

static GyroStats staticStats;
static bool staticRunning = false;
static unsigned long staticStartMs = 0;

void GyroMeasure_StartStatic(unsigned long nowMs) {
  staticRunning = true;
  staticStartMs = nowMs;
  GyroMeasure_Begin(&staticStats);
  Log_Printf("測定", "静止測定を始めます（%d秒。車体に触れないでください）", GYRO_MEASURE_STATIC_MS / 1000);
}

bool GyroMeasure_IsStaticRunning(void) {
  return staticRunning;
}

void GyroMeasure_Update(unsigned long nowMs) {
  if (!staticRunning) {
    return;
  }
  const char *abortReason = NULL;
  if (!Motion_IsStill()) {
    abortReason = "車体が動いた";
  } else if (!Gyro_IsReading()) {
    abortReason = "ジャイロが読めなくなった";
  }
  if (abortReason == NULL && nowMs - staticStartMs < GYRO_MEASURE_STATIC_MS) {
    return;
  }
  staticRunning = false;
  GyroMeasure_End();
  if (abortReason != NULL) {
    Log_Printf("測定", "静止測定を中止しました（%s。%lums で）。ここまでの結果：", abortReason, nowMs - staticStartMs);
  } else {
    Log_Printf("測定", "静止測定が終わりました");
  }
  GyroMeasure_Report(staticStats, "静止", true);
}
