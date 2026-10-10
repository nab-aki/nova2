#include "loop_stats.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "motion.h"

#define LOOP_LONG_US 20000   // モーターの更新間隔（MOTION_UPDATE_INTERVAL_MS）と同じ。これを超える周を数える

struct PeriodStats {
  uint32_t count;
  uint32_t longCount;
  uint32_t maxUs;
  uint64_t sumUs;
};

static const char *SECTION_NAMES[LOOP_SEC_COUNT] = {
  "センサー・キー", "ジャイロ", "首・障害物", "振る舞い", "安全層", "目・動き・足あと", "ログ・状態行"
};

static bool haveLast = false;
static uint32_t lastBeginUs = 0;
static uint32_t lastMarkUs = 0;
static uint32_t current[LOOP_SEC_COUNT];       // 今の周の区間ごとの時間
static uint32_t sectionMax[LOOP_SEC_COUNT];
static uint64_t sectionSum[LOOP_SEC_COUNT];
static bool skipLoop = false;                  // キー操作の表示などで長くなった周は、記録に入れない
static uint32_t previous[LOOP_SEC_COUNT];      // 直前の周の区間ごとの時間（周の時間は次の Begin で分かるため）
static uint32_t maxBreakdown[LOOP_SEC_COUNT];  // 周の時間が最大だった周の内訳
static PeriodStats all;
static PeriodStats turning;                    // 回転中の周だけ
static bool previousTurning = false;

static void AddPeriod(PeriodStats *p, uint32_t us) {
  p->count++;
  p->sumUs += us;
  if (us >= LOOP_LONG_US) {
    p->longCount++;
  }
  if (us > p->maxUs) {
    p->maxUs = us;
  }
}

void LoopStats_Begin(void) {
  uint32_t nowUs = micros();
  if (haveLast && !skipLoop) {
    uint32_t period = nowUs - lastBeginUs;
    AddPeriod(&all, period);
    if (previousTurning) {
      AddPeriod(&turning, period);
    }
    if (period >= all.maxUs) {
      memcpy(maxBreakdown, previous, sizeof(maxBreakdown));
    }
  }
  skipLoop = false;
  memcpy(previous, current, sizeof(previous));
  memset(current, 0, sizeof(current));
  previousTurning = Motion_IsTurning();
  lastBeginUs = nowUs;
  lastMarkUs = nowUs;
  haveLast = true;
}

void LoopStats_Mark(LoopSection section) {
  uint32_t nowUs = micros();
  uint32_t us = nowUs - lastMarkUs;
  lastMarkUs = nowUs;
  current[section] += us;
  if (skipLoop) {
    return;
  }
  sectionSum[section] += us;
  if (us > sectionMax[section]) {
    sectionMax[section] = us;
  }
}

void LoopStats_SkipThisLoop(void) {
  skipLoop = true;
}

static void FormatBreakdown(const uint32_t *values, char *out, size_t size) {
  size_t used = 0;
  out[0] = '\0';
  for (int i = 0; i < LOOP_SEC_COUNT && used < size; i++) {
    used += snprintf(out + used, size - used, "%s%s %.2f", i == 0 ? "" : "／", SECTION_NAMES[i], values[i] / 1000.0f);
  }
}

void LoopStats_Print(void) {
  if (all.count == 0) {
    Log_Printf("loop", "まだ記録がありません");
    return;
  }
  Log_Printf("loop", "loop 1周の時間：平均 %.2fms・最大 %.2fms（%lu周）／%dms 以上の周 %lu回",
             (float)((double)all.sumUs / all.count / 1000.0), all.maxUs / 1000.0f,
             (unsigned long)all.count, LOOP_LONG_US / 1000, (unsigned long)all.longCount);
  if (turning.count > 0) {
    Log_Printf("loop", "  回転中の周だけ：平均 %.2fms・最大 %.2fms（%lu周）／%dms 以上の周 %lu回",
               (float)((double)turning.sumUs / turning.count / 1000.0), turning.maxUs / 1000.0f,
               (unsigned long)turning.count, LOOP_LONG_US / 1000, (unsigned long)turning.longCount);
  } else {
    Log_Printf("loop", "  回転中の周だけ：まだ回転していません");
  }
  uint32_t maxValues[LOOP_SEC_COUNT];
  for (int i = 0; i < LOOP_SEC_COUNT; i++) {
    maxValues[i] = sectionMax[i];
  }
  char text[256];
  FormatBreakdown(maxValues, text, sizeof(text));
  Log_Printf("loop", "  区間ごとの最大（ms。区間ごとに別の周）：%s", text);
  FormatBreakdown(maxBreakdown, text, sizeof(text));
  uint32_t sum = 0;
  for (int i = 0; i < LOOP_SEC_COUNT; i++) {
    sum += maxBreakdown[i];
  }
  uint32_t outside = (all.maxUs > sum) ? all.maxUs - sum : 0;
  Log_Printf("loop", "  最大の周の内訳（ms）：%s／loop の外（周の時間 − 区間の合計。yield・割り込み・別タスク）%.2f",
             text, outside / 1000.0f);

  const LogStats &log = Log_GetStats();
  if (log.count > 0) {
    Log_Printf("loop", "  シリアルへの出力：%lu行・平均 %.2fms/行（%.0fバイト/行）・最大 %.2fms（%luバイト）・%.0fms 以上 %lu回",
               (unsigned long)log.count, (float)((double)log.totalUs / log.count / 1000.0),
               (float)((double)log.totalBytes / log.count), log.maxUs / 1000.0f, (unsigned long)log.maxBytes,
               LOG_SLOW_US / 1000.0f, (unsigned long)log.slowCount);
  }
}
