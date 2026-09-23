#include "test_stats.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_reset.h"
#include "../hal/hal_storage.h"
#include "motion.h"
#include "trace.h"

// 1回の試験の集計値。NVSにもこのまま保存する（構造を変えたら古い保存値は読み捨てられる）
struct TestRecord {
  uint32_t elapsedMs;         // 経過時間（最後に保存した時点まで）
  uint8_t startResetCode;     // この試験が始まったブートの、起動時のリセット理由（Reset_ReasonCode）
  uint16_t stopCount;         // ID9：気づいて停止した回数（走行中・回転中の両方を含む）
  uint16_t stopWhileTurning;  // うち、回転中に気づいた回数（内訳。Motion_IsTurning() で判定）
  uint16_t stopDistUnknown;   // うち、停止後の距離が測れなかった回数
  float stopDistSumCm;        // 測れた分の合計（平均用）
  float stopDistMinCm;        // 測れた分の最小
  uint16_t troubleStarts;     // ID15：立て直しを始めた回数
  uint16_t troubleCleared;    // ID15：正面が空いて戻った回数
  uint16_t troubleGiveups;    // ID15：あきらめた回数
  uint16_t pivotPerformed;    // ID25：片側旋回を実施した回数
  uint16_t pivotInterrupted;  // ID25：片側旋回が途中で止められた回数
  uint16_t recoverPerformed;    // ID25：後退+その場回転（張りつき対策）を実施した回数
  uint16_t recoverInterrupted;  // ID25：後退+その場回転が途中で交代された回数
  uint16_t liftCount;         // 持ち上げを検知した回数
};

// NVSに保存する全体。直近3回分のリングバッファ
struct TestStatsStorage {
  uint32_t magic;
  uint8_t headIndex;   // 現在（最新）の枠。validCount==0 のときは無効
  uint8_t validCount;  // 何枠に意味のあるデータが入っているか（0〜3）
  TestRecord ring[3];
};

#define TEST_STATS_MAGIC  0x54534E31u  // "TSN1"

static TestStatsStorage storage;
static TestRecord current;            // 記録中（または保存待ち）の枠の、生きた値
static unsigned long testStartMs = 0;
static unsigned long lastSaveMs = 0;
static bool recording = false;        // 再開してから一時停止するまでの間か
static bool pendingSave = false;      // 一時停止したが、まだ止まりきっていないため保存を待っている
static uint8_t bootResetCode = 0;     // このブートのリセット理由（新しい試験を始めるたびに記録に刻む）

static void ResetRecord(TestRecord *r) {
  *r = TestRecord{};
  r->stopDistMinCm = -1.0f;   // まだ値がない
}

// リングを1つ進めて、新しい試験の枠にする
static void BeginNewTest(unsigned long nowMs) {
  storage.headIndex = (storage.validCount == 0) ? 0 : (uint8_t)((storage.headIndex + 1) % 3);
  if (storage.validCount < 3) {
    storage.validCount++;
  }
  ResetRecord(&current);
  current.startResetCode = bootResetCode;
  testStartMs = nowMs;
  lastSaveMs = nowMs;
  recording = true;
  pendingSave = false;
  Log_Printf("試験", "新しい試験を始めます（%d回目の枠。起動理由：%s）",
             storage.headIndex + 1, Reset_ReasonName(bootResetCode));
}

// 現在の値をリングに書き込み、NVSへ保存する
static void SaveCurrent(unsigned long nowMs, const char *reason) {
  current.elapsedMs = nowMs - testStartMs;
  storage.ring[storage.headIndex] = current;
  Storage_Save(STORAGE_KEY_TEST_STATS, &storage, sizeof(storage));
  Log_Printf("試験", "保存しました（%s。起動:%s 経過%lums 気づく停止%d回(回転中%d回) 困る開始%d/空き%d/あきらめ%d "
             "うろうろ旋回%d/中断%d 後退回転%d/中断%d 持ち上げ%d）",
             reason, Reset_ReasonName(current.startResetCode), current.elapsedMs, current.stopCount,
             current.stopWhileTurning,
             current.troubleStarts, current.troubleCleared, current.troubleGiveups,
             current.pivotPerformed, current.pivotInterrupted,
             current.recoverPerformed, current.recoverInterrupted, current.liftCount);
}

void TestStats_Setup(bool startPaused, unsigned long nowMs) {
  recording = false;
  pendingSave = false;
  bootResetCode = Reset_ReasonCode();
  ResetRecord(&current);

  if (!Storage_Load(STORAGE_KEY_TEST_STATS, &storage, sizeof(storage)) || storage.magic != TEST_STATS_MAGIC) {
    storage.magic = TEST_STATS_MAGIC;
    storage.headIndex = 0;
    storage.validCount = 0;
    for (int i = 0; i < 3; i++) {
      ResetRecord(&storage.ring[i]);
    }
    Log_Printf("試験", "保存されている集計はありません（初回、または構造が変わりました）");
  } else {
    Log_Printf("試験", "保存されている集計を読み込みました（%d件）", storage.validCount);
  }

  if (!startPaused) {
    // 一時停止で始まらない設定は、起動そのものを「再開」として扱う
    BeginNewTest(nowMs);
  }
}

void TestStats_Update(unsigned long nowMs) {
  if (!recording && !pendingSave) {
    return;   // 試験を始めていない（一時停止のまま。まだ一度も再開していない）
  }
  if (!Motion_IsStill()) {
    return;   // 走行中はフラッシュに書かない
  }
  if (pendingSave) {
    SaveCurrent(nowMs, "一時停止");
    pendingSave = false;
    lastSaveMs = nowMs;
    return;
  }
  if (nowMs - lastSaveMs < TEST_STATS_SAVE_INTERVAL_MS) {
    return;
  }
  SaveCurrent(nowMs, "周期保存");
  lastSaveMs = nowMs;
}

void TestStats_OnPauseToggle(bool paused, unsigned long nowMs) {
  if (paused) {
    if (recording) {
      recording = false;
      pendingSave = true;   // 保存は、なめらかに減速して止まってから（TestStats_Update）
    }
    return;
  }
  // 再開
  if (pendingSave) {
    // まれに、止まりきる前に再開された場合。ベストエフォートでその場で確定・保存してから、次の試験を始める
    SaveCurrent(nowMs, "一時停止（止まる前に再開）");
    pendingSave = false;
  }
  BeginNewTest(nowMs);
}

void TestStats_RecordNoticeStop(float restCm, bool whileTurning) {
  if (!recording) {
    return;
  }
  current.stopCount++;
  if (whileTurning) {
    current.stopWhileTurning++;
  }
  if (restCm < 0.0f) {
    current.stopDistUnknown++;
    return;
  }
  current.stopDistSumCm += restCm;
  if (current.stopDistMinCm < 0.0f || restCm < current.stopDistMinCm) {
    current.stopDistMinCm = restCm;
  }
}

void TestStats_RecordTroubleStart(void) {
  if (!recording) {
    return;
  }
  current.troubleStarts++;
}

void TestStats_RecordTroubleCleared(void) {
  if (!recording) {
    return;
  }
  current.troubleCleared++;
}

void TestStats_RecordTroubleGiveup(void) {
  if (!recording) {
    return;
  }
  current.troubleGiveups++;
}

void TestStats_RecordPivotPerformed(void) {
  if (!recording) {
    return;
  }
  current.pivotPerformed++;
}

void TestStats_RecordPivotInterrupted(void) {
  if (!recording) {
    return;
  }
  current.pivotInterrupted++;
}

void TestStats_RecordRecoverPerformed(void) {
  if (!recording) {
    return;
  }
  current.recoverPerformed++;
}

void TestStats_RecordRecoverInterrupted(void) {
  if (!recording) {
    return;
  }
  current.recoverInterrupted++;
}

void TestStats_RecordLift(void) {
  if (!recording) {
    return;
  }
  current.liftCount++;
}

void TestStats_RequestSave(unsigned long nowMs) {
  if (!recording && !pendingSave) {
    Log_Printf("試験", "保存する試験がありません（まだ再開していません）");
    return;
  }
  if (!Motion_IsStill()) {
    Log_Printf("試験", "走行中は保存しません。止まってから押してください");
    return;
  }
  SaveCurrent(nowMs, "手動保存（h）");
  pendingSave = false;
  lastSaveMs = nowMs;
}

// 集計1件を1行にまとめて出す（elapsedMs は呼び出し側で、生きた値か保存済みの値かを決めて渡す）
static void PrintRecord(int slot, const TestRecord &r, unsigned long elapsedMs, bool live, bool isRecording) {
  char distPart[64];
  int measured = r.stopCount - r.stopDistUnknown;
  if (measured > 0) {
    snprintf(distPart, sizeof(distPart), "最小%.1fcm 平均%.1fcm（測れず%d回）",
             r.stopDistMinCm, r.stopDistSumCm / measured, r.stopDistUnknown);
  } else if (r.stopCount > 0) {
    snprintf(distPart, sizeof(distPart), "すべて測れず（%d回）", r.stopDistUnknown);
  } else {
    snprintf(distPart, sizeof(distPart), "-");
  }
  Log_Printf("試験", "[%d]%s 起動:%s 経過%lums 気づく:停止%d回(うち回転中%d回)(%s) 困る:開始%d/空き%d/あきらめ%d "
             "うろうろ:旋回 実施%d/中断%d 後退回転 実施%d/中断%d 持ち上げ%d回",
             slot, isRecording ? "（記録中）" : (live ? "（保存待ち）" : ""),
             Reset_ReasonName(r.startResetCode),
             elapsedMs, r.stopCount, r.stopWhileTurning, distPart,
             r.troubleStarts, r.troubleCleared, r.troubleGiveups,
             r.pivotPerformed, r.pivotInterrupted,
             r.recoverPerformed, r.recoverInterrupted, r.liftCount);
}

void TestStats_Print(unsigned long nowMs) {
  // validCount は BeginNewTest でしか増えないので、recording・pendingSave が立っていれば
  // 必ず1以上になっている（試験を始めていないのに立つことはない）
  if (storage.validCount == 0) {
    Log_Printf("試験", "まだ試験を1回も始めていません（再開すると始まります）");
    return;
  }
  Log_Printf("試験", "直近%d件（新しい順）：", storage.validCount);
  for (int i = 0; i < storage.validCount; i++) {
    int idx = (storage.headIndex - i + 3) % 3;
    bool isHead = (i == 0);
    bool showLive = isHead && (recording || pendingSave);
    const TestRecord &r = showLive ? current : storage.ring[idx];
    unsigned long liveElapsed = showLive ? (nowMs - testStartMs) : r.elapsedMs;
    PrintRecord(i + 1, r, liveElapsed, showLive, isHead && recording);
  }
}

static void ClearAll(unsigned long nowMs) {
  Storage_Clear(STORAGE_KEY_TEST_STATS);
  Trace_ClearSaved();   // 保存してある「落ちる直前の流れ」も、x でいっしょに消す
  storage.headIndex = 0;
  storage.validCount = 0;
  for (int i = 0; i < 3; i++) {
    ResetRecord(&storage.ring[i]);
  }
  ResetRecord(&current);
  pendingSave = false;
  if (recording) {
    current.startResetCode = bootResetCode;
    testStartMs = nowMs;
    lastSaveMs = nowMs;
    storage.headIndex = 0;
    storage.validCount = 1;
  }
}

void TestStats_HandleClearKey(unsigned long nowMs) {
  static bool armed = false;
  static unsigned long armedAtMs = 0;

  if (armed && nowMs - armedAtMs < TEST_STATS_CLEAR_CONFIRM_MS) {
    ClearAll(nowMs);
    armed = false;
    Log_Printf("試験", "集計を消去しました");
    return;
  }
  armed = true;
  armedAtMs = nowMs;
  Log_Printf("試験", "もう一度 x を押すと集計を消去します（%lums以内）", (unsigned long)TEST_STATS_CLEAR_CONFIRM_MS);
}
