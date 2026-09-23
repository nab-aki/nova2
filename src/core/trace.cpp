#include "trace.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_reset.h"
#include "../hal/hal_storage.h"
#include "../hal/hal_trace.h"
#include "motion.h"

// NVS に写した「落ちる直前の流れ」。構造を変えたら magic も変える（古い保存値は読み捨てられる）
#define TRACE_SAVED_MAGIC  0x54524331u   // "TRC1"

struct TraceSaved {
  uint32_t magic;
  uint32_t savedAtMs;   // 写したときの millis（＝落ちた直後の起動の、ごく早い時刻）
  uint8_t count;
  uint8_t resetCode;    // 落ち方（このブートのリセット理由。ふつうはブラウンアウト）
  uint16_t pad;
  TraceRecord ring[TRACE_MAX_RECORDS];
};

static TraceSaved saved;
static bool hasSaved = false;
static bool currentBootIsBrownout = false;   // 今回の起動そのものがブラウンアウトだったか
static unsigned long lastAliveMs = 0;

// いまのモーターの出力を、いちばん新しい記録に刻む。
// 回転中は生のPWM（直進の目標は0のままなので、これがないと「止まっている」ように見える）
static void RecordMotor(unsigned long nowMs) {
  if (Motion_IsTurning()) {
    RtcTrace_Motor(Motion_GetSpeed(), Motion_GetTarget(), Motion_GetTurnPwm(),
                   (uint8_t)Motion_GetTurnKind(), nowMs);
  } else {
    RtcTrace_Motor(Motion_GetSpeed(), Motion_GetTarget(), 0, TRACE_TURN_NONE, nowMs);
  }
}

// 1件を1行にまとめて出す
static void PrintRecord(int index, const TraceRecord *r, bool last) {
  char motor[48];
  if (r->turnKind == TRACE_TURN_NONE) {
    snprintf(motor, sizeof(motor), "速度%.2f→%.2f", r->speed, r->target);
  } else {
    snprintf(motor, sizeof(motor), "%s PWM%d", Motion_TurnName((TurnKind)r->turnKind), r->turnPwm);
  }
  Log_Printf("足あと", "  %d) %lums %s／%s %s（%lums続いた）%s",
             index, (unsigned long)r->enteredMs, r->behavior, r->state, motor,
             (unsigned long)(r->aliveMs - r->enteredMs),
             last ? " ←ここで途切れた" : "");
}

// 起動直後（モーターが止まっている間）に、取り出した足あとを NVS へ写す
static void SaveCrashTrail(uint8_t resetCode, unsigned long nowMs) {
  int count = RtcTrace_PrevCount();
  saved.magic = TRACE_SAVED_MAGIC;
  saved.savedAtMs = (uint32_t)nowMs;
  saved.count = (uint8_t)count;
  saved.resetCode = resetCode;
  saved.pad = 0;
  for (int i = 0; i < count; i++) {
    const TraceRecord *r = RtcTrace_Prev(i);
    if (r != NULL) {
      saved.ring[i] = *r;
    }
  }
  Storage_Save(STORAGE_KEY_TRACE, &saved, sizeof(saved));
  hasSaved = true;
  Log_Printf("足あと", "落ちる直前の流れ（%d件）をフラッシュに保存しました（x で消すまで t で見られます）", count);
}

void Trace_Setup(uint8_t resetCode, unsigned long nowMs) {
  RtcTrace_Setup();
  lastAliveMs = 0;
  currentBootIsBrownout = Reset_IsBrownout(resetCode);

  hasSaved = Storage_Load(STORAGE_KEY_TRACE, &saved, sizeof(saved)) &&
             saved.magic == TRACE_SAVED_MAGIC &&
             saved.count <= TRACE_MAX_RECORDS;

  // ブラウンアウトで落ちた直後の起動なら、RAM の控えが次のリセットで消える前にフラッシュへ写す。
  // 記録が空のときは写さない（一時停止のまま2回目のブラウンアウトが起きても、前の記録を潰さない）
  if (Reset_IsBrownout(resetCode) && RtcTrace_PrevCount() > 0) {
    SaveCrashTrail(resetCode, nowMs);
  }
}

void Trace_Mark(const char *behavior, const char *state, unsigned long nowMs) {
  RtcTrace_Add(behavior, state, nowMs);
  RecordMotor(nowMs);
  lastAliveMs = nowMs;
}

void Trace_Update(unsigned long nowMs) {
  if (nowMs - lastAliveMs < TRACE_ALIVE_INTERVAL_MS) {
    return;
  }
  lastAliveMs = nowMs;
  RecordMotor(nowMs);
}

void Trace_PrintPrevious(const char *title) {
  int count = RtcTrace_PrevCount();
  if (count <= 0) {
    Log_Printf("足あと", "%s：記録はありません（電源を入れ直した直後か、まだ何も記録していません）", title);
    return;
  }
  Log_Printf("足あと", "%s（%d件。時刻は前回の起動からの millis。古い順）", title, count);
  for (int i = 0; i < count; i++) {
    const TraceRecord *r = RtcTrace_Prev(i);
    if (r != NULL) {
      PrintRecord(i + 1, r, i == count - 1);
    }
  }
}

void Trace_Print(void) {
  if (hasSaved && saved.count > 0) {
    unsigned long lastMs = (unsigned long)saved.ring[saved.count - 1].aliveMs;
    Log_Printf("足あと", "保存してある「落ちる直前の流れ」（%d件。起動:%s。最後の記録の時刻%lums。"
               "時刻は落ちた回の millis。古い順。x で消えます）",
               saved.count, Reset_ReasonName(saved.resetCode), lastMs);
    if (!currentBootIsBrownout) {
      Log_Printf("足あと", "  ※これは以前の起動で保存された記録です。今回の起動はブラウンアウトではありません");
    }
    for (int i = 0; i < saved.count; i++) {
      PrintRecord(i + 1, &saved.ring[i], i == saved.count - 1);
    }
    return;
  }
  Trace_PrintPrevious("前回の起動の直前の流れ");
}

void Trace_ClearSaved(void) {
  if (!hasSaved) {
    return;
  }
  Storage_Clear(STORAGE_KEY_TRACE);
  hasSaved = false;
  saved.count = 0;
  Log_Printf("足あと", "保存してあった「落ちる直前の流れ」を消しました");
}
