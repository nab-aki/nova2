#include "stuck_watch.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "test_stats.h"
#include "tof_look.h"
#include "tof_view.h"

// 信号B
static StuckScanResult prevScan;
static bool hasPrev = false;
static bool walked = false;               // 前回の見回しのあと、巡航を最後まで終えたか
static unsigned long walkedMs = 0;        // その巡航の時間
static int sameCount = 0;                 // 「同じ」が続いた回数
static unsigned long unchangedSinceMs = 0;  // 「同じ」が続き始めた（基準の見回しの）時刻
static bool shortRun = false;

// 信号A
static bool pushBaseValid = false;
static float pushBaseCm = 0.0f;
static unsigned long pushBaseMs = 0;
static int pushSamples = 0;

// 気づいた内容（ID26 が受け取るまで残す）
static bool detected = false;
static StuckEvent detectedEvent = {STUCK_REASON_PUSH, STUCK_WALL_UNKNOWN};

static const char *const DIR_NAMES[STUCK_DIR_COUNT] = {"正面", "左", "右"};

// 信号B（ToF）：前回の列ごとの最も近い距離
static float prevColumns[TOF_LOOK_COLUMNS];
static uint32_t lastJudged = 0;   // 信号A：最後に見た枚

const char *StuckWatch_ReasonName(StuckReason reason) {
  switch (reason) {
    case STUCK_REASON_PUSH:    return "信号A・巡航中に正面が縮まない";
    case STUCK_REASON_FRONT:   return "信号B・正面不変";
    case STUCK_REASON_CONTACT: return "信号B・横が接触";
    default:                   return "信号B・横だけ";
  }
}

bool StuckWatch_IsSignalA(StuckReason reason) {
  return reason == STUCK_REASON_PUSH;
}

// 見回しの左右から壁の側を決める（測れていて STUCK_SCAN_NEAR_CM 未満の側。両方なら近いほう）
static StuckWallSide WallFromScan(const StuckScanResult &r) {
  bool leftNear = r.valid[STUCK_DIR_LEFT] > 0 && r.cm[STUCK_DIR_LEFT] < STUCK_SCAN_NEAR_CM;
  bool rightNear = r.valid[STUCK_DIR_RIGHT] > 0 && r.cm[STUCK_DIR_RIGHT] < STUCK_SCAN_NEAR_CM;
  if (leftNear && rightNear) {
    return (r.cm[STUCK_DIR_LEFT] <= r.cm[STUCK_DIR_RIGHT]) ? STUCK_WALL_LEFT : STUCK_WALL_RIGHT;
  }
  if (leftNear) return STUCK_WALL_LEFT;
  if (rightNear) return STUCK_WALL_RIGHT;
  return STUCK_WALL_UNKNOWN;
}

static const char *WallName(StuckWallSide wall) {
  switch (wall) {
    case STUCK_WALL_LEFT:  return "左";
    case STUCK_WALL_RIGHT: return "右";
    default:               return "分からない";
  }
}

static void Detect(StuckReason reason, StuckWallSide wall) {
  detected = true;
  detectedEvent.reason = reason;
  detectedEvent.wall = wall;
  shortRun = false;
  TestStats_RecordStuckDetected(reason);
}

void StuckWatch_Reset(bool dropDetection) {
  hasPrev = false;
  walked = false;
  walkedMs = 0;
  sameCount = 0;
  shortRun = false;
  pushBaseValid = false;
  if (dropDetection) {
    detected = false;
  }
}

void StuckWatch_OnCruiseStart(void) {
  pushBaseValid = false;
}

void StuckWatch_OnCruiseSample(const SensorData &sensors, unsigned long nowMs) {
  (void)nowMs;
  if (detected) {
    return;   // ID26 に交代するまでの1ループ
  }
#if TOF_ENABLE
  (void)sensors;
  uint32_t judged = TofView_JudgedCount();
  if (judged == lastJudged) {
    return;
  }
  lastJudged = judged;
  int mm = TofView_NearestMm();
  float cm = mm / 10.0f;
  if (mm < 0 || cm > STUCK_PUSH_MAX_CM) {
    pushBaseValid = false;   // 通路に物がない、または遠い。比べない
    return;
  }
  if (!pushBaseValid) {
    pushBaseValid = true;
    pushBaseCm = cm;
    pushBaseMs = nowMs;
    pushSamples = 1;
    return;
  }
  pushSamples++;
  unsigned long dtTof = nowMs - pushBaseMs;
  float shrinkTof = pushBaseCm - cm;
  if (shrinkTof <= -STUCK_PUSH_JUMP_CM) {
    pushBaseCm = cm;   // 前の物が通路から外れた。別の物として測り直す
    pushBaseMs = nowMs;
    pushSamples = 1;
    return;
  }
  if (dtTof < STUCK_PUSH_WINDOW_MS) {
    return;
  }
  float expectedTof = dtTof * CRUISE_CM_PER_S / 1000.0f;
  if (pushSamples >= STUCK_PUSH_MIN_SAMPLES && shrinkTof < expectedTof * STUCK_UNCHANGED_RATIO) {
    float leftCm = -1.0f;
    float rightCm = -1.0f;
    if (hasPrev) {
      for (int col = 0; col < TOF_LOOK_COLUMNS; col++) {
        float value = prevColumns[col];
        if (value < 0.0f) continue;
        if (col <= STUCK_LEFT_COL_LAST && (leftCm < 0.0f || value < leftCm)) leftCm = value;
        if (col >= STUCK_RIGHT_COL_FIRST && (rightCm < 0.0f || value < rightCm)) rightCm = value;
      }
    }
    bool leftNear = leftCm >= 0.0f && leftCm < STUCK_SCAN_NEAR_CM;
    bool rightNear = rightCm >= 0.0f && rightCm < STUCK_SCAN_NEAR_CM;
    StuckWallSide wallTof = leftNear && rightNear ? (leftCm <= rightCm ? STUCK_WALL_LEFT : STUCK_WALL_RIGHT)
                            : leftNear ? STUCK_WALL_LEFT : rightNear ? STUCK_WALL_RIGHT : STUCK_WALL_UNKNOWN;
    Log_Printf("詰まり", "巡航中に通路の前が縮まない（%lums で %.1f→%.1fcm、期待 %.1fcm の %.0f%%未満。%d枚）。詰まりに気づいた（信号A）",
               dtTof, pushBaseCm, cm, expectedTof, STUCK_UNCHANGED_RATIO * 100.0f, pushSamples);
    Detect(STUCK_REASON_PUSH, wallTof);
    return;
  }
  pushBaseCm = cm;
  pushBaseMs = nowMs;
  pushSamples = 1;
  return;
#endif
  if (!sensors.distanceUpdated || !sensors.distanceNeckSteady) {
    return;
  }
  if (!sensors.distanceValid) {
    pushBaseValid = false;   // 窓の中に「測れず」があったら測り直す
    return;
  }
  if (!pushBaseValid) {
    pushBaseValid = true;
    pushBaseCm = sensors.distanceCm;
    pushBaseMs = sensors.distanceMs;
    pushSamples = 1;
    return;
  }
  pushSamples++;
  unsigned long dt = sensors.distanceMs - pushBaseMs;
  if (dt < STUCK_PUSH_WINDOW_MS) {
    return;
  }
  float expected = dt * CRUISE_CM_PER_S / 1000.0f;
  float shrink = pushBaseCm - sensors.distanceCm;
  if (pushSamples >= STUCK_PUSH_MIN_SAMPLES && shrink < expected * STUCK_UNCHANGED_RATIO) {
    StuckWallSide wall = hasPrev ? WallFromScan(prevScan) : STUCK_WALL_UNKNOWN;
    Log_Printf("詰まり", "巡航中に正面が縮まない（%lums で %.1f→%.1fcm、期待 %.1fcm の %.0f%%未満。有効%d回）。"
               "詰まりに気づいた（信号A）。壁は%s（直前の見回しから）",
               dt, pushBaseCm, sensors.distanceCm, expected, STUCK_UNCHANGED_RATIO * 100.0f, pushSamples,
               WallName(wall));
    Detect(STUCK_REASON_PUSH, wall);
    return;
  }
  // 縮んでいる。この値を新しい基準にして続ける
  pushBaseCm = sensors.distanceCm;
  pushBaseMs = sensors.distanceMs;
  pushSamples = 1;
}

void StuckWatch_OnWalked(unsigned long cruiseMs) {
  walked = true;
  walkedMs = cruiseMs;
}

// 1方向の比べた結果を「正面 測れず／左 22.1→22.3cm」の形で書く
static int FormatDir(char *buf, size_t size, int d, const StuckScanResult &a, const StuckScanResult &b) {
  if (a.valid[d] == 0 || b.valid[d] == 0) {
    return snprintf(buf, size, "%s 測れず", DIR_NAMES[d]);
  }
  return snprintf(buf, size, "%s %.1f→%.1fcm", DIR_NAMES[d], a.cm[d], b.cm[d]);
}

void StuckWatch_OnScan(const StuckScanResult &r, unsigned long nowMs) {
  if (detected) {
    return;
  }
  if (!hasPrev || !walked) {
    // 比べる相手がない（やり直した直後）か、歩いていない。今回を次の相手にする
    prevScan = r;
    hasPrev = true;
    walked = false;
    sameCount = 0;
    shortRun = false;
    unchangedSinceMs = nowMs;
    return;
  }

  int compared = 0;
  bool allSame = true;
  bool frontCompared = false;
  bool sideNear = false;      // 比べた横のどれかが STUCK_SCAN_NEAR_CM 未満
  bool sideContact = false;   // 比べた横のどれかが WANDER_SIDE_VERY_NEAR_CM 未満
  float expectedFront = walkedMs * CRUISE_CM_PER_S / 1000.0f;

  for (int d = 0; d < STUCK_DIR_COUNT; d++) {
    if (prevScan.valid[d] == 0 || r.valid[d] == 0) {
      continue;   // どちらかが「測れず」の方向は比べない
    }
    compared++;
    float diff = fabsf(r.cm[d] - prevScan.cm[d]);
    bool same;
    if (d == STUCK_DIR_FRONT) {
      frontCompared = true;
      same = diff < expectedFront * STUCK_UNCHANGED_RATIO;
    } else {
      same = diff < STUCK_SCAN_SAME_CM;
      if (r.cm[d] < STUCK_SCAN_NEAR_CM) sideNear = true;
      if (r.cm[d] < WANDER_SIDE_VERY_NEAR_CM) sideContact = true;
    }
    if (!same) {
      allSame = false;
    }
  }

  char text[120];
  int n = 0;
  for (int d = 0; d < STUCK_DIR_COUNT && n < (int)sizeof(text); d++) {
    n += snprintf(text + n, sizeof(text) - n, d == 0 ? "" : "／");
    if (n < (int)sizeof(text)) {
      n += FormatDir(text + n, sizeof(text) - n, d, prevScan, r);
    }
  }

  prevScan = r;   // 次回は今回と比べる
  walked = false;

  if (compared == 0 || (allSame && !frontCompared && !sideNear)) {
    // 比べられない（3方向とも片方が測れず）か、横が遠くて見分けられない。数えは進めもやり直しもしない
    return;
  }
  if (!allSame) {
    if (sameCount > 0) {
      Log_Printf("詰まり", "見回しが変わった（%s）。数えをやり直す", text);
    }
    sameCount = 0;
    shortRun = false;
    unchangedSinceMs = nowMs;
    return;
  }

  sameCount++;
  TestStats_RecordStuckUnchangedMs(nowMs - unchangedSinceMs);
  bool strong = frontCompared || sideContact;
  int need = strong ? STUCK_SAME_SCANS_STRONG : STUCK_SAME_SCANS_SIDE;
  StuckReason reason = frontCompared ? STUCK_REASON_FRONT
                       : (sideContact ? STUCK_REASON_CONTACT : STUCK_REASON_SIDE);
  if (sameCount >= need) {
    StuckWallSide wall = WallFromScan(r);
    Log_Printf("詰まり", "詰まりに気づいた（%s。「同じ」%d回：%s、巡航 %lums・正面の期待 %.1fcm）。壁は%s（見回しから）",
               StuckWatch_ReasonName(reason), sameCount, text, walkedMs, expectedFront, WallName(wall));
    Detect(reason, wall);
    return;
  }
  shortRun = true;
  Log_Printf("詰まり", "見回しが前回と同じ %d回目（%s）。%s。あと%d回で確定。次の巡航は %dms",
             sameCount, text, StuckWatch_ReasonName(reason), need - sameCount, STUCK_SUSPECT_RUN_MS);
}

void StuckWatch_OnColumns(const float *cm, unsigned long nowMs) {
  if (detected) {
    return;
  }
  if (!hasPrev || !walked) {
    // 比べる相手がない（やり直した直後）か、歩いていない。今回を次の相手にする
    memcpy(prevColumns, cm, sizeof(prevColumns));
    hasPrev = true;
    walked = false;
    sameCount = 0;
    shortRun = false;
    unchangedSinceMs = nowMs;
    return;
  }

  int compared = 0;
  bool allSame = true;
  bool frontCompared = false;
  bool sideNear = false;
  bool sideContact = false;
  float expectedFront = walkedMs * CRUISE_CM_PER_S / 1000.0f;
  float leftCm = -1.0f;
  float rightCm = -1.0f;
  for (int col = 0; col < TOF_LOOK_COLUMNS; col++) {
    if (cm[col] >= 0.0f && col <= STUCK_LEFT_COL_LAST && (leftCm < 0.0f || cm[col] < leftCm)) leftCm = cm[col];
    if (cm[col] >= 0.0f && col >= STUCK_RIGHT_COL_FIRST && (rightCm < 0.0f || cm[col] < rightCm)) rightCm = cm[col];
    if (prevColumns[col] < 0.0f || cm[col] < 0.0f) {
      continue;   // どちらかが「測れず」の列は比べない
    }
    compared++;
    float diff = fabsf(cm[col] - prevColumns[col]);
    bool same;
    if (col >= STUCK_FRONT_COL_FIRST && col <= STUCK_FRONT_COL_LAST) {
      frontCompared = true;
      same = diff < expectedFront * STUCK_UNCHANGED_RATIO;
    } else {
      same = diff < STUCK_SCAN_SAME_CM;
      if (cm[col] < STUCK_SCAN_NEAR_CM) sideNear = true;
      if (cm[col] < WANDER_SIDE_VERY_NEAR_CM) sideContact = true;
    }
    if (!same) {
      allSame = false;
    }
  }
  memcpy(prevColumns, cm, sizeof(prevColumns));
  walked = false;

  if (compared == 0 || (allSame && !frontCompared && !sideNear)) {
    return;   // 比べられない、または横が遠くて見分けられない。数えは進めもやり直しもしない
  }
  if (!allSame) {
    if (sameCount > 0) {
      Log_Printf("詰まり", "前の3枚の列が変わった（比べた列 %d）。数えをやり直す", compared);
    }
    sameCount = 0;
    shortRun = false;
    unchangedSinceMs = nowMs;
    return;
  }

  sameCount++;
  TestStats_RecordStuckUnchangedMs(nowMs - unchangedSinceMs);
  bool strong = frontCompared || sideContact;
  int need = strong ? STUCK_SAME_SCANS_STRONG : STUCK_SAME_SCANS_SIDE;
  StuckReason reason = frontCompared ? STUCK_REASON_FRONT
                       : (sideContact ? STUCK_REASON_CONTACT : STUCK_REASON_SIDE);
  if (sameCount >= need) {
    bool leftNear = leftCm >= 0.0f && leftCm < STUCK_SCAN_NEAR_CM;
    bool rightNear = rightCm >= 0.0f && rightCm < STUCK_SCAN_NEAR_CM;
    StuckWallSide wall = leftNear && rightNear ? (leftCm <= rightCm ? STUCK_WALL_LEFT : STUCK_WALL_RIGHT)
                         : leftNear ? STUCK_WALL_LEFT : rightNear ? STUCK_WALL_RIGHT : STUCK_WALL_UNKNOWN;
    Log_Printf("詰まり", "詰まりに気づいた（%s。「同じ」%d回。比べた列 %d、巡航 %lums・正面の期待 %.1fcm）。壁は%s",
               StuckWatch_ReasonName(reason), sameCount, compared, walkedMs, expectedFront, WallName(wall));
    Detect(reason, wall);
    return;
  }
  shortRun = true;
  Log_Printf("詰まり", "前の3枚の列が前回と同じ %d回目（%s）。あと%d回で確定。次の巡航は %dms",
             sameCount, StuckWatch_ReasonName(reason), need - sameCount, STUCK_SUSPECT_RUN_MS);
}

bool StuckWatch_WantShortRun(void) {
  return shortRun;
}

bool StuckWatch_IsDetected(void) {
  return detected;
}

StuckEvent StuckWatch_Take(void) {
  detected = false;
  return detectedEvent;
}
