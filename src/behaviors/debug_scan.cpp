#include "debug_scan.h"

#include "../config.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../hal/hal_log.h"
#include "../hal/hal_servo.h"

static const char *const DIR_NAMES[3] = {"正面", "左", "右"};
static const char *const CUT_NAMES[3] = {"持ち上げ", "気づく（障害物「あり」）", "ほかの振る舞い"};

const char *DebugScanBehavior::StateName(State state) {
  switch (state) {
    case STATE_SETTLE:     return "ため";
    case STATE_SCAN_FRONT: return "正面を測る";
    case STATE_SCAN_LEFT:  return "左を測る";
    case STATE_SCAN_RIGHT: return "右を測る";
    case STATE_FACE_FRONT: return "正面へ戻す";
    default:               return "履歴がたまるのを待つ";
  }
}

void DebugScanBehavior::request() {
  busy_ = true;
  fresh_ = true;
}

// うろうろ（ID25）と同じ優先度。ID9（気づく）などは、うろうろ中と同じように割り込める
int DebugScanBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return busy_ ? PRIORITY_WANDER : 0;
}

void DebugScanBehavior::BeginIteration(unsigned long nowMs) {
  if (iteration_ >= DEBUG_SCAN_COUNT) {
    PrintSummary("終わりました");
    busy_ = false;
    inIteration_ = false;
    Neck_Release(NECK_OWNER_RANGE);
    return;
  }
  iteration_++;
  inIteration_ = true;
  state_ = STATE_SETTLE;
  stateStartMs_ = nowMs;
}

void DebugScanBehavior::onStart(unsigned long nowMs) {
  if (fresh_) {
    fresh_ = false;
    iteration_ = 0;
    completed_ = 0;
    lifted_ = false;
    for (int d = 0; d < DIR_COUNT; d++) {
      measured_[d] = 0;
      noEcho_[d] = 0;
      minCm_[d] = 0.0f;
      maxCm_[d] = 0.0f;
      sumCm_[d] = 0.0;
    }
    memset(cutByReason_, 0, sizeof(cutByReason_));
    memset(cutByState_, 0, sizeof(cutByState_));
    Log_Printf("見回し試験", "見回しを%d回くり返します（車体は動きません。ため %dms → 正面 → 左 → 右、各%d回の測距。どのキーでも中断）",
               DEBUG_SCAN_COUNT, WANDER_REST_MS, WANDER_SCAN_SAMPLES);
  }
  BeginIteration(nowMs);   // 割り込みから戻ったときも、次の回を最初（ため）から始める
}

// 見回しの途中で打ち切られた。どの方向の途中で、何が割り込んだかを残す
void DebugScanBehavior::NoteCut(CutReason reason, unsigned long nowMs) {
  if (!inIteration_) {
    return;
  }
  inIteration_ = false;
  cutByReason_[reason]++;
  cutByState_[state_]++;
  Log_Printf("見回し試験", "%d/%d 「%s」の途中で打ち切り（%s。%lums 目。首 %d°・直近の距離 %.1fcm・障害物 %s）",
             iteration_, DEBUG_SCAN_COUNT, StateName(state_), CUT_NAMES[reason], nowMs - stateStartMs_,
             Servo_GetAngle(SERVO_PAN), Obstacle_LastCm(), Obstacle_IsBlocked() ? "あり" : "なし");
}

// ほかの振る舞いに取って代わられた（調停）。障害物「あり」なら ID9 が割り込んでいる
void DebugScanBehavior::onStop(unsigned long nowMs) {
  if (busy_) {
    NoteCut(Obstacle_IsBlocked() ? CUT_NOTICE : CUT_OTHER, nowMs);
  }
  Neck_Release(NECK_OWNER_RANGE);
}

void DebugScanBehavior::abort(const char *reason) {
  if (!busy_) {
    return;
  }
  char why[64];
  snprintf(why, sizeof(why), "中断しました（%s）", reason);
  PrintSummary(why);
  busy_ = false;
  inIteration_ = false;
  Neck_Release(NECK_OWNER_RANGE);
}

void DebugScanBehavior::StoreScan(Dir dir, unsigned long nowMs) {
  cm_[dir] = scan_.cm();
  valid_[dir] = scan_.validCount();
  panDeg_[dir] = Servo_GetAngle(SERVO_PAN);
  tookMs_[dir] = nowMs - stateStartMs_;
}

void DebugScanBehavior::FinishIteration(unsigned long nowMs) {
  completed_++;
  for (int d = 0; d < DIR_COUNT; d++) {
    if (valid_[d] == 0) {
      noEcho_[d]++;
      continue;
    }
    if (measured_[d] == 0 || cm_[d] < minCm_[d]) minCm_[d] = cm_[d];
    if (measured_[d] == 0 || cm_[d] > maxCm_[d]) maxCm_[d] = cm_[d];
    sumCm_[d] += cm_[d];
    measured_[d]++;
  }
  Log_Printf("見回し試験", "%d/%d 正面 %.1fcm（有効%d/%d）／左 %.1fcm（有効%d/%d）／右 %.1fcm（有効%d/%d） "
             "首 %d°/%d°/%d° かかった時間 %lu/%lu/%lums",
             iteration_, DEBUG_SCAN_COUNT,
             cm_[DIR_FRONT], valid_[DIR_FRONT], WANDER_SCAN_SAMPLES,
             cm_[DIR_LEFT], valid_[DIR_LEFT], WANDER_SCAN_SAMPLES,
             cm_[DIR_RIGHT], valid_[DIR_RIGHT], WANDER_SCAN_SAMPLES,
             panDeg_[DIR_FRONT], panDeg_[DIR_LEFT], panDeg_[DIR_RIGHT],
             tookMs_[DIR_FRONT], tookMs_[DIR_LEFT], tookMs_[DIR_RIGHT]);
  inIteration_ = false;
  BeginIteration(nowMs);
}

void DebugScanBehavior::PrintSummary(const char *why) {
  int cuts = 0;
  for (int r = 0; r < CUT_COUNT; r++) {
    cuts += cutByReason_[r];
  }
  Log_Printf("見回し試験", "%s。最後まで測れた回 %d／打ち切り %d（始めた回 %d／%d）", why, completed_, cuts, iteration_, DEBUG_SCAN_COUNT);
  Log_Raw("| 方向 | 有効な回 | 測れず（80cm 扱い） | 最小 cm | 最大 cm | 平均 cm |");
  Log_Raw("|---|---|---|---|---|---|");
  for (int d = 0; d < DIR_COUNT; d++) {
    if (measured_[d] > 0) {
      Log_Raw("| %s | %d | %d | %.1f | %.1f | %.1f |", DIR_NAMES[d], measured_[d], noEcho_[d],
              minCm_[d], maxCm_[d], (float)(sumCm_[d] / measured_[d]));
    } else {
      Log_Raw("| %s | 0 | %d | - | - | - |", DIR_NAMES[d], noEcho_[d]);
    }
  }
  Log_Printf("見回し試験", "打ち切りの理由：%s %d／%s %d／%s %d",
             CUT_NAMES[CUT_LIFT], cutByReason_[CUT_LIFT], CUT_NAMES[CUT_NOTICE], cutByReason_[CUT_NOTICE],
             CUT_NAMES[CUT_OTHER], cutByReason_[CUT_OTHER]);
  Log_Printf("見回し試験", "打ち切られた所：%s %d／%s %d／%s %d／%s %d／%s %d／%s %d",
             StateName(STATE_SETTLE), cutByState_[STATE_SETTLE], StateName(STATE_SCAN_FRONT), cutByState_[STATE_SCAN_FRONT],
             StateName(STATE_SCAN_LEFT), cutByState_[STATE_SCAN_LEFT], StateName(STATE_SCAN_RIGHT), cutByState_[STATE_SCAN_RIGHT],
             StateName(STATE_FACE_FRONT), cutByState_[STATE_FACE_FRONT], StateName(STATE_READY), cutByState_[STATE_READY]);
}

void DebugScanBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  if (!busy_) {
    return;
  }
  // 持ち上げられている間は進めない。床に戻ったら、次の回を最初（ため）から始める（うろうろと同じ）
  if (Safety_IsLifted()) {
    if (!lifted_) {
      lifted_ = true;
      NoteCut(CUT_LIFT, nowMs);
    }
    return;
  }
  if (lifted_) {
    lifted_ = false;
    BeginIteration(nowMs);
    return;
  }

  switch (state_) {
    case STATE_SETTLE:
      if (nowMs - stateStartMs_ < WANDER_REST_MS) {
        return;
      }
      scan_.begin(SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      state_ = STATE_SCAN_FRONT;
      stateStartMs_ = nowMs;
      return;

    case STATE_SCAN_FRONT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      StoreScan(DIR_FRONT, nowMs);
      scan_.begin(SCAN_LEFT_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      state_ = STATE_SCAN_LEFT;
      stateStartMs_ = nowMs;
      return;

    case STATE_SCAN_LEFT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      StoreScan(DIR_LEFT, nowMs);
      scan_.begin(SCAN_RIGHT_DEG, SERVO2_LEVEL_DEG, WANDER_SCAN_SAMPLES, nowMs);
      state_ = STATE_SCAN_RIGHT;
      stateStartMs_ = nowMs;
      return;

    case STATE_SCAN_RIGHT:
      if (!scan_.update(sensors, nowMs)) {
        return;
      }
      StoreScan(DIR_RIGHT, nowMs);
      state_ = STATE_FACE_FRONT;
      stateStartMs_ = nowMs;
      return;

    case STATE_FACE_FRONT:
      // うろうろと同じ：首を正面・水平に戻し、安定を待つ
      if (!Neck_Request(NECK_OWNER_RANGE, SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, nowMs)) {
        return;
      }
      if (!Neck_IsFront() || !Neck_IsSteady(nowMs)) {
        return;
      }
      Neck_Release(NECK_OWNER_RANGE);
      state_ = STATE_READY;
      stateStartMs_ = nowMs;
      return;

    default:
      // うろうろの「歩き出す準備」と同じ：正面の測距の履歴がたまるのを待つ。
      // ここで障害物「あり」になれば、ID9 が割り込む（打ち切りとして数える）
      if (!Obstacle_IsReady()) {
        return;
      }
      FinishIteration(nowMs);
      return;
  }
}
