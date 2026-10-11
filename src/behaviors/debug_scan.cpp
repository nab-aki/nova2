#include "debug_scan.h"

#include "../config.h"
#include "../core/gyro.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../hal/hal_log.h"
#include "../hal/hal_servo.h"

static const char *const DIR_NAMES[3] = {"正面", "左", "右"};
static const char *const CUT_NAMES[3] = {"持ち上げ", "気づく（障害物「あり」）", "ほかの振る舞い"};

// 左右へ向ける指示の直後のジャイロ。1件読むたびに呼ばれ、指示から DEBUG_SCAN_GYRO_WINDOW_MS の間の最大の絶対値を取る
static bool kickOpen = false;
static uint32_t kickStartUs = 0;
static float kickMaxDps = 0.0f;
static int kickCount = 0;

// 動き出すまでの時間：指示から、ジャイロの絶対値が初めて DEBUG_SCAN_ONSET_DPS 以上になるまで
static bool onsetWatch = false;
static uint32_t onsetStartUs = 0;
static uint32_t onsetLimitUs = 0;   // 0＝次の指示まで
static int onsetFoundMs = -1;

// 上下の動き（キー u）：水平2軸（X・Y）の合成。1つ目の指示から、2つ目の指示を出すまで見る
static bool tiltWatch = false;
static uint32_t tiltStartUs = 0;
static int tiltFoundMs = -1;
static float tiltKickMax = 0.0f;
static int tiltKickCount = 0;

static void OnGyroSample(const GyroSample &sample) {
  float dps = fabsf(sample.yawRateDps);
  if (onsetWatch && onsetFoundMs < 0) {
    int32_t onsetSinceUs = (int32_t)(sample.timeUs - onsetStartUs);
    if (onsetSinceUs >= 0 && (onsetLimitUs == 0 || (uint32_t)onsetSinceUs < onsetLimitUs) && dps >= DEBUG_SCAN_ONSET_DPS) {
      onsetFoundMs = onsetSinceUs / 1000;
    }
  }
  if (tiltWatch) {
    int32_t tiltSinceUs = (int32_t)(sample.timeUs - tiltStartUs);
    if (tiltSinceUs >= 0) {
      float hx = sample.rawDps[0] - Gyro_ZeroDps(0);
      float hy = sample.rawDps[1] - Gyro_ZeroDps(1);
      float h = sqrtf(hx * hx + hy * hy);
      if (tiltFoundMs < 0 && h >= DEBUG_SCAN_TILT_ONSET_DPS) {
        tiltFoundMs = tiltSinceUs / 1000;
      }
      if (tiltSinceUs < (int32_t)DEBUG_SCAN_GYRO_WINDOW_MS * 1000) {
        if (tiltKickCount == 0 || h > tiltKickMax) {
          tiltKickMax = h;
        }
        tiltKickCount++;
      }
    }
  }
  if (!kickOpen) {
    return;
  }
  int32_t sinceUs = (int32_t)(sample.timeUs - kickStartUs);
  if (sinceUs < 0 || sinceUs >= (int32_t)DEBUG_SCAN_GYRO_WINDOW_MS * 1000) {
    return;
  }
  if (kickCount == 0 || dps > kickMaxDps) {
    kickMaxDps = dps;
  }
  kickCount++;
}

static const char *OrderText(bool rightFirst, bool tilt) {
  if (rightFirst) {
    return "正面 → 右 → 左";
  }
  return tilt ? "正面 → 左（上下も同時）→ 右" : "正面 → 左 → 右";
}

static const char *SkipText(void) {
  return Servo_IsSkipUnchangedTilt() ? "省く" : "省かない";
}

static void FormatOnset(int ms, char *out, size_t size) {
  if (ms < 0) {
    snprintf(out, size, "-");
  } else {
    snprintf(out, size, "%d", ms);
  }
}

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

void DebugScanBehavior::request(bool rightFirst, bool tiltWithFirst) {
  busy_ = true;
  fresh_ = true;
  rightFirst_ = rightFirst;
  tiltMode_ = tiltWithFirst;
}

// 試験を終える（最後まで終わった・中断した）
void DebugScanBehavior::Finish(void) {
  busy_ = false;
  inIteration_ = false;
  finishing_ = false;
  pending_ = false;
  kickOpen = false;
  onsetWatch = false;
  onsetIndex_ = -1;
  tiltWatch = false;
  if (skipOverridden_) {
    Servo_SetSkipUnchangedTilt(savedSkip_);   // キー u で固定していた設定を元に戻す
    skipOverridden_ = false;
  }
  Gyro_SetListener(NULL);
  Neck_Release(NECK_OWNER_RANGE);
}

// 左または右を測り始める。ジャイロの記録は、首へ指示が出た時点（使用権が取れた時点）から始める
void DebugScanBehavior::BeginSide(Dir dir, unsigned long nowMs) {
  int tiltDeg = tiltMode_ ? DEBUG_SCAN_TILT_DEG : SERVO2_LEVEL_DEG;   // キー u：1つ目も2つ目も上下は DEBUG_SCAN_TILT_DEG のまま
  scan_.begin(dir == DIR_LEFT ? SCAN_LEFT_DEG : SCAN_RIGHT_DEG, tiltDeg, WANDER_SCAN_SAMPLES, nowMs);
  kickOpen = false;
  kickMaxDps = 0.0f;
  kickCount = 0;
  state_ = (dir == DIR_LEFT) ? STATE_SCAN_LEFT : STATE_SCAN_RIGHT;
  stateStartMs_ = nowMs;
}

// 首へ指示を出した。直前の指示の見張りを終え、この指示の動き出しを見始める（index：0＝1つ目、1＝2つ目、2＝正面へ戻す）
void DebugScanBehavior::OpenMove(int index, uint32_t instructedUs, unsigned long nowMs) {
  CloseMove();
  if (index == 0) {
    FlushPending();   // 前の回の行がまだ出ていなければ、ここで出す
    for (int i = 0; i < 3; i++) {
      onsetMs_[i] = -1;
    }
    tiltOnsetMs_ = -1;
    tiltKickN_ = 0;
    tiltKickDps_ = 0.0f;
    if (tiltMode_) {
      tiltStartUs = instructedUs;
      tiltFoundMs = -1;
      tiltKickMax = 0.0f;
      tiltKickCount = 0;
      tiltWatch = true;
    }
  }
  onsetWatch = true;
  onsetStartUs = instructedUs;
  onsetLimitUs = (index == 2) ? (uint32_t)DEBUG_SCAN_ONSET_FRONT_MS * 1000UL : 0;
  onsetFoundMs = -1;
  onsetIndex_ = index;
  onsetOpenMs_ = nowMs;
}

// 見張りを終えて、見つかった時間（なければ -1）を残す
void DebugScanBehavior::CloseMove(void) {
  if (onsetWatch && onsetIndex_ >= 0) {
    onsetMs_[onsetIndex_] = onsetFoundMs;
  }
  onsetWatch = false;
  if (tiltWatch) {   // 2つ目の指示を出した（または終わり）。上下の見張りもここで終える
    tiltOnsetMs_ = tiltFoundMs;
    tiltKickN_ = tiltKickCount;
    tiltKickDps_ = tiltKickMax;
    tiltWatch = false;
  }
}

// 待たせていた行を、動き出しの時間をつけて出す。まとめの集計もここで行う
void DebugScanBehavior::FlushPending(void) {
  if (!pending_) {
    return;
  }
  CloseMove();
  pending_ = false;
  char t[3][12];
  for (int i = 0; i < 3; i++) {
    FormatOnset(onsetMs_[i], t[i], sizeof(t[i]));
    if (onsetMs_[i] < 0) {
      onsetNone_[i]++;
    } else {
      if (onsetN_[i] == 0 || onsetMs_[i] < onsetMin_[i]) onsetMin_[i] = onsetMs_[i];
      if (onsetN_[i] == 0 || onsetMs_[i] > onsetMax_[i]) onsetMax_[i] = onsetMs_[i];
      onsetN_[i]++;
    }
  }
  // キー u：上下の動き（左右とは別）
  char tiltText[160];
  tiltText[0] = '\0';
  if (tiltMode_) {
    char tOnset[12];
    char tKick[12];
    FormatOnset(tiltOnsetMs_, tOnset, sizeof(tOnset));
    if (tiltKickN_ > 0) {
      snprintf(tKick, sizeof(tKick), "%.1f", tiltKickDps_);
      if (tiltKickCount_ == 0 || tiltKickDps_ < tiltKickMin_) tiltKickMin_ = tiltKickDps_;
      if (tiltKickCount_ == 0 || tiltKickDps_ > tiltKickMax_) tiltKickMax_ = tiltKickDps_;
      tiltKickCount_++;
    } else {
      snprintf(tKick, sizeof(tKick), "-");
    }
    if (tiltOnsetMs_ < 0) {
      tiltOnsetNone_++;
    } else {
      if (tiltOnsetN_ == 0 || tiltOnsetMs_ < tiltOnsetMin_) tiltOnsetMin_ = tiltOnsetMs_;
      if (tiltOnsetN_ == 0 || tiltOnsetMs_ > tiltOnsetMax_) tiltOnsetMax_ = tiltOnsetMs_;
      tiltOnsetN_++;
    }
    snprintf(tiltText, sizeof(tiltText), "／上下（水平2軸の合成。しきい値 %.1fdps）動き出すまで %sms・指示の直後%dmsの最大 %sdps",
             DEBUG_SCAN_TILT_ONSET_DPS, tOnset, DEBUG_SCAN_GYRO_WINDOW_MS, tKick);
  }
  Log_Printf("見回し試験", "%s 動き出すまで（しきい値 %.1fdps）1つ目 %sms／2つ目 %sms／正面へ戻す %sms%s",
             pendingRow_, DEBUG_SCAN_ONSET_DPS, t[0], t[1], t[2], tiltText);
  Log_Printf("見回し試験", "%s", pendingBack_);
}

// うろうろ（ID25）と同じ優先度。ID9（気づく）などは、うろうろ中と同じように割り込める
int DebugScanBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return busy_ ? PRIORITY_WANDER : 0;
}

void DebugScanBehavior::BeginIteration(unsigned long nowMs) {
  if (iteration_ >= DEBUG_SCAN_COUNT) {
    if (pending_) {
      finishing_ = true;   // 最後の回の動き出しを見終わるまで待ってから、まとめを出す
      return;
    }
    PrintSummary("終わりました");
    Finish();
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
    finishing_ = false;
    pending_ = false;
    for (int i = 0; i < 3; i++) {
      onsetMs_[i] = -1;
      onsetN_[i] = 0;
      onsetNone_[i] = 0;
      onsetMin_[i] = 0;
      onsetMax_[i] = 0;
    }
    for (int d = 0; d < DIR_COUNT; d++) {
      measured_[d] = 0;
      noEcho_[d] = 0;
      mismatch_[d] = 0;
      readFail_[d] = 0;
      kickDps_[d] = 0.0f;
      kickSamples_[d] = 0;
      minCm_[d] = 0.0f;
      maxCm_[d] = 0.0f;
      sumCm_[d] = 0.0;
    }
    tiltOnsetMs_ = -1;
    tiltKickN_ = 0;
    tiltKickDps_ = 0.0f;
    tiltOnsetN_ = 0;
    tiltOnsetNone_ = 0;
    tiltOnsetMin_ = 0;
    tiltOnsetMax_ = 0;
    tiltKickCount_ = 0;
    tiltKickMin_ = 0.0f;
    tiltKickMax_ = 0.0f;
    if (tiltMode_) {
      savedSkip_ = Servo_IsSkipUnchangedTilt();
      Servo_SetSkipUnchangedTilt(false);   // キー u：キー k の状態に関係なく、上下の書き込みは省かない
      skipOverridden_ = true;
    }
    memset(cutByReason_, 0, sizeof(cutByReason_));
    memset(cutByState_, 0, sizeof(cutByState_));
    Log_Printf("見回し試験", "見回しを%d回くり返します（車体は動きません。ため %dms → %s、各%d回の測距。どのキーでも中断）",
               DEBUG_SCAN_COUNT, WANDER_REST_MS, OrderText(rightFirst_, tiltMode_), WANDER_SCAN_SAMPLES);
    Log_Printf("見回し試験", "上下のサーボへの書き込み：%s（キー k で切り替え）／ジャイロ：%s", SkipText(), Gyro_StateName());
    if (tiltMode_) {
      Log_Printf("見回し試験", "上下も動かす版：1つ目の指示（左）と同時に servo2 を %d°→%d°（2つ目はそのまま、正面へ戻すとき %d° へ戻す）。"
                 "この版では、キー k に関係なく上下の書き込みは省かない（終わると元に戻す）。上下の動きはジャイロの水平2軸（X・Y）の合成で見る（しきい値 %.1fdps【仮】）。"
                 "上下が上がるので、1つ目・2つ目の距離は水平のときと違う値になる",
                 SERVO2_LEVEL_DEG, DEBUG_SCAN_TILT_DEG, SERVO2_LEVEL_DEG, DEBUG_SCAN_TILT_ONSET_DPS);
    }
  }
  Gyro_SetListener(OnGyroSample);
  BeginIteration(nowMs);   // 割り込みから戻ったときも、次の回を最初（ため）から始める
}

// 見回しの途中で打ち切られた。どの方向の途中で、何が割り込んだかを残す
void DebugScanBehavior::NoteCut(CutReason reason, unsigned long nowMs) {
  if (!inIteration_) {
    return;
  }
  inIteration_ = false;
  kickOpen = false;
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
  Finish();
}

void DebugScanBehavior::StoreScan(Dir dir, unsigned long nowMs) {
  cm_[dir] = scan_.cm();
  valid_[dir] = scan_.validCount();
  panDeg_[dir] = Servo_GetAngle(SERVO_PAN);
  tookMs_[dir] = nowMs - stateStartMs_;
  if (dir != DIR_FRONT) {
    kickDps_[dir] = kickMaxDps;
    kickSamples_[dir] = kickCount;
    kickOpen = false;
  }
  // 切り分け用：サーボのチャンネルの値を PCA9685 から読み戻し、書いたはずの値と比べる（読むだけ。動きは変えない）
  bool anyFail = false;
  bool anyMismatch = false;
  for (int v = 0; v < 2; v++) {
    readOk_[dir][v] = Servo_ReadBack(v == 0 ? SERVO_PAN : SERVO_TILT, &written_[dir][v], &readOn_[dir][v], &readOff_[dir][v]);
    if (!readOk_[dir][v]) {
      anyFail = true;
    } else if (!Matches(dir, v)) {
      anyMismatch = true;
    }
  }
  if (anyFail) {
    readFail_[dir]++;
  }
  if (anyMismatch) {
    mismatch_[dir]++;
  }
}

// 読み戻しが、書いたはずの値（ON＝0・OFF＝パルス幅）と一致しているか
bool DebugScanBehavior::Matches(Dir dir, int servo) const {
  return readOn_[dir][servo] == 0 && readOff_[dir][servo] == written_[dir][servo];
}

// 1方向ぶんの読み戻しの結果を文字にする。一致なら「一致」、違えば書いた値と読んだ値を出す
void DebugScanBehavior::FormatReadBack(Dir dir, char *out, size_t size) const {
  static const char *const SERVO_NAMES[2] = {"左右", "上下"};
  size_t used = 0;
  out[0] = '\0';
  for (int v = 0; v < 2 && used < size; v++) {
    if (!readOk_[dir][v]) {
      used += snprintf(out + used, size - used, "%s読めず ", SERVO_NAMES[v]);
    } else if (!Matches(dir, v)) {
      used += snprintf(out + used, size - used, "%s不一致（書いた %u・読んだ ON %u OFF %u） ", SERVO_NAMES[v],
                       (unsigned)written_[dir][v], (unsigned)readOn_[dir][v], (unsigned)readOff_[dir][v]);
    }
  }
  if (used == 0) {
    snprintf(out, size, "一致（%u）", (unsigned)written_[dir][0]);
  }
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
  char back[DIR_COUNT][96];
  for (int d = 0; d < DIR_COUNT; d++) {
    FormatReadBack((Dir)d, back[d], sizeof(back[d]));
  }
  // 指示の直後のジャイロ。1件も読めていなければ「-」
  char kick[DIR_COUNT][12];
  for (int d = DIR_LEFT; d < DIR_COUNT; d++) {
    if (kickSamples_[d] > 0) {
      snprintf(kick[d], sizeof(kick[d]), "%.1f", kickDps_[d]);
    } else {
      snprintf(kick[d], sizeof(kick[d]), "-");
    }
  }
  // 動き出し以外の部分を整形して、「正面へ戻す」の動き出しが分かるまで待たせる（FlushPending で出す）
  snprintf(pendingRow_, sizeof(pendingRow_),
           "%d/%d 正面 %.1fcm（有効%d/%d）／左 %.1fcm（有効%d/%d）／右 %.1fcm（有効%d/%d） "
           "首 %d°/%d°/%d° かかった時間 %lu/%lu/%lums 指示の直後%dmsのジャイロ 左 %s／右 %sdps 上下の書き込み：%s",
           iteration_, DEBUG_SCAN_COUNT,
           cm_[DIR_FRONT], valid_[DIR_FRONT], WANDER_SCAN_SAMPLES,
           cm_[DIR_LEFT], valid_[DIR_LEFT], WANDER_SCAN_SAMPLES,
           cm_[DIR_RIGHT], valid_[DIR_RIGHT], WANDER_SCAN_SAMPLES,
           panDeg_[DIR_FRONT], panDeg_[DIR_LEFT], panDeg_[DIR_RIGHT],
           tookMs_[DIR_FRONT], tookMs_[DIR_LEFT], tookMs_[DIR_RIGHT],
           DEBUG_SCAN_GYRO_WINDOW_MS, kick[DIR_LEFT], kick[DIR_RIGHT], SkipText());
  snprintf(pendingBack_, sizeof(pendingBack_), "   読み戻し 正面：%s／左：%s／右：%s",
           back[DIR_FRONT], back[DIR_LEFT], back[DIR_RIGHT]);
  pending_ = true;
  inIteration_ = false;
  BeginIteration(nowMs);
}

void DebugScanBehavior::PrintSummary(const char *why) {
  FlushPending();   // 動き出しを見終わっていない行があれば、見えている分で出す
  int cuts = 0;
  for (int r = 0; r < CUT_COUNT; r++) {
    cuts += cutByReason_[r];
  }
  Log_Printf("見回し試験", "%s。順番：%s。上下のサーボへの書き込み：%s（省いた回数 %lu）。最後まで測れた回 %d／打ち切り %d（始めた回 %d／%d）", why,
             OrderText(rightFirst_, tiltMode_), SkipText(), (unsigned long)Servo_TiltWritesSkipped(),
             completed_, cuts, iteration_, DEBUG_SCAN_COUNT);
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
  Log_Printf("見回し試験", "動き出すまでの時間（指示から、ジャイロの絶対値が初めて %.1fdps【仮】以上になるまで。「-」は見ている間に超えなかった回）：",
             DEBUG_SCAN_ONSET_DPS);
  Log_Raw("| 指示 | 動き出した回 | 「-」の回 | 最小 ms | 最大 ms |");
  Log_Raw("|---|---|---|---|---|");
  for (int i = 0; i < 3; i++) {
    // 順番どおりに、何番目の指示が左右のどちらか
    bool left = (i == 0) ? !rightFirst_ : rightFirst_;
    const char *label = (i == 2) ? "正面へ戻す" : (i == 0 ? (left ? "1つ目（左）" : "1つ目（右）") : (left ? "2つ目（左）" : "2つ目（右）"));
    if (onsetN_[i] > 0) {
      Log_Raw("| %s | %d | %d | %d | %d |", label, onsetN_[i], onsetNone_[i], onsetMin_[i], onsetMax_[i]);
    } else {
      Log_Raw("| %s | 0 | %d | - | - |", label, onsetNone_[i]);
    }
  }
  if (tiltMode_) {
    if (tiltOnsetN_ > 0) {
      Log_Raw("| 上下（1つ目と同時。水平2軸の合成。しきい値 %.1fdps【仮】） | %d | %d | %d | %d |", DEBUG_SCAN_TILT_ONSET_DPS,
              tiltOnsetN_, tiltOnsetNone_, tiltOnsetMin_, tiltOnsetMax_);
    } else {
      Log_Raw("| 上下（1つ目と同時。水平2軸の合成。しきい値 %.1fdps【仮】） | 0 | %d | - | - |", DEBUG_SCAN_TILT_ONSET_DPS, tiltOnsetNone_);
    }
    if (tiltKickCount_ > 0) {
      Log_Printf("見回し試験", "上下の指示の直後%dmsの、水平2軸の合成の最大：最小 %.1f／最大 %.1fdps（%d回分）",
                 DEBUG_SCAN_GYRO_WINDOW_MS, tiltKickMin_, tiltKickMax_, tiltKickCount_);
    } else {
      Log_Printf("見回し試験", "上下の指示の直後%dmsの、水平2軸の合成の最大：取れた回なし（ジャイロが読めていない）", DEBUG_SCAN_GYRO_WINDOW_MS);
    }
  }
  Log_Printf("見回し試験", "サーボの読み戻し（測り終えた時点で PCA9685 から読んだ値と、書いた値の比較）：不一致 正面 %d／左 %d／右 %d回、読めず 正面 %d／左 %d／右 %d回",
             mismatch_[DIR_FRONT], mismatch_[DIR_LEFT], mismatch_[DIR_RIGHT],
             readFail_[DIR_FRONT], readFail_[DIR_LEFT], readFail_[DIR_RIGHT]);
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
  // 「正面へ戻す」の動き出しを見終えたら（見つかったか、見る時間が過ぎた）、待たせていた行を出す
  if (pending_ && (!onsetWatch || nowMs - onsetOpenMs_ >= (unsigned long)DEBUG_SCAN_ONSET_FRONT_MS + 50)) {
    FlushPending();
  }
  if (finishing_) {
    if (!pending_) {
      finishing_ = false;
      BeginIteration(nowMs);   // 最後の行が出たので、まとめを出して終わる
    }
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
      BeginSide(rightFirst_ ? DIR_RIGHT : DIR_LEFT, nowMs);
      return;

    case STATE_SCAN_LEFT:
    case STATE_SCAN_RIGHT: {
      Dir dir = (state_ == STATE_SCAN_LEFT) ? DIR_LEFT : DIR_RIGHT;
      Dir first = rightFirst_ ? DIR_RIGHT : DIR_LEFT;
      uint32_t beforeUs = micros();
      bool done = scan_.update(sensors, nowMs);
      if (!kickOpen && scan_.isGranted()) {
        // 首へ指示が出た（この update の中で使用権が取れた）。ここから動き出しと、DEBUG_SCAN_GYRO_WINDOW_MS の間のジャイロを見る
        kickStartUs = beforeUs;
        kickOpen = true;
        OpenMove(dir == first ? 0 : 1, beforeUs, nowMs);
      }
      if (!done) {
        return;
      }
      StoreScan(dir, nowMs);
      if (dir == first) {
        BeginSide(dir == DIR_LEFT ? DIR_RIGHT : DIR_LEFT, nowMs);
      } else {
        state_ = STATE_FACE_FRONT;
        stateStartMs_ = nowMs;
        frontInstructed_ = false;
      }
      return;
    }

    case STATE_FACE_FRONT:
      // うろうろと同じ：首を正面・水平に戻し、安定を待つ
      {
        uint32_t beforeUs = micros();
        if (!Neck_Request(NECK_OWNER_RANGE, SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, nowMs)) {
          return;
        }
        if (!frontInstructed_) {
          frontInstructed_ = true;
          OpenMove(2, beforeUs, nowMs);
        }
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
