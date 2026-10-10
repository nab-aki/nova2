#include "gyro.h"

#include "../config.h"
#include "../hal/hal_gyro.h"
#include "../hal/hal_log.h"
#include "motion.h"

static GyroState state = GYRO_PROBE;
static int failStreak = 0;             // 続けて失敗した回数
static unsigned long lastStepMs = 0;   // 前回、初期化を進めた／FIFO を読んだ時刻
static unsigned long stateSinceMs = 0;
static bool backlog = false;           // FIFO に読み残しがある（次の loop で続けて読む）

// 設定の読み戻し
static uint8_t whoAmI = 0;
static GyroHalSettings settings;
static bool settingsValid = false;

// 積算の時間の基準。件数 × 1/120 ではなく、内部周波数の補正値（INTERNAL_FREQ_FINE）から求めた実際の ODR を使う
static float odrHz = 120.0f;
static float sampleDtS = 1.0f / 120.0f;
static uint32_t sampleDtUs = 8333;

// 実測の ODR（FIFO を始めてからの件数 ÷ 経過時間）。積算には使わず、補正値と見比べるために出す
static unsigned long fifoStartMs = 0;
static uint32_t samplesSinceFifoStart = 0;

// ゼロ点と積算
static float zeroDps[3] = {0.0f, 0.0f, 0.0f};
static bool calibrated = false;
static double yawDeg = 0.0;
static float yawRateDps = 0.0f;
static float lastRawDps[3] = {0.0f, 0.0f, 0.0f};
static bool haveLast = false;

static GyroCounters counters;
static GyroSampleListener listener = NULL;

// 1回の読み取り（件数の確認＋たまった分の読み出し）で loop を止めた時間
static uint32_t readUsMax = 0;
static uint64_t readUsSum = 0;
static uint32_t readCount = 0;

// ------------------------ ゼロ点補正 ------------------------ //
static unsigned long calDiscardUntilMs = 0;
static int calCount = 0;
static double calSum[3];
static float calMin[3];
static float calMax[3];

static void CalBegin(unsigned long nowMs) {
  calDiscardUntilMs = nowMs + GYRO_CAL_DISCARD_MS;   // 始める前の約100ms は捨てる
  calCount = 0;
  for (int a = 0; a < 3; a++) {
    calSum[a] = 0.0;
    calMin[a] = 0.0f;
    calMax[a] = 0.0f;
  }
}

static void CalAdd(const GyroSample &s, unsigned long nowMs) {
  if ((long)(nowMs - calDiscardUntilMs) < 0) {
    return;
  }
  for (int a = 0; a < 3; a++) {
    float v = s.rawDps[a];
    calSum[a] += v;
    if (calCount == 0 || v < calMin[a]) calMin[a] = v;
    if (calCount == 0 || v > calMax[a]) calMax[a] = v;
  }
  if (++calCount < GYRO_CAL_SAMPLES) {
    return;
  }

  float mean[3];
  float spread[3];
  bool moved = false;
  for (int a = 0; a < 3; a++) {
    mean[a] = (float)(calSum[a] / calCount);
    spread[a] = calMax[a] - calMin[a];
    if (spread[a] > GYRO_CAL_MAX_SPREAD_DPS || fabsf(mean[a]) > GYRO_CAL_MAX_OFFSET_DPS) {
      moved = true;
    }
  }
  if (moved) {
    counters.calRetries++;
    Log_Printf("ジャイロ", "ゼロ点補正をやり直します（動いていた）：平均 X%+.2f Y%+.2f Z%+.2fdps／振れ幅 X%.2f Y%.2f Z%.2fdps"
               "（振れ幅 %.1fdps・平均 ±%.1fdps を超えるとやり直し）",
               mean[0], mean[1], mean[2], spread[0], spread[1], spread[2],
               GYRO_CAL_MAX_SPREAD_DPS, GYRO_CAL_MAX_OFFSET_DPS);
    CalBegin(nowMs);
    return;
  }

  Log_Printf("ジャイロ", "ゼロ点補正 完了（%d件）：補正前の平均 X%+.3f Y%+.3f Z%+.3fdps／振れ幅 X%.2f Y%.2f Z%.2fdps／"
             "前のゼロ点 X%+.3f Y%+.3f Z%+.3fdps%s",
             calCount, mean[0], mean[1], mean[2], spread[0], spread[1], spread[2],
             zeroDps[0], zeroDps[1], zeroDps[2], calibrated ? "" : "（未補正）");
  for (int a = 0; a < 3; a++) {
    zeroDps[a] = mean[a];
  }
  calibrated = true;
  counters.calDone++;
  yawDeg = 0.0;   // 向きの積算は、補正のたびに 0 から数え直す
  state = GYRO_RUNNING;
  stateSinceMs = nowMs;
}

// ------------------------ 回転ごとの記録 ------------------------ //
// 回転の開始（モーターを回し始めた）から、止めたあと車体が止まるまでを1件として積算する。
// FIFO の件には時刻を割り当ててあるので、開始より前・止めたあとの件を時刻で分ける。
struct TurnRecord {
  const char *name;
  unsigned long driveMs;   // モーターを回していた時間
  float driveDeg;          // 止めるまでに回った角度
  float afterDeg;          // 止めたあとに回った角度（行きすぎ）
  float peakDps;           // 最大の角速度（符号つき）
  long onsetMs;            // 回り始めるまでの時間（GYRO_TURN_ONSET_DPS を超えるまで。超えなければ負）
  uint32_t saturated;      // 頭打ちの件数
  uint32_t samples;
  bool calibrated;         // ゼロ点補正が済んでいたか
  const char *endNote;     // 記録の締め方
};

static bool turnRecording = true;
static bool turnActive = false;
static bool turnStopped = false;
static TurnRecord turn;
static uint32_t turnStartUs = 0;
static uint32_t turnStopUs = 0;
static unsigned long turnStartMs = 0;
static unsigned long turnStopMs = 0;
static double turnDriveDeg = 0.0;
static double turnAfterDeg = 0.0;
static long turnOnsetUs = -1;
static bool turnRestRunning = false;
static uint32_t turnRestStartUs = 0;

static TurnRecord turnLog[GYRO_TURN_LOG_COUNT];
static int turnLogHead = 0;    // 次に書く位置
static int turnLogCount = 0;

static void FormatTurn(const TurnRecord &r, char *out, size_t size) {
  char onset[24];
  if (r.onsetMs >= 0) {
    snprintf(onset, sizeof(onset), "%ldms", r.onsetMs);
  } else {
    snprintf(onset, sizeof(onset), "なし");
  }
  snprintf(out, size, "%s %lums：%+.1f°（止めるまで %+.1f°・止めたあと %+.1f°）最大 %+.0fdps 回り始め %s 頭打ち %lu件（%s%s）",
           r.name, r.driveMs, r.driveDeg + r.afterDeg, r.driveDeg, r.afterDeg, r.peakDps, onset,
           (unsigned long)r.saturated, r.endNote, r.calibrated ? "" : "・ゼロ点 未補正");
}

static void FinishTurn(const char *endNote) {
  if (!turnActive) {
    return;
  }
  turnActive = false;
  turn.driveDeg = (float)turnDriveDeg;
  turn.afterDeg = (float)turnAfterDeg;
  turn.onsetMs = (turnOnsetUs >= 0) ? (turnOnsetUs / 1000) : -1;
  turn.endNote = endNote;
  if (!turnStopped) {
    turn.driveMs = millis() - turnStartMs;
  }
  turnLog[turnLogHead] = turn;
  turnLogHead = (turnLogHead + 1) % GYRO_TURN_LOG_COUNT;
  if (turnLogCount < GYRO_TURN_LOG_COUNT) {
    turnLogCount++;
  }
  char line[256];
  FormatTurn(turn, line, sizeof(line));
  Log_Printf("ジャイロ", "%s", line);
}

static void TurnAdd(const GyroSample &s) {
  if (!turnActive) {
    return;
  }
  int32_t sinceStartUs = (int32_t)(s.timeUs - turnStartUs);
  if (sinceStartUs < 0) {
    return;   // 回し始める前の件（FIFO に残っていた分）
  }
  turn.samples++;
  if (s.saturated) {
    turn.saturated++;
  }
  if (fabsf(s.yawRateDps) > fabsf(turn.peakDps)) {
    turn.peakDps = s.yawRateDps;
  }
  double stepDeg = (double)s.yawRateDps * s.dtS;
  bool afterStop = turnStopped && (int32_t)(s.timeUs - turnStopUs) > 0;
  if (!afterStop) {
    turnDriveDeg += stepDeg;
    if (turnOnsetUs < 0 && fabsf(s.yawRateDps) >= GYRO_TURN_ONSET_DPS) {
      turnOnsetUs = sinceStartUs;
    }
    return;
  }
  turnAfterDeg += stepDeg;
  // 止めたあと、角速度が小さい状態が続いたら「止まった」として締める
  if (fabsf(s.yawRateDps) >= GYRO_TURN_REST_DPS) {
    turnRestRunning = false;
    return;
  }
  if (!turnRestRunning) {
    turnRestRunning = true;
    turnRestStartUs = s.timeUs;
    return;
  }
  if (s.timeUs - turnRestStartUs >= (uint32_t)GYRO_TURN_REST_MS * 1000UL) {
    FinishTurn("止まるまで測った");
  }
}

void Gyro_OnTurnStart(const char *name, unsigned long nowMs) {
  FinishTurn("次の回転が始まったので打ち切り");
  if (!turnRecording || !Gyro_IsReading()) {
    return;
  }
  turnActive = true;
  turnStopped = false;
  turn.name = name;
  turn.driveMs = 0;
  turn.peakDps = 0.0f;
  turn.saturated = 0;
  turn.samples = 0;
  turn.calibrated = calibrated;
  turnStartUs = micros();
  turnStartMs = nowMs;
  turnDriveDeg = 0.0;
  turnAfterDeg = 0.0;
  turnOnsetUs = -1;
  turnRestRunning = false;
}

void Gyro_OnTurnStop(unsigned long nowMs) {
  if (!turnActive || turnStopped) {
    return;
  }
  turnStopped = true;
  turnStopUs = micros();
  turnStopMs = nowMs;
  turn.driveMs = nowMs - turnStartMs;
}

void Gyro_SetTurnRecording(bool enabled) {
  turnRecording = enabled;
}

// 止めたあと、いつまでも締まらない記録を締める
static void UpdateTurnRecord(unsigned long nowMs) {
  if (!turnActive || !turnStopped) {
    return;
  }
  if (fabsf(Motion_GetTarget()) >= MOTOR_SPEED_EPSILON) {
    FinishTurn("次の動きが始まったので打ち切り");
  } else if (nowMs - turnStopMs >= GYRO_TURN_SETTLE_MAX_MS) {
    FinishTurn("止まりきらず打ち切り");
  }
}

// ------------------------ 失敗の扱い ------------------------ //

// 読み書きに失敗したときに呼ぶ。続けて GYRO_FAIL_LIMIT 回になったら、読むのをやめる
// （回転は時間ベースのままなので、車体の動きは変わらない）
static void NoteFail(const char *what, unsigned long nowMs) {
  if (++failStreak < GYRO_FAIL_LIMIT) {
    return;
  }
  bool probing = (state == GYRO_PROBE);
  state = probing ? GYRO_NOT_FOUND : GYRO_FAILED;
  stateSinceMs = nowMs;
  backlog = false;
  if (probing) {
    Log_Printf("ジャイロ", "応答がありません（%s が%d回続けて失敗）。ジャイロなしで動きます（z で確かめ直せます）",
               what, GYRO_FAIL_LIMIT);
  } else {
    counters.stops++;
    Log_Printf("ジャイロ", "%s が%d回続けて失敗したので、読み取りをやめます。回転は時間ベースのままです（z で再開を試せます）",
               what, GYRO_FAIL_LIMIT);
  }
  FinishTurn("ジャイロが止まったので打ち切り");
}

static void RestartInit(unsigned long nowMs) {
  state = GYRO_PROBE;
  stateSinceMs = nowMs;
  backlog = false;
  haveLast = false;
  settingsValid = false;
}

// ------------------------ 初期化（1回の呼び出しで1段ずつ）------------------------ //

static void StepProbe(unsigned long nowMs) {
  if (!GyroHal_ReadWhoAmI(&whoAmI)) {
    NoteFail("WHO_AM_I の読み取り", nowMs);
    return;
  }
  if (whoAmI != GYRO_WHO_AM_I_VALUE) {
    state = GYRO_NOT_FOUND;
    stateSinceMs = nowMs;
    Log_Printf("ジャイロ", "WHO_AM_I＝0x%02X（0x%02X ではありません）。ジャイロなしで動きます", whoAmI, GYRO_WHO_AM_I_VALUE);
    return;
  }
  Log_Printf("ジャイロ", "WHO_AM_I＝0x%02X（LSM6DSV16X）。リセットして設定します", whoAmI);
  if (!GyroHal_StartReset()) {
    NoteFail("リセットの書き込み", nowMs);
    return;
  }
  failStreak = 0;
  state = GYRO_RESET;
  stateSinceMs = nowMs;
}

static void StepReset(unsigned long nowMs) {
  bool done = false;
  if (!GyroHal_IsResetDone(&done)) {
    NoteFail("リセットの確認", nowMs);
    return;
  }
  if (!done) {
    if (nowMs - stateSinceMs >= GYRO_RESET_TIMEOUT_MS) {
      state = GYRO_FAILED;
      stateSinceMs = nowMs;
      counters.stops++;
      Log_Printf("ジャイロ", "リセットが%dms たっても終わりません。読み取りをやめます（z で再開を試せます）", GYRO_RESET_TIMEOUT_MS);
    }
    return;
  }
  if (!GyroHal_Configure()) {
    NoteFail("設定の書き込み", nowMs);
    if (state == GYRO_RESET) {
      RestartInit(nowMs);   // 途中まで書けたかもしれないので、最初からやり直す
    }
    return;
  }
  failStreak = 0;
  state = GYRO_SETTLE;   // ここから GYRO_SETTLE_MS は FIFO を始めない（立ち上がりの間のデータを入れない）
  stateSinceMs = nowMs;
}

static void StepSettle(unsigned long nowMs) {
  if (nowMs - stateSinceMs < GYRO_SETTLE_MS) {
    return;
  }
  if (!GyroHal_StartFifo() || !GyroHal_ReadSettings(&settings)) {
    NoteFail("FIFO の開始・設定の読み戻し", nowMs);
    if (state == GYRO_SETTLE) {
      RestartInit(nowMs);
    }
    return;
  }
  settingsValid = true;
  bool match = GyroHal_SettingsMatch(settings);
  Log_Printf("ジャイロ", "設定の読み戻し：IF_CFG=0x%02X CTRL1=0x%02X CTRL2=0x%02X CTRL3=0x%02X CTRL6=0x%02X "
             "FIFO_CTRL3=0x%02X FIFO_CTRL4=0x%02X → %s",
             settings.ifCfg, settings.ctrl1, settings.ctrl2, settings.ctrl3, settings.ctrl6,
             settings.fifoCtrl3, settings.fifoCtrl4,
             match ? "書いたとおり（±500dps・120Hz・ASF_CTRL=1・BDU=1・FIFO 連続）" : "書いた値と違います");
  if (!match) {
    state = GYRO_FAILED;
    stateSinceMs = nowMs;
    counters.stops++;
    Log_Printf("ジャイロ", "設定が合わないので、読み取りをやめます（z で再開を試せます）");
    return;
  }
  // 積算の時間の基準：内部周波数の補正値から求めた実際の ODR
  odrHz = GyroHal_ActualOdrHz(settings.freqFine);
  sampleDtS = 1.0f / odrHz;
  sampleDtUs = (uint32_t)(1000000.0f / odrHz + 0.5f);
  Log_Printf("ジャイロ", "ODR：補正値 FREQ_FINE=%d → %.2fHz（1件 %.3fms。積算はこの値を使います）",
             settings.freqFine, odrHz, sampleDtS * 1000.0f);
  failStreak = 0;
  fifoStartMs = nowMs;
  samplesSinceFifoStart = 0;
  haveLast = false;
  state = GYRO_WAIT_CAL;
  stateSinceMs = nowMs;
}

// ------------------------ FIFO の読み取り ------------------------ //

static void ProcessSample(const int16_t xyz[3], uint32_t timeUs, unsigned long nowMs) {
  GyroSample s;
  s.saturated = false;
  bool jumped = false;
  for (int a = 0; a < 3; a++) {
    s.rawDps[a] = (float)xyz[a] * GYRO_DPS_PER_LSB;
    if (abs((int)xyz[a]) >= GYRO_SATURATION_RAW) {
      s.saturated = true;
    }
    if (haveLast && fabsf(s.rawDps[a] - lastRawDps[a]) >= GYRO_JUMP_DPS) {
      jumped = true;
    }
    lastRawDps[a] = s.rawDps[a];
  }
  haveLast = true;
  s.yawRateDps = GYRO_YAW_SIGN * (s.rawDps[GYRO_YAW_AXIS] - zeroDps[GYRO_YAW_AXIS]);
  s.timeUs = timeUs;
  s.dtS = sampleDtS;

  counters.samples++;
  samplesSinceFifoStart++;
  if (s.saturated) counters.saturated++;
  if (jumped) counters.jumps++;

  yawRateDps = s.yawRateDps;
  yawDeg += (double)s.yawRateDps * sampleDtS;

  TurnAdd(s);
  if (listener != NULL) {
    listener(s);
  }
  if (state == GYRO_CALIBRATING) {
    CalAdd(s, nowMs);
  }
}

static void ReadFifo(unsigned long nowMs) {
  uint32_t beginUs = micros();
  uint16_t words = 0;
  bool overrun = false;
  if (!GyroHal_ReadFifoStatus(&words, &overrun)) {
    backlog = false;
    NoteFail("FIFO の読み取り", nowMs);
    return;
  }
  uint32_t statusUs = micros();   // いちばん新しい件は、ほぼこの時刻のもの
  bool failed = false;
  if (overrun) {
    counters.overruns++;
    Log_Printf("ジャイロ", "FIFO があふれました（読むのが間に合わず、古いデータが消えた。%u件たまっていた）", (unsigned)words);
  }

  uint16_t count = (words > GYRO_MAX_WORDS_PER_READ) ? GYRO_MAX_WORDS_PER_READ : words;
  backlog = words > count;
  for (uint16_t i = 0; i < count; i++) {
    uint8_t tag = 0;
    int16_t xyz[3];
    if (!GyroHal_ReadFifoWord(&tag, xyz)) {
      backlog = false;
      failed = true;
      NoteFail("FIFO の読み取り", nowMs);
      break;
    }
    if (tag != GYRO_FIFO_TAG_GYRO) {
      counters.otherTags++;
      continue;
    }
    // 古い件ほど前の時刻。たまっていた件数ぶん、1件の時間ずつさかのぼる
    uint32_t timeUs = statusUs - (uint32_t)(words - 1 - i) * sampleDtUs;
    ProcessSample(xyz, timeUs, nowMs);
    if (!Gyro_IsReading()) {
      break;
    }
  }

  // 続けて失敗した回数は、件数の確認から読み出しまでが1回とも失敗しなかったときだけ 0 に戻す
  // （件数の確認だけ成功して読み出しが失敗し続ける場合も、数え上がるように）
  if (!failed) {
    failStreak = 0;
  }

  uint32_t elapsedUs = micros() - beginUs;
  readCount++;
  readUsSum += elapsedUs;
  if (elapsedUs > readUsMax) {
    readUsMax = elapsedUs;
  }
}

// ------------------------ 公開関数 ------------------------ //

void Gyro_Setup(unsigned long nowMs) {
  memset(&counters, 0, sizeof(counters));
  failStreak = 0;
  calibrated = false;
  yawDeg = 0.0;
  turnActive = false;
  RestartInit(nowMs);
  lastStepMs = nowMs;
}

void Gyro_Update(unsigned long nowMs) {
  UpdateTurnRecord(nowMs);

  if (state == GYRO_NOT_FOUND || state == GYRO_FAILED) {
    return;
  }

  // ゼロ点補正は、車体が止まっている間だけ進める
  if (state == GYRO_WAIT_CAL && Motion_IsStill()) {
    state = GYRO_CALIBRATING;
    stateSinceMs = nowMs;
    CalBegin(nowMs);
  } else if (state == GYRO_CALIBRATING && !Motion_IsStill()) {
    state = GYRO_WAIT_CAL;
    stateSinceMs = nowMs;
    Log_Printf("ジャイロ", "車体が動き出したので、ゼロ点補正を中断します（止まったらやり直します）");
  }

  if (!backlog && nowMs - lastStepMs < GYRO_READ_INTERVAL_MS) {
    return;
  }
  lastStepMs = nowMs;

  switch (state) {
    case GYRO_PROBE:  StepProbe(nowMs);  break;
    case GYRO_RESET:  StepReset(nowMs);  break;
    case GYRO_SETTLE: StepSettle(nowMs); break;
    default:          ReadFifo(nowMs);   break;
  }
}

GyroState Gyro_GetState(void) {
  return state;
}

const char *Gyro_StateName(void) {
  switch (state) {
    case GYRO_NOT_FOUND:   return "未検出";
    case GYRO_PROBE:
    case GYRO_RESET:
    case GYRO_SETTLE:      return "設定中";
    case GYRO_WAIT_CAL:    return "補正待ち";
    case GYRO_CALIBRATING: return "補正中";
    case GYRO_RUNNING:     return "測定中";
    default:               return "停止（失敗）";
  }
}

bool Gyro_IsReading(void) {
  return state == GYRO_WAIT_CAL || state == GYRO_CALIBRATING || state == GYRO_RUNNING;
}

bool Gyro_IsCalibrated(void) {
  return calibrated;
}

float Gyro_YawRateDps(void) {
  return yawRateDps;
}

float Gyro_YawDeg(void) {
  return (float)yawDeg;
}

float Gyro_ZeroDps(int axis) {
  return zeroDps[axis];
}

float Gyro_OdrHz(void) {
  return odrHz;
}

const GyroCounters &Gyro_GetCounters(void) {
  return counters;
}

void Gyro_RequestCalibration(unsigned long nowMs) {
  if (state == GYRO_NOT_FOUND || state == GYRO_FAILED) {
    Log_Printf("ジャイロ", "初期化からやり直します（いまの状態：%s）", Gyro_StateName());
    failStreak = 0;
    RestartInit(nowMs);
    return;
  }
  if (!Gyro_IsReading()) {
    Log_Printf("ジャイロ", "設定中です。少し待ってから押してください");
    return;
  }
  Log_Printf("ジャイロ", "ゼロ点補正をやり直します（車体を動かさないでください。約2秒）");
  state = GYRO_WAIT_CAL;
  stateSinceMs = nowMs;
}

void Gyro_SetListener(GyroSampleListener fn) {
  listener = fn;
}

// 実測の ODR（FIFO を始めてからの件数 ÷ 経過時間）。10秒たつまでは求めない
static bool MeasuredOdrHz(float *hz) {
  if (!Gyro_IsReading()) {
    return false;
  }
  unsigned long elapsedMs = millis() - fifoStartMs;
  if (elapsedMs < 10000 || samplesSinceFifoStart == 0) {
    return false;
  }
  *hz = (float)((double)samplesSinceFifoStart * 1000.0 / (double)elapsedMs);
  return true;
}

void Gyro_Print(void) {
  char measured[32];
  float hz = 0.0f;
  if (MeasuredOdrHz(&hz)) {
    snprintf(measured, sizeof(measured), "%.2fHz", hz);
  } else {
    snprintf(measured, sizeof(measured), "-");
  }
  Log_Printf("ジャイロ", "状態:%s WHO_AM_I:0x%02X 設定:%s ODR 補正値 %.2fHz／実測 %s（あふれ・失敗があると実測は下がる）",
             Gyro_StateName(), whoAmI,
             !settingsValid ? "未設定" : (GyroHal_SettingsMatch(settings) ? "±500dps・120Hz（読み戻し一致）" : "読み戻しが不一致"),
             odrHz, measured);
  Log_Printf("ジャイロ", "ゼロ点 X%+.3f Y%+.3f Z%+.3fdps（%s。補正 %lu回・やり直し %lu回） 件数 %lu FIFOあふれ %lu 頭打ち %lu 飛び %lu "
             "ほかのタグ %lu 読み取り停止 %lu回",
             zeroDps[0], zeroDps[1], zeroDps[2], calibrated ? "補正済み" : "未補正",
             (unsigned long)counters.calDone, (unsigned long)counters.calRetries,
             (unsigned long)counters.samples, (unsigned long)counters.overruns,
             (unsigned long)counters.saturated, (unsigned long)counters.jumps,
             (unsigned long)counters.otherTags, (unsigned long)counters.stops);
  if (readCount > 0) {
    Log_Printf("ジャイロ", "1回の読み取りで loop を止めた時間：平均 %.2fms・最大 %.2fms（%lu回。%dms ごと）",
               (float)((double)readUsSum / readCount / 1000.0), readUsMax / 1000.0f,
               (unsigned long)readCount, GYRO_READ_INTERVAL_MS);
  }
  if (turnLogCount == 0) {
    Log_Printf("ジャイロ", "回転の記録：まだありません");
    return;
  }
  Log_Printf("ジャイロ", "回転の記録（直近%d件。新しい順。正＝上から見て反時計回り）：", turnLogCount);
  for (int i = 0; i < turnLogCount; i++) {
    int idx = (turnLogHead - 1 - i + 2 * GYRO_TURN_LOG_COUNT) % GYRO_TURN_LOG_COUNT;
    char line[256];
    FormatTurn(turnLog[idx], line, sizeof(line));
    Log_Printf("ジャイロ", "  %d) %s", i + 1, line);
  }
}

void Gyro_PrintNow(void) {
  Log_Printf("ジャイロ", "状態:%s いまの値（補正前）X%+.2f Y%+.2f Z%+.2fdps／車体の回る速さ %+.2fdps／向き %+.1f°（%s）",
             Gyro_StateName(), lastRawDps[0], lastRawDps[1], lastRawDps[2], yawRateDps, (float)yawDeg,
             calibrated ? "補正済み" : "ゼロ点 未補正");
  Gyro_Print();
}
