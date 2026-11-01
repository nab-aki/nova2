#include "debug_arc.h"

#include "../core/debug_pause.h"
#include "../core/gyro.h"
#include "../core/motion.h"
#include "../core/neck.h"
#include "../core/obstacle.h"
#include "../core/safety.h"
#include "../core/tof_view.h"
#include "../hal/hal_log.h"
#include "../hal/hal_motor.h"

// 結果の記録（RAM。電源を切る・リセットで消える）
struct ArcRecord {
  uint32_t uptimeSec;    // 測定が終わったときの、起動からの秒（リセットされていないかの確認用）
  int8_t dir;            // 1＝左、-1＝右、0＝直進の基準
  int16_t leftPwm;
  int16_t rightPwm;
  uint8_t end;           // DebugArcBehavior::End
  bool hasAverage;
  float averageDps;      // 曲がる向きを正とした、集計の窓の平均
  float maxDps;
  float minDps;
  uint16_t windowMs;
  float totalDeg;        // 差動の前進の間に回った角度（曲がる向きを正）
  uint16_t runMs;        // 差動の前進が続いた時間
  float batteryStartV;
  float batteryEndV;
  float angle[ARC_ANGLE_COUNT];   // 差動の前進の始めから ARC_ANGLE_LIST_MS の時点までに向きを変えた角度（曲がる向きが正）
  uint8_t angleValid;    // angle のうち、その時点まで走っていたものの印（ビット）
  uint8_t speedCount;
  int16_t speedAtMs[ARC_SPEED_MAX];     // 差動の前進の始まりからの時刻（負＝直進の間）
  int16_t speedMmS[ARC_SPEED_MAX];      // 前に進む速さ（mm/秒）。INT16_MIN＝測れず
  int16_t startSpeedMmS;                // 差動の前進を始めた時点の速さ。INT16_MIN＝なし
  uint8_t rateCount;     // 下の 0.1 秒刻みの数
  uint8_t skipSteps;     // そのうち、集計から外した最初の刻みの数
  float rates[ARC_MAX_STEPS];
};

static ArcRecord records[ARC_RESULT_COUNT];
static int recordCount = 0;        // 残っている件数（最大 ARC_RESULT_COUNT）
static uint32_t recordTotal = 0;   // 起動からの通し番号
static int recordHead = 0;         // 次に書く場所

static const int OUTER_PWMS[ARC_OUTER_PWM_COUNT] = ARC_OUTER_PWM_LIST;
static const int ANGLE_AT_MS[ARC_ANGLE_COUNT] = ARC_ANGLE_LIST_MS;
#define SPEED_NONE INT16_MIN

static const char *EndName(uint8_t end) {
  switch (end) {
    case 0:  return "最後まで走った";
    case 1:  return "途中で止まった（ToF で止める判断）";
    case 2:  return "途中で止まった（持ち上げなど、ほかの理由）";
    default: return "途中で止めた（キー・リモコン）";
  }
}

static void PrintRecord(const ArcRecord &r, uint32_t number) {
  const char *kind = r.dir == 0 ? "直進の基準" : (r.dir > 0 ? "左へ曲がる" : "右へ曲がる");
  char average[96];
  if (r.hasAverage) {
    snprintf(average, sizeof(average), "平均 %.1f°/秒（窓 %dms・最大 %.1f・最小 %.1f）",
             r.averageDps, r.windowMs, r.maxDps, r.minDps);
  } else {
    snprintf(average, sizeof(average), "平均なし（集計の時間が %dms 未満）", ARC_PARTIAL_MIN_MS);
  }
  const char *note = (r.end != 0 && r.hasAverage) ? "【途中で止まった回のそれまでの平均。参考】" : "";
  Log_Printf("測定", "結果 #%lu %s 左PWM%d・右PWM%d：%s %s／回った角度 %.1f°（差動 %dms）／電池 %.2f→%.2fV／%s／起動から %lu秒",
             (unsigned long)number, kind, r.leftPwm, r.rightPwm, average, note, r.totalDeg, r.runMs,
             r.batteryStartV, r.batteryEndV, EndName(r.end), (unsigned long)r.uptimeSec);
  // 0.1 秒ごとの曲がる速さ。最初の ARC_SKIP_MS は集計外（| の左）
  char line[300];
  int n = 0;
  line[0] = '\0';
  for (int k = 0; k < r.rateCount && n < (int)sizeof(line) - 12; k++) {
    if (k == r.skipSteps) {
      n += snprintf(line + n, sizeof(line) - n, "| ");
    }
    n += snprintf(line + n, sizeof(line) - n, "%.0f ", r.rates[k]);
  }
  Log_Printf("測定", "  0.1秒ごと（°/秒。曲がる向きが正。| の左は集計外）：%s", line);

  // 差動の前進の始めから 0.3/0.5/1.0 秒の時点で、何度向きを変えたか
  char angles[100];
  int a = 0;
  angles[0] = '\0';
  for (int i = 0; i < ARC_ANGLE_COUNT && a < (int)sizeof(angles) - 24; i++) {
    if (r.angleValid & (1 << i)) {
      a += snprintf(angles + a, sizeof(angles) - a, "%.1f秒 %.1f°　", ANGLE_AT_MS[i] / 1000.0f, r.angle[i]);
    } else {
      a += snprintf(angles + a, sizeof(angles) - a, "%.1f秒 なし　", ANGLE_AT_MS[i] / 1000.0f);
    }
  }
  Log_Printf("測定", "  差動の前進の始めから向きを変えた角度（曲がる向きが正）：%s", angles);

  // 前に進む速さ（ToF 中央の列 c3〜c4 の距離の減り方。| から右が差動の前進）
  char speeds[300];
  int sp = 0;
  speeds[0] = '\0';
  bool bar = false;
  for (int k = 0; k < r.speedCount && sp < (int)sizeof(speeds) - 20; k++) {
    if (!bar && r.speedAtMs[k] >= 0) {
      sp += snprintf(speeds + sp, sizeof(speeds) - sp, "| ");
      bar = true;
    }
    if (r.speedMmS[k] == SPEED_NONE) {
      sp += snprintf(speeds + sp, sizeof(speeds) - sp, "%+.1f:- ", r.speedAtMs[k] / 1000.0f);
    } else {
      sp += snprintf(speeds + sp, sizeof(speeds) - sp, "%+.1f:%d ", r.speedAtMs[k] / 1000.0f, r.speedMmS[k]);
    }
  }
  if (r.speedCount == 0) {
    Log_Printf("測定", "  前に進む速さ：記録なし（ToF の中央の列が測れていない）");
  } else {
    Log_Printf("測定", "  前に進む速さ（mm/秒。秒:速さ。秒は差動の前進の始まりから。| から右が差動の前進）：%s", speeds);
  }
  if (r.startSpeedMmS == SPEED_NONE) {
    Log_Printf("測定", "  差動の前進を始めた時点の速さ：測れず");
  } else {
    Log_Printf("測定", "  差動の前進を始めた時点の速さ：%d mm/秒", r.startSpeedMmS);
  }
}

int DebugArcBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
  return isBusy() ? PRIORITY_DEBUG_ARC : 0;
}

// ------------------------------ 準備（i・k・l） ------------------------------ //

void DebugArcBehavior::printSetting(const char *prefix) const {
  if (outerIndex_ == 0) {
    Log_Printf("測定", "%s直進の基準（左右とも PWM%d。向きは使わない）。外側の値は l で次へ（基準→%d→%d→%d→%d）",
               prefix, Motor_SpeedToPwm(CRUISE_SPEED), OUTER_PWMS[0], OUTER_PWMS[1], OUTER_PWMS[2], OUTER_PWMS[3]);
  } else {
    int outer = OUTER_PWMS[outerIndex_ - 1];
    Log_Printf("測定", "%s%sへ曲がる：内側 PWM%d・外側 PWM%d（左PWM%d・右PWM%d）。外側の値は l で次へ（基準→%d→%d→%d→%d）",
               prefix, dir_ > 0 ? "左" : "右", ARC_INNER_PWM, outer,
               dir_ > 0 ? ARC_INNER_PWM : outer, dir_ > 0 ? outer : ARC_INNER_PWM,
               OUTER_PWMS[0], OUTER_PWMS[1], OUTER_PWMS[2], OUTER_PWMS[3]);
  }
  Log_Printf("測定", "  差動の前進 %.1f秒（直進 %.1f秒のあと）。準備ができました。ケーブルを抜き、前 1.5m 以上の先に壁（速さの記録用）・曲がる向きと反対側の壁から 15〜20cm にして、リモコンの ▶ で始めます",
             (outerIndex_ == 0 ? ARC_BASELINE_RUN_MS : ARC_RUN_MS) / 1000.0f, ARC_LEAD_MS / 1000.0f);
}

void DebugArcBehavior::selectLeft() {
  dir_ = 1;
  prepared_ = true;
  printSetting("設定：");
}

void DebugArcBehavior::selectRight() {
  dir_ = -1;
  prepared_ = true;
  printSetting("設定：");
}

void DebugArcBehavior::nextOuter() {
  outerIndex_ = (outerIndex_ + 1) % (ARC_OUTER_PWM_COUNT + 1);
  prepared_ = true;
  printSetting("設定：");
}

void DebugArcBehavior::request() {
  requested_ = true;
}

// ------------------------------ 測定 ------------------------------ //

void DebugArcBehavior::begin(const SensorData &sensors, unsigned long nowMs) {
  requested_ = false;
  prepared_ = false;   // 準備は1回ぶん。次の測定は i・k・l でもう一度準備する
  runDir_ = dir_;
  runOuterIndex_ = outerIndex_;
  if (runOuterIndex_ == 0) {
    runLeftPwm_ = runRightPwm_ = Motor_SpeedToPwm(CRUISE_SPEED);
  } else {
    int outer = OUTER_PWMS[runOuterIndex_ - 1];
    runLeftPwm_ = runDir_ > 0 ? ARC_INNER_PWM : outer;
    runRightPwm_ = runDir_ > 0 ? outer : ARC_INNER_PWM;
  }
  batteryStartV_ = sensors.batteryV;
  stepCount_ = 0;
  phase_ = PHASE_PREP;
  phaseStartMs_ = nowMs;
  prepStartMs_ = nowMs;
  Log_Printf("測定", "曲がり測定を始めます：%s／左PWM%d・右PWM%d。首を走行姿勢にして、前が空いているか確かめます（電池 %.2fV）",
             runOuterIndex_ == 0 ? "直進の基準" : (runDir_ > 0 ? "左へ曲がる" : "右へ曲がる"),
             runLeftPwm_, runRightPwm_, batteryStartV_);
}

void DebugArcBehavior::onStart(unsigned long nowMs) {
  begin(Sensors_Get(), nowMs);
}

// 新しい1枚があれば、前に進む速さ（中央の列の距離の縮み ÷ 時間）を足す
void DebugArcBehavior::sampleSpeed() {
  uint32_t readyMs;
  int centerMm;
  while (TofView_TakeFrameCenter(&speedSeq_, &readyMs, &centerMm)) {
    if (speedPrevValid_ && centerMm >= 0 && readyMs > speedPrevMs_ && speedCount_ < ARC_SPEED_MAX) {
      speedMs_[speedCount_] = (speedPrevMs_ + readyMs) / 2;
      speedMmS_[speedCount_] = (int16_t)((speedPrevMm_ - centerMm) * 1000L / (long)(readyMs - speedPrevMs_));
      speedCount_++;
    } else if (speedPrevValid_ && speedCount_ < ARC_SPEED_MAX) {
      speedMs_[speedCount_] = (speedPrevMs_ + readyMs) / 2;
      speedMmS_[speedCount_] = SPEED_NONE;
      speedCount_++;
    }
    speedPrevValid_ = centerMm >= 0;
    speedPrevMs_ = readyMs;
    speedPrevMm_ = centerMm;
  }
}

void DebugArcBehavior::startArc(unsigned long nowMs) {
  Motion_StartArc(runLeftPwm_, runRightPwm_, nowMs);
  arcStartMs_ = nowMs;
  stepMs_[0] = nowMs;
  stepYaw_[0] = Gyro_YawDeg();
  stepCount_ = 1;
  lastActiveMs_ = nowMs;
  lastActiveYaw_ = stepYaw_[0];
  phase_ = PHASE_ARC;
  phaseStartMs_ = nowMs;
}

// 差動の前進の記録から、1回ぶんの結果を作って残し、出す
void DebugArcBehavior::finishRecord(End end, const SensorData &sensors, unsigned long nowMs) {
  ArcRecord &r = records[recordHead];
  memset(&r, 0, sizeof(r));
  r.uptimeSec = nowMs / 1000;
  r.dir = runOuterIndex_ == 0 ? 0 : (int8_t)runDir_;
  r.leftPwm = runLeftPwm_;
  r.rightPwm = runRightPwm_;
  r.end = end;
  r.batteryStartV = batteryStartV_;
  r.batteryEndV = sensors.batteryV;
  float sign = runDir_ > 0 ? 1.0f : -1.0f;   // 曲がる向きを正にする（ジャイロは反時計回りが正）

  // 集計の窓の始まり：差動の前進の始まりから ARC_SKIP_MS 以降の最初の記録
  int startIndex = -1;
  for (int k = 0; k < stepCount_; k++) {
    if (stepMs_[k] - arcStartMs_ >= ARC_SKIP_MS) {
      startIndex = k;
      break;
    }
  }

  // 0.1 秒刻み
  r.rateCount = 0;
  for (int k = 1; k < stepCount_ && r.rateCount < ARC_MAX_STEPS; k++) {
    float dt = (stepMs_[k] - stepMs_[k - 1]) / 1000.0f;
    r.rates[r.rateCount++] = dt > 0.0f ? sign * (stepYaw_[k] - stepYaw_[k - 1]) / dt : 0.0f;
  }
  r.skipSteps = startIndex >= 0 ? (uint8_t)startIndex : r.rateCount;

  // 集計：最初の ARC_SKIP_MS を除いた、差動の前進の最後まで（途中で止まったら、まだ続いていた最後の loop まで）
  unsigned long endMs = lastActiveMs_;
  float endYaw = lastActiveYaw_;
  r.runMs = (uint16_t)(endMs - arcStartMs_);
  r.totalDeg = sign * (endYaw - stepYaw_[0]);
  if (startIndex >= 0 && endMs > stepMs_[startIndex]) {
    unsigned long windowMs = endMs - stepMs_[startIndex];
    if (end == END_COMPLETE || windowMs >= ARC_PARTIAL_MIN_MS) {
      r.hasAverage = true;
      r.windowMs = (uint16_t)windowMs;
      r.averageDps = sign * (endYaw - stepYaw_[startIndex]) / (windowMs / 1000.0f);
      bool first = true;
      for (int k = startIndex + 1; k < stepCount_; k++) {
        float rate = r.rates[k - 1];
        if (first || rate > r.maxDps) r.maxDps = rate;
        if (first || rate < r.minDps) r.minDps = rate;
        first = false;
      }
    }
  }

  // 差動の前進の始めから ARC_ANGLE_LIST_MS の時点までの角度（その時点以降の最初の記録。そこまで走っていなければ「なし」）
  for (int i = 0; i < ARC_ANGLE_COUNT; i++) {
    for (int k = 1; k < stepCount_; k++) {
      if (stepMs_[k] - arcStartMs_ >= (unsigned long)ANGLE_AT_MS[i]) {
        r.angle[i] = sign * (stepYaw_[k] - stepYaw_[0]);
        r.angleValid |= (1 << i);
        break;
      }
    }
  }

  // 前に進む速さ。差動の前進を始めた時点に最も近い刻みを「始めた時点の速さ」とする
  r.startSpeedMmS = SPEED_NONE;
  long bestGap = 1000000L;
  for (int k = 0; k < speedCount_; k++) {
    long rel = (long)speedMs_[k] - (long)arcStartMs_;
    r.speedAtMs[r.speedCount] = (int16_t)rel;
    r.speedMmS[r.speedCount] = speedMmS_[k];
    r.speedCount++;
    if (speedMmS_[k] != SPEED_NONE && labs(rel) < bestGap) {
      bestGap = labs(rel);
      r.startSpeedMmS = speedMmS_[k];
    }
  }

  recordHead = (recordHead + 1) % ARC_RESULT_COUNT;
  if (recordCount < ARC_RESULT_COUNT) {
    recordCount++;
  }
  recordTotal++;
  PrintRecord(r, recordTotal);
}

void DebugArcBehavior::finish(const char *reason) {
  phase_ = PHASE_IDLE;
  requested_ = false;
  Neck_Release(NECK_OWNER_RANGE);
  Log_Printf("測定", "曲がり測定を終えました（%s）。次は i・k・l で準備してください（結果は t でも見られます）", reason);
}

void DebugArcBehavior::abort(const char *reason, unsigned long nowMs) {
  if (!isBusy()) {
    return;
  }
  if (phase_ == PHASE_ARC) {
    Motion_EmergencyStop();
    Log_Printf("測定", "中断：%s", reason);
    finishRecord(END_ABORT, Sensors_Get(), nowMs);
    phase_ = PHASE_STOPPING;
    phaseStartMs_ = nowMs;
    return;
  }
  if (phase_ == PHASE_LEAD) {
    Motion_EmergencyStop();
  }
  Log_Printf("測定", "中断：%s", reason);
  finish("中断。結果はありません");
}

void DebugArcBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  // 調停から外された。差動の前進が残っていれば、必ず止める
  if (Motion_IsArcing() || phase_ == PHASE_LEAD || phase_ == PHASE_ARC) {
    Motion_EmergencyStop();
  }
  phase_ = PHASE_IDLE;
  requested_ = false;
}

void DebugArcBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  if (phase_ == PHASE_IDLE) {
    if (requested_) {
      begin(sensors, nowMs);   // 終わってから調停が手放すまでの間に、次の予約が入った場合
    }
    return;
  }
  if (phase_ != PHASE_STOPPING) {
    if (!Pause_IsPaused()) {
      abort("一時停止が解除された", nowMs);
      return;
    }
    if (!Gyro_IsTurnUsable()) {
      abort("ジャイロが使えない（補正済みで測定中のときだけ測れる）", nowMs);
      return;
    }
  }

  if (phase_ == PHASE_LEAD || phase_ == PHASE_ARC) {
    sampleSpeed();
  }
  unsigned long elapsed = nowMs - phaseStartMs_;
  switch (phase_) {
    case PHASE_PREP: {
      if (Safety_IsLifted()) {
        abort(Safety_IsTofStopped() ? "ToF が使えず止めている" : "持ち上げられている（または床を読めない）", nowMs);
        return;
      }
      if (nowMs - prepStartMs_ >= ARC_PREP_TIMEOUT_MS) {
        abort("首の姿勢か判定が決まらない（待ちの上限）", nowMs);
        return;
      }
      if (!Neck_Request(NECK_OWNER_RANGE, SERVO1_DRIVE_DEG, SERVO2_LEVEL_DEG, nowMs)) {
        return;
      }
      if (!Neck_IsDrivePose() || !Neck_IsSteady(nowMs) || !Obstacle_IsReady() ||
          nowMs - prepStartMs_ < ARC_START_DELAY_MS) {
        return;
      }
      // 開始条件：前が空いている（ID25 が歩き出すときと同じ。ToF の判定が「なし」で、通路が WANDER_WALK_MIN_MM 以上）
      bool clear = !Obstacle_IsBlocked();
      float frontCm = Obstacle_LastCm();
#if TOF_ENABLE
      TofViewResult view;
      TofView_EvaluateFront(WANDER_WALK_MIN_MM, &view);
      clear = clear && (view.state == TOF_VIEW_CLEAR || view.state == TOF_VIEW_FAR_CLEAR);
      frontCm = view.cm;
#endif
      if (!clear) {
        Log_Printf("測定", "前が空いていないので始めません（通路の最も近い距離 %.1fcm。%dmm 以上が必要）",
                   frontCm, WANDER_WALK_MIN_MM);
        abort("前が空いていない", nowMs);
        return;
      }
      Neck_Release(NECK_OWNER_RANGE);   // 走るあいだの首は安全層が走行姿勢に保つ
      batteryStartV_ = sensors.batteryV;
      speedSeq_ = 0;
      speedPrevValid_ = false;
      speedCount_ = 0;
      sampleSpeed();   // 止まっているうちの最新の1枚を、速さの基準にする
      Motion_SetSpeed(CRUISE_SPEED, ARC_LEAD_ACCEL_MS, nowMs);
      phase_ = PHASE_LEAD;
      phaseStartMs_ = nowMs;
      Log_Printf("測定", "前が空いています（通路 %.1fcm）。直進 %dms のあと差動の前進 %dms", frontCm, ARC_LEAD_MS,
                 runOuterIndex_ == 0 ? ARC_BASELINE_RUN_MS : ARC_RUN_MS);
      return;
    }

    case PHASE_LEAD:
      if (Motion_GetTarget() < MOTOR_SPEED_EPSILON) {
        // 安全層が止めた（ToF・持ち上げ）。差動の前進に入る前なので、結果はない
        Log_Printf("測定", "直進の間に止まりました（%s）", Safety_IsLifted() ? "持ち上げなど" : "ToF で止める判断");
        finish("差動の前進の前に止まった。結果はありません");
        return;
      }
      if (elapsed >= ARC_LEAD_MS) {
        startArc(nowMs);
      }
      return;

    case PHASE_ARC: {
      if (!Motion_IsArcing()) {
        // 安全層が止めた。まだ続いていた最後の loop までを集計する
        End end = Safety_IsLifted() ? END_OTHER
                                    : ((Obstacle_IsBlocked() || Obstacle_IsEmergency()) ? END_TOF : END_OTHER);
        finishRecord(end, sensors, nowMs);
        phase_ = PHASE_STOPPING;
        phaseStartMs_ = nowMs;
        return;
      }
      lastActiveMs_ = nowMs;
      lastActiveYaw_ = Gyro_YawDeg();
      while (stepCount_ < ARC_MAX_STEPS && nowMs - arcStartMs_ >= (unsigned long)stepCount_ * ARC_STEP_MS) {
        stepMs_[stepCount_] = nowMs;
        stepYaw_[stepCount_] = lastActiveYaw_;
        stepCount_++;
      }
      unsigned long runMs = runOuterIndex_ == 0 ? ARC_BASELINE_RUN_MS : ARC_RUN_MS;
      if (nowMs - arcStartMs_ >= runMs) {
        Motion_SetSpeed(0.0f, MOTION_DECEL_MS, nowMs);   // いつもの減速で止まる（差動の前進はここで取り消される）
        finishRecord(END_COMPLETE, sensors, nowMs);
        phase_ = PHASE_STOPPING;
        phaseStartMs_ = nowMs;
      }
      return;
    }

    case PHASE_STOPPING:
      if (Motion_IsStill() || elapsed >= MOTION_DECEL_MS + 1000) {
        finish("止まりました");
      }
      return;

    default:
      return;
  }
}

// ------------------------------ t キー ------------------------------ //

void DebugArcBehavior::PrintResults() {
  if (recordCount == 0) {
    Log_Printf("測定", "曲がり測定の結果はありません（電源を切る・リセットで消えます。いま起動から %lu秒）",
               (unsigned long)(millis() / 1000));
    return;
  }
  Log_Printf("測定", "曲がり測定の結果（直近 %d回。新しい順。いま起動から %lu秒。リセットされていれば、ここは空です）：",
             recordCount, (unsigned long)(millis() / 1000));
  for (int i = 0; i < recordCount; i++) {
    int index = (recordHead - 1 - i + ARC_RESULT_COUNT) % ARC_RESULT_COUNT;
    PrintRecord(records[index], recordTotal - i);
  }
}
