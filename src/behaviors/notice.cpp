#include "notice.h"

#include "../config.h"
#include "../core/arbiter.h"
#include "../core/eyes.h"
#include "../core/motion.h"
#include "../core/obstacle.h"
#include "../core/test_stats.h"
#include "../core/tof_view.h"
#include "../hal/hal_log.h"
#include "steer_avoid.h"

// ------------------------ 車体 ------------------------ //

// 停止ごとの記録（t で出す。起動から。電源を切ると消える）
struct NoticeStopRecord {
  uint32_t timeMs;            // 気づいた時刻（起動から）
  const char *before;         // 直前まで動いていた車体の振る舞い
  char phase;                 // 気づいたときの速度の変わり方：'+' 加速中・'=' 一定・'-' 減速中
  bool arcing;                // 差動の前進（ID27 で曲がっている途中）だったか
  float speed;                // 気づいたときの速度（正規化）
  int32_t sinceSteerMs;       // ID27 が曲がり終えてからの時間。曲がり終えた記録がなければ負
  float noticedCm;            // 気づいたときの距離（通路の中の最も近い物）
  bool emergency;             // 気づいたとき、非常停止の距離より近かったか
  bool usBlocked;             // 気づいたとき、超音波も「あり」だったか
  int16_t lastFwdMm;          // 気づく前に、前進中に判定した最後の1枚の通路の最も近い距離。なければ負
  int32_t lastFwdAgeMs;       // その枚が、気づいたときから何 ms 前か。枚がなければ負
  uint16_t skippedFrames;     // その枚から気づくまでに捨てた枚
  uint16_t stillFrames;       // 同じく、前進中でなく判定した枚
  float restCm;               // 止まったあとの距離
  int16_t gapMm;              // 止まったあとの前端の隙間。通路の中になければ負
};

static NoticeStopRecord stopRecords[NOTICE_STOP_RECORDS];
static int stopRecordHead = 0;    // 次に書く場所
static int stopRecordTotal = 0;   // 起動からの数
static bool stopMinValid = false;
static NoticeStopRecord stopMin;  // 前端の隙間が最小の回
static NoticeStopRecord pendingStop;   // 気づいたときに取り、止まったあとに足す

static void PrintStopRecord(const char *head, const NoticeStopRecord &r) {
  char steer[32];
  if (r.sinceSteerMs >= 0) {
    snprintf(steer, sizeof(steer), "ID27 が終わって %ldms", (long)r.sinceSteerMs);
  } else {
    snprintf(steer, sizeof(steer), "ID27 の終わりの記録なし");
  }
  char fwd[80];
  if (r.lastFwdAgeMs >= 0) {
    snprintf(fwd, sizeof(fwd), "%dmm（%ldms 前。間に捨てた %u枚・止まって判定 %u枚）",
             r.lastFwdMm, (long)r.lastFwdAgeMs, r.skippedFrames, r.stillFrames);
  } else {
    snprintf(fwd, sizeof(fwd), "なし");
  }
  Log_Printf("気づく", "%s %lums 直前 %s（%s・速度%.2f%s）%s／気づいた %.1fcm（非常停止の距離 %s・超音波 %s）→ 止まった %.1fcm 前端の隙間 %dmm／最後の前進中の枚 %s",
             head, (unsigned long)r.timeMs, r.before,
             r.phase == '+' ? "加速中" : (r.phase == '-' ? "減速中" : "一定"), r.speed, r.arcing ? "・曲がっている途中" : "",
             steer, r.noticedCm, r.emergency ? "より近い" : "より遠い", r.usBlocked ? "あり" : "なし",
             r.restCm, r.gapMm, fwd);
}

void NoticeBehavior::PrintRecords() {
  int count = min(stopRecordTotal, NOTICE_STOP_RECORDS);
  Log_Printf("気づく", "停止ごとの記録（起動から %d回。直近 %d回まで。新しい順。数えた停止だけ）：", stopRecordTotal, NOTICE_STOP_RECORDS);
  char head[8];
  for (int i = 0; i < count; i++) {
    int k = (stopRecordHead - 1 - i + NOTICE_STOP_RECORDS) % NOTICE_STOP_RECORDS;
    snprintf(head, sizeof(head), "  %d)", i + 1);
    PrintStopRecord(head, stopRecords[k]);
  }
  if (stopMinValid) {
    PrintStopRecord("前端の隙間が最小の停止：", stopMin);
  }
}

// 障害物があるときだけ発動する。空いたあとも、見開き・停止後の報告・その後の間合いが
// 終わるまでは手放さない（終わったら優先度0になり、ID25「うろうろ」に戻る）
int NoticeBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (Obstacle_IsBlocked()) {
    return PRIORITY_NOTICE;
  }
  if (state_ != STATE_NOTICED) {
    return 0;
  }
  if ((long)(nowMs - noticedAtMs_) < (long)NOTICE_REACT_MS) {
    return PRIORITY_NOTICE;   // 反応の途中
  }
  if (!clearTimerOn_ || nowMs - clearSinceMs_ < NOTICE_HOLD_MS) {
    return PRIORITY_NOTICE;   // 空いてからの間合い
  }
  return 0;
}

void NoticeBehavior::onStart(unsigned long nowMs) {
  EnterNoticed(nowMs);
}

void NoticeBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  state_ = STATE_IDLE;
}

// 障害物に気づいた。即停止は安全層が行うので、ここでは目標速度を0にして記録を残す
void NoticeBehavior::EnterNoticed(unsigned long nowMs) {
  noticedCm_ = Obstacle_LastCm();
  noticedSpeedOk_ = Obstacle_ApproachSpeed(&noticedSpeed_);
  // 安全層が止めるのはこの後（loop の順番）なので、ここでは気づいた瞬間の速度が読める。
  // 止まっているときに気づいた（目の前に物を置かれた）ぶんは、完了条件の回数に数えない。
  // 回転中は Motion_StartTurn() が速度スムーザーを0に戻しているため noticedWhileMoving_ では
  // 拾えないので別に記録する。ID25 の片側旋回に割り込むときは、調停がこの onStart() より先に
  // WanderBehavior::onStop() を呼んで回転を取り消しているため、同じ周回での取り消しも回転中とみなす
  noticedWhileMoving_ = fabsf(Motion_GetSpeed()) >= MOTOR_SPEED_EPSILON;
  noticedWhileTurning_ = Motion_IsTurning() || Motion_TurnCanceledAt(nowMs);
  CaptureStop(nowMs);
  state_ = STATE_NOTICED;
  noticedAtMs_ = nowMs;
  clearTimerOn_ = false;
  stopReported_ = false;
  Motion_SetSpeed(0.0f, 0, nowMs);

  if (noticedSpeedOk_) {
    Log_Printf("気づく", "正面 %.1fcm に気づいた（接近速度 %.1fcm/s）。即停止して目を見開く",
               noticedCm_, noticedSpeed_);
  } else {
    Log_Printf("気づく", "正面 %.1fcm に気づいた（接近速度は不明）。即停止して目を見開く", noticedCm_);
  }
}

// 気づいた瞬間の様子を取っておく（止まったあと ReportStop で記録に足す）。安全層が止めるのはこの後
void NoticeBehavior::CaptureStop(unsigned long nowMs) {
  NoticeStopRecord &r = pendingStop;
  r.timeMs = nowMs;
  r.before = Arbiter_PreviousName(LAYER_BODY);
  float speed = Motion_GetSpeed();
  float target = Motion_GetTarget();
  r.phase = target > speed + MOTOR_SPEED_EPSILON ? '+' : (target < speed - MOTOR_SPEED_EPSILON ? '-' : '=');
  r.arcing = Motion_IsArcing();
  r.speed = speed;
  unsigned long endMs;
  r.sinceSteerMs = SteerAvoid_LastEndMs(&endMs) ? (int32_t)(nowMs - endMs) : -1;
  r.noticedCm = noticedCm_;
  r.emergency = Obstacle_IsEmergency();
  r.usBlocked = Obstacle_UsIsBlocked();
  int mm;
  unsigned long readyMs;
  uint16_t skipped, still;
  if (TofView_LastForwardFrame(&mm, &readyMs, &skipped, &still)) {
    r.lastFwdMm = (int16_t)mm;
    r.lastFwdAgeMs = max((long)(nowMs - readyMs), 0L);
    r.skippedFrames = skipped;
    r.stillFrames = still;
  } else {
    r.lastFwdMm = -1;
    r.lastFwdAgeMs = -1;
    r.skippedFrames = 0;
    r.stillFrames = 0;
  }
  r.restCm = -1.0f;
  r.gapMm = -1;
}

// 止まりきったころに、停止後の距離を1回だけ出す（ID9 の完了条件の確認と config.h の見直しに使う）
void NoticeBehavior::ReportStop(unsigned long nowMs) {
  (void)nowMs;
  stopReported_ = true;
  float restCm = Obstacle_LastCm();
  float slide = (noticedCm_ >= 0.0f && restCm >= 0.0f) ? (noticedCm_ - restCm) : -1.0f;

  bool countable = noticedWhileMoving_ || noticedWhileTurning_;
  if (!countable) {
    Log_Printf("気づく", "停止後の距離 %.1fcm（止まっているときに気づいたので、通算には数えません）", restCm);
    return;
  }

  stopCount_++;
  TestStats_RecordNoticeStop(restCm, noticedWhileTurning_);
  int gapMm = TofView_ReportStop();
  pendingStop.restCm = restCm;
  pendingStop.gapMm = (int16_t)gapMm;
  stopRecords[stopRecordHead] = pendingStop;
  stopRecordHead = (stopRecordHead + 1) % NOTICE_STOP_RECORDS;
  stopRecordTotal++;
  if (gapMm >= 0 && (!stopMinValid || gapMm < stopMin.gapMm)) {
    stopMinValid = true;
    stopMin = pendingStop;
  }
  const char *turnNote = noticedWhileTurning_ ? "、回転中の検知として別途カウント" : "";
  if (slide >= 0.0f) {
    Log_Printf("気づく", "停止後の距離 %.1fcm（気づいたとき %.1fcm、滑走 %.1fcm、閾値 %.0fcm%s）通算%d回目",
               restCm, noticedCm_, slide, OBSTACLE_SHOWN_STOP_CM, turnNote, stopCount_);
  } else {
    Log_Printf("気づく", "停止後の距離 %.1fcm（気づいたとき %.1fcm、閾値 %.0fcm%s）通算%d回目",
               restCm, noticedCm_, OBSTACLE_SHOWN_STOP_CM, turnNote, stopCount_);
  }
  const float contactWarnCm = 10.0f;   // 非常停止の 12cm より下。ここまで近いと接触を疑う
  if (restCm >= 0.0f && restCm < contactWarnCm) {
    Log_Printf("気づく", "※停止後の距離が%.0fcm未満です。接触したおそれがあります", contactWarnCm);
  }
}

void NoticeBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (state_ != STATE_NOTICED) {
    return;
  }

  if (!stopReported_ && nowMs - noticedAtMs_ >= NOTICE_STOP_REPORT_MS) {
    ReportStop(nowMs);
  }

  if (Obstacle_IsBlocked()) {
    clearTimerOn_ = false;   // まだ空いていない
    return;
  }
  if (!clearTimerOn_) {
    clearTimerOn_ = true;
    clearSinceMs_ = nowMs;
    Log_Printf("気づく", "正面が空いた（%lums 待ってから、うろうろに戻る）", (unsigned long)NOTICE_HOLD_MS);
  }
  // 間合いが過ぎると priority() が0を返し、調停が ID25「うろうろ」に戻す
}

// ------------------------ 目 ------------------------ //

int NoticeEyesBehavior::priority(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  if (!body_->isNoticing()) {
    return 0;
  }
  if ((long)(nowMs - body_->noticedAtMs()) >= (long)NOTICE_WIDE_MS) {
    return 0;   // 見開く時間が過ぎたら、まばたきに戻す
  }
  return PRIORITY_NOTICE_EYES;
}

void NoticeEyesBehavior::onStart(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_WIDE);
}

void NoticeEyesBehavior::onUpdate(const SensorData &sensors, unsigned long nowMs) {
  (void)sensors;
  (void)nowMs;
}

void NoticeEyesBehavior::onStop(unsigned long nowMs) {
  (void)nowMs;
  Eyes_Set(EYE_NORMAL);
}
