#include "safety.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "motion.h"
#include "neck.h"
#include "obstacle.h"
#include "test_stats.h"

static bool stopping = false;
static bool debugTurnActive = false;   // デバッグ回転の1ステップの間は、障害物「あり」では止めない
static bool lifted = false;
static int liftClearCount = 0;   // 床に戻ってから、111 以外を読んだ連続回数

// ライントラッキング（PCF8574）が読めない状態。持ち上げを判定できないので、持ち上げと同じように止める
static bool trackLost = false;
static int trackFailCount = 0;       // 続けて読めなかった回数
static int trackOkCount = 0;         // 読めない状態になってから、続けて読めた回数
static uint32_t trackLostTotal = 0;  // 読めない状態になった回数（起動から。t で表示）

void Safety_Setup(void) {
  stopping = false;
  debugTurnActive = false;
  lifted = false;
  liftClearCount = 0;
  trackLost = false;
  trackFailCount = 0;
  trackOkCount = 0;
}

// 車体が動いている（または動こうとしている）か。回転も含む（core/motion.* の共通判定）
static bool BodyIsMoving(void) {
  return !Motion_IsStill();
}

// 障害物に近づく向きの動きか。前進と片側旋回（片輪が前に出て車体が前へふくらむ）が対象。
// 後退は離れる向き、その場回転は中心が動かないので、障害物があっても止めない
static bool MovingTowardObstacle(void) {
  bool forward = Motion_GetSpeed() >= MOTOR_SPEED_EPSILON ||
                 Motion_GetTarget() >= MOTOR_SPEED_EPSILON;
  return forward || Motion_IsPivoting();
}

// ライントラッキングが読めているかを見張る。読み取ったときだけ数える。
// 続けて読めなければ「読めない状態」にして止め、続けて読めるようになったら戻す
static void WatchTrackRead(const SensorData &sensors) {
  if (!sensors.trackUpdated) {
    return;
  }
  if (!sensors.trackReadOk) {
    trackOkCount = 0;
    if (!trackLost && ++trackFailCount >= SAFETY_TRACK_FAIL_COUNT) {
      trackLost = true;
      trackLostTotal++;
      Log_Printf("安全", "ライントラッキングが%d回続けて読めません。持ち上げを判定できないので、モーターを止めます",
                 SAFETY_TRACK_FAIL_COUNT);
    }
    return;
  }
  trackFailCount = 0;
  if (!trackLost) {
    return;
  }
  // 戻すのは慎重に。続けて読めたときだけ解除する（持ち上げの解除と同じ回数）
  if (++trackOkCount >= SAFETY_LIFT_CLEAR_COUNT) {
    trackLost = false;
    trackOkCount = 0;
    Log_Printf("安全", "ライントラッキングが読めるようになりました。振る舞いは最初からやり直します");
  }
}

// 持ち上げ（ライン 111）を見張る。読み取ったときだけ数える
static void WatchLift(const SensorData &sensors) {
  if (!sensors.trackUpdated) {
    return;
  }
  if (!sensors.trackReadOk) {
    return;   // 読めなかったときの値は前回のまま。判定に使わない
  }
  if (sensors.track == SAFETY_LIFT_TRACK) {
    liftClearCount = 0;
    if (!lifted) {
      lifted = true;
      Log_Printf("安全", "持ち上げを検知（ライン%d%d%d）。モーターを止めます",
                 sensors.track & 0x01, (sensors.track >> 1) & 0x01, (sensors.track >> 2) & 0x01);
      TestStats_RecordLift();
    }
    return;
  }
  if (!lifted) {
    return;
  }
  // 戻すのは慎重に。111 以外が続けて読めたときだけ解除する
  if (++liftClearCount >= SAFETY_LIFT_CLEAR_COUNT) {
    lifted = false;
    liftClearCount = 0;
    Log_Printf("安全", "床に戻りました。振る舞いは最初からやり直します");
  }
}

void Safety_Update(const SensorData &sensors, unsigned long nowMs) {
  WatchTrackRead(sensors);
  WatchLift(sensors);

  // 1. 車体が動いている間は首を正面・水平に固定する。止まったら使用権を返す
  //    （停止中は測距・表情が首を使える）
  if (BodyIsMoving()) {
    Neck_Request(NECK_OWNER_SAFETY, SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG, nowMs);
  } else if (Neck_Owner() == NECK_OWNER_SAFETY) {
    Neck_Release(NECK_OWNER_SAFETY);
  }

  // 2. 持ち上げられていたら（または、ライントラッキングが読めず持ち上げを判定できなければ）、
  //    向きに関わらずすべての動きを止める
  if (lifted || trackLost) {
    if (BodyIsMoving()) {
      Motion_EmergencyStop();   // 目標速度も回転も0に戻る
    }
    stopping = true;
    return;
  }

  // 3. 障害物があれば、近づく向きの動き（前進・片側旋回）だけを即停止する。
  //    デバッグ回転の1ステップの間は、「あり」の判定では止めない（非常停止距離未満では止める）
  bool danger = Obstacle_IsBlocked() || Obstacle_IsEmergency();
  bool mustStop = (debugTurnActive ? false : Obstacle_IsBlocked()) || Obstacle_IsEmergency();
  if (mustStop && MovingTowardObstacle()) {
    const char *reason = Obstacle_IsEmergency() ? "非常停止（近すぎる）" : "障害物あり";
    Log_Printf("安全", "%s のため即停止（距離 %.1fcm）", reason, Obstacle_LastCm());
    Motion_EmergencyStop();
  }
  stopping = danger;
}

void Safety_SetDebugTurnActive(bool active) {
  debugTurnActive = active;
}

bool Safety_IsStopping(void) {
  return stopping;
}

bool Safety_IsLifted(void) {
  return lifted || trackLost;
}

bool Safety_IsTrackLost(void) {
  return trackLost;
}

uint32_t Safety_TrackLostCount(void) {
  return trackLostTotal;
}
