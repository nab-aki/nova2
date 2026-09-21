#include "neck.h"

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_servo.h"
#include "obstacle.h"

static NeckOwner owner = NECK_OWNER_NONE;
static unsigned long steadyAtMs = 0;   // この時刻を過ぎたら安定とみなす
static bool steady = false;

static const char *const OWNER_NAMES[NECK_OWNER_COUNT] = {"なし", "表情", "測距", "安全"};

static int OwnerPriority(NeckOwner o) {
  switch (o) {
    case NECK_OWNER_SAFETY:     return NECK_PRIORITY_SAFETY;
    case NECK_OWNER_RANGE:      return NECK_PRIORITY_RANGE;
    case NECK_OWNER_EXPRESSION: return NECK_PRIORITY_EXPRESSION;
    default:                    return 0;
  }
}

// 動かした角度から、測距値を信用してよくなるまでの待ち時間を求める
static unsigned long SettleMs(int movedDeg) {
  return NECK_SETTLE_BASE_MS + (unsigned long)movedDeg * NECK_SETTLE_PER_DEG_MS;
}

void Neck_Setup(void) {
  owner = NECK_OWNER_NONE;
  steady = false;
  // 起動時は Servo_Setup() で正面・水平に動いた直後。どこから動いたか分からないので
  // 可動範囲いっぱい（180°）動いたものとして、いちばん長い待ち時間を取る
  steadyAtMs = millis() + SettleMs(180);
  Log_Printf("首", "正面・水平から開始（左右%d° 上下%d°）。%lums は測距を使わない",
             Servo_GetAngle(SERVO_PAN), Servo_GetAngle(SERVO_TILT), SettleMs(180));
}

bool Neck_Request(NeckOwner requester, int panDeg, int tiltDeg, unsigned long nowMs) {
  if (requester == NECK_OWNER_NONE) {
    return false;
  }
  // 自分より優先度の高い持ち主が使っている間は断る
  if (owner != NECK_OWNER_NONE && owner != requester &&
      OwnerPriority(requester) < OwnerPriority(owner)) {
    return false;
  }

  if (owner != requester) {
    Log_Printf("首", "使用権 %s→%s", OWNER_NAMES[owner], OWNER_NAMES[requester]);
    owner = requester;
  }

  int beforePan = Servo_GetAngle(SERVO_PAN);
  int beforeTilt = Servo_GetAngle(SERVO_TILT);
  int afterPan = Servo_SetAngle(SERVO_PAN, panDeg);
  int afterTilt = Servo_SetAngle(SERVO_TILT, tiltDeg);

  // 実際に動いた角度（左右・上下の大きいほう）から安定までの待ち時間を決める
  int moved = max(abs(afterPan - beforePan), abs(afterTilt - beforeTilt));
  if (moved > 0) {
    steadyAtMs = nowMs + SettleMs(moved);
    if (steady) {
      steady = false;
      Obstacle_Reset();   // 首が動いたので、それまでの測距の履歴は使えない
      Log_Printf("首", "%s が %d°動かした（左右%d° 上下%d°）。%lums は測距を使わない",
                 OWNER_NAMES[owner], moved, afterPan, afterTilt, SettleMs(moved));
    }
  }
  return true;
}

void Neck_Release(NeckOwner o) {
  if (o == NECK_OWNER_NONE || owner != o) {
    return;
  }
  Log_Printf("首", "使用権 %s→なし", OWNER_NAMES[owner]);
  owner = NECK_OWNER_NONE;
}

NeckOwner Neck_Owner(void) {
  return owner;
}

const char *Neck_OwnerName(NeckOwner o) {
  return (o >= 0 && o < NECK_OWNER_COUNT) ? OWNER_NAMES[o] : "不明";
}

bool Neck_IsSteady(unsigned long nowMs) {
  return (long)(nowMs - steadyAtMs) >= 0;
}

bool Neck_IsFront(void) {
  return Servo_GetAngle(SERVO_PAN) == SERVO1_FRONT_DEG &&
         Servo_GetAngle(SERVO_TILT) == SERVO2_LEVEL_DEG;
}

void Neck_Update(unsigned long nowMs) {
  if (!steady && Neck_IsSteady(nowMs)) {
    steady = true;
    Log_Printf("首", "安定（左右%d° 上下%d°）。測距を使い始める",
               Servo_GetAngle(SERVO_PAN), Servo_GetAngle(SERVO_TILT));
  }
}
