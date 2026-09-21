#include "obstacle.h"

#include "../config.h"
#include "../hal/hal_log.h"

// 1回の測距の分類
struct Sample {
  bool near;           // 閾値未満（障害物）
  bool clear;          // 閾値＋余裕 以上（空いている）
  bool measured;       // 有効範囲内の距離が取れたか（接近速度に使えるか）
  float cm;
  unsigned long ms;
};

static Sample samples[OBSTACLE_HISTORY];
static int sampleCount = 0;   // たまっている数（最大 OBSTACLE_HISTORY）
static int head = 0;          // 次に書き込む位置
static bool blocked = false;
static bool emergency = false;
static float lastCm = -1.0f;

// 古いほうから i 番目（0 が最も古い）の位置
static int IndexOf(int i) {
  return (head + OBSTACLE_HISTORY - sampleCount + i) % OBSTACLE_HISTORY;
}

static void Push(const Sample &s) {
  samples[head] = s;
  head = (head + 1) % OBSTACLE_HISTORY;
  if (sampleCount < OBSTACLE_HISTORY) {
    sampleCount++;
  }
}

void Obstacle_Setup(void) {
  Obstacle_Reset();
  blocked = false;
  lastCm = -1.0f;
}

void Obstacle_Reset(void) {
  sampleCount = 0;
  head = 0;
  emergency = false;
  // blocked は変えない（履歴がたまり直すまで、いまの判定を保つ）
}

int Obstacle_NearCount(void) {
  int n = 0;
  for (int i = 0; i < sampleCount; i++) {
    if (samples[IndexOf(i)].near) n++;
  }
  return n;
}

// 「空いている」と言い切れるサンプルの数（ヒステリシス用）
static int ClearCount(void) {
  int n = 0;
  for (int i = 0; i < sampleCount; i++) {
    if (samples[IndexOf(i)].clear) n++;
  }
  return n;
}

void Obstacle_Update(const SensorData &sensors, unsigned long nowMs) {
  (void)nowMs;
  // 測距が完了していない、または首が動いている間の値は使わない
  if (!sensors.distanceUpdated || !sensors.distanceNeckSteady) {
    return;
  }

  Sample sample;
  sample.ms = sensors.distanceMs;
  sample.cm = sensors.distanceCm;
  sample.measured = sensors.distanceValid;
  emergency = false;

  if (sensors.distanceValid) {
    sample.near = sensors.distanceCm < OBSTACLE_STOP_CM;
    sample.clear = sensors.distanceCm >= OBSTACLE_STOP_CM + OBSTACLE_CLEAR_MARGIN_CM;
    lastCm = sensors.distanceCm;
    if (sensors.distanceCm < OBSTACLE_EMERGENCY_CM) {
      emergency = true;
    }
  } else if (sensors.distanceRawCm >= 0.0f && sensors.distanceRawCm < ULTRASONIC_MIN_CM) {
    // 近すぎて測れない。危険側に倒す
    sample.near = true;
    sample.clear = false;
    emergency = true;
  } else {
    // エコーなし、または安定して測れる範囲より遠い。空いているとみなす
    sample.near = false;
    sample.clear = true;
  }
  Push(sample);

  // 履歴がたまるまでは判定を変えない
  if (sampleCount < OBSTACLE_HISTORY) {
    return;
  }

  bool before = blocked;
  if (!blocked) {
    if (Obstacle_NearCount() >= OBSTACLE_NEAR_COUNT) {
      blocked = true;
    }
  } else if (ClearCount() >= OBSTACLE_NEAR_COUNT) {
    blocked = false;
  }

  if (blocked != before) {
    if (blocked) {
      Log_Printf("障害物", "あり（直近%d回のうち%d回が%.0fcm未満。直近の距離 %.1fcm）",
                 sampleCount, Obstacle_NearCount(), OBSTACLE_STOP_CM, lastCm);
    } else {
      Log_Printf("障害物", "なし（直近%d回のうち%d回が%.0fcm以上。直近の距離 %.1fcm）",
                 sampleCount, ClearCount(), OBSTACLE_STOP_CM + OBSTACLE_CLEAR_MARGIN_CM, lastCm);
    }
  }
}

bool Obstacle_IsBlocked(void) {
  return blocked;
}

bool Obstacle_IsEmergency(void) {
  return emergency;
}

bool Obstacle_IsReady(void) {
  return sampleCount >= OBSTACLE_HISTORY;
}

int Obstacle_SampleCount(void) {
  return sampleCount;
}

float Obstacle_LastCm(void) {
  return lastCm;
}

// 履歴の中の有効な測距値から、最も古いものと最も新しいものを使って接近速度を求める
bool Obstacle_ApproachSpeed(float *cmPerSec) {
  const Sample *oldest = NULL;
  const Sample *newest = NULL;
  for (int i = 0; i < sampleCount; i++) {
    const Sample &s = samples[IndexOf(i)];
    if (!s.measured) continue;
    if (oldest == NULL) oldest = &s;
    newest = &s;
  }
  if (oldest == NULL || newest == oldest) {
    return false;
  }
  unsigned long dtMs = newest->ms - oldest->ms;
  if (dtMs < OBSTACLE_SPEED_MIN_MS) {
    return false;
  }
  *cmPerSec = (oldest->cm - newest->cm) * 1000.0f / (float)dtMs;
  return true;
}
