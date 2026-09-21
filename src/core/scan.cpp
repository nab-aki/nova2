#include "scan.h"

#include "../config.h"
#include "neck.h"

void NeckScan::begin(int panDeg, int tiltDeg, int samples, unsigned long nowMs) {
  panDeg_ = panDeg;
  tiltDeg_ = tiltDeg;
  wantedCount_ = max(samples, 1);
  takenCount_ = 0;
  validCount_ = 0;
  closestCm_ = ULTRASONIC_MAX_CM;
  beginMs_ = nowMs;
  granted_ = false;
  done_ = false;
}

bool NeckScan::update(const SensorData &sensors, unsigned long nowMs) {
  if (done_) {
    return true;
  }

  // 首を向ける。ほかがより高い優先度で使っている間は取れないので、取れるまで申し出続ける
  if (!granted_) {
    granted_ = Neck_Request(NECK_OWNER_RANGE, panDeg_, tiltDeg_, nowMs);
    if (!granted_) {
      return false;
    }
  }

  // 首が安定した状態で完了した測距だけを数える。
  // begin() と同じループで完了した測距は、首を向ける前に始まったものなので使わない
  if (!sensors.distanceUpdated || !sensors.distanceNeckSteady ||
      (long)(sensors.distanceMs - beginMs_) <= 0) {
    return false;
  }

  takenCount_++;
  if (sensors.distanceValid) {
    if (validCount_ == 0 || sensors.distanceCm < closestCm_) {
      closestCm_ = sensors.distanceCm;
    }
    validCount_++;
  }

  if (takenCount_ >= wantedCount_) {
    done_ = true;
  }
  return done_;
}
