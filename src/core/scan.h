// 首を一方向に向けて測る（共通部品。ID25 の見回しと ID15 の左右確認で使う）
//
// 使い方：begin() で向きと回数を決め、毎ループ update() を呼ぶ。true が返ったら cm() が読める。
// 首の使用権は「測距」（NECK_OWNER_RANGE）で取る。取れなければ取れるまで待つ
// （走行中は安全が優先なので、止まっているときだけ進む）。
//
// 首が安定してから完了した測距だけを数える（設計原則2）。
// その方向の値は「最も近い有効値」とする（危険側に倒す）。有効な値が1つもなければ
// ULTRASONIC_MAX_CM（80cm）＝空いている扱いにする。
#ifndef NOVA_CORE_SCAN_H
#define NOVA_CORE_SCAN_H

#include <Arduino.h>

#include "sensors.h"

class NeckScan {
 public:
  // 首を (panDeg, tiltDeg) に向け、samples 回の測距を集め始める
  void begin(int panDeg, int tiltDeg, int samples, unsigned long nowMs);

  // loop から呼ぶ。集め終わったら true
  bool update(const SensorData &sensors, unsigned long nowMs);

  bool isDone() const { return done_; }
  float cm() const { return closestCm_; }      // 最も近い有効値（なければ ULTRASONIC_MAX_CM）
  int validCount() const { return validCount_; }
  int takenCount() const { return takenCount_; }

 private:
  int panDeg_ = 0;
  int tiltDeg_ = 0;
  int wantedCount_ = 0;
  int takenCount_ = 0;
  int validCount_ = 0;
  float closestCm_ = 0.0f;
  unsigned long beginMs_ = 0;
  bool granted_ = false;
  bool done_ = true;
};

#endif // NOVA_CORE_SCAN_H
