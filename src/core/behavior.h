// 振る舞い（behavior）の基底クラス
// 1機能1ファイル（src/behaviors/）。各振る舞いは
//   ・どんなときに発動したいか（優先度を返す）
//   ・発動したら何をするか
// だけを持ち、実行するかどうかは core の調停（arbiter）が優先度で決める。
//
// 調停は「レイヤー」ごとに行い、各レイヤーで同時に実行されるのは常に1つだけ。
// レイヤー = 同じ部品を取り合う振る舞いのグループ。
//   LAYER_BODY … 車体（と首）の動き
//   LAYER_EYES … 目の表情
// 目は車体と別の部品なので、「動きながらまばたき」のように別レイヤーの振る舞いは並行して動ける。
#ifndef NOVA_CORE_BEHAVIOR_H
#define NOVA_CORE_BEHAVIOR_H

#include <Arduino.h>

#include "sensors.h"

enum BehaviorLayer {
  LAYER_BODY,
  LAYER_EYES,
  LAYER_COUNT
};

class Behavior {
 public:
  virtual ~Behavior() {}

  virtual const char *name() const = 0;
  virtual BehaviorLayer layer() const = 0;

  // 発動したいときは優先度（1以上。大きいほど優先）、発動したくないときは 0 を返す。
  // 実行中も毎ループ呼ばれる。状態を変えてはいけない（見るだけ）。
  virtual int priority(const SensorData &sensors, unsigned long nowMs) = 0;

  // 選ばれた瞬間に1回呼ばれる
  virtual void onStart(unsigned long nowMs) { (void)nowMs; }

  // 選ばれている間、毎ループ呼ばれる
  virtual void onUpdate(const SensorData &sensors, unsigned long nowMs) = 0;

  // 他の振る舞いに取って代わられた（または発動条件がなくなった）ときに1回呼ばれる。
  // 使っていた部品（車体など）を安全な状態に戻す。
  virtual void onStop(unsigned long nowMs) { (void)nowMs; }
};

#endif // NOVA_CORE_BEHAVIOR_H
