// センサー値の集約
// hal から一定間隔で読み取り、振る舞いが参照する1つの構造体にまとめる。
#ifndef NOVA_CORE_SENSORS_H
#define NOVA_CORE_SENSORS_H

#include <Arduino.h>

struct SensorData {
  // 超音波（首に搭載。首の向いている方向を測る）
  float distanceRawCm;         // 生値。エコーが返らなければ負
  float distanceCm;            // 有効範囲（config.h の ULTRASONIC_MIN/MAX_CM）内なら距離、そうでなければ負
  bool distanceValid;          // distanceCm が有効か
  bool distanceUpdated;        // このループで測距が1回完了したか（障害物の判定はこのときだけ進める）
  bool distanceNeckSteady;     // その測距を、首が安定した状態で取れたか
  unsigned long distanceMs;    // その測距が完了した時刻

  // 光センサー（ADC生値 0〜4095）
  int lightAdc;

  // ライントラッキング（bit0=左、bit1=中央、bit2=右。実測：床=000、持ち上げ=111）
  uint8_t track;
  bool trackUpdated;           // このループで読み取ったか（持ち上げの判定は読んだ回数で数える）
  bool trackReadOk;            // 直近の読み取りが成功したか。失敗のとき track は前回の値のまま（信用しない）

  // 電池
  int batteryAdc;
  float batteryV;
};

void Sensors_Setup(void);

// loop() から毎回呼ぶ。各センサーを、それぞれの間隔で読み取る
void Sensors_Update(unsigned long nowMs);

const SensorData &Sensors_Get(void);

#endif // NOVA_CORE_SENSORS_H
