// センサー値の集約
// hal から一定間隔で読み取り、振る舞いが参照する1つの構造体にまとめる。
#ifndef NOVA_CORE_SENSORS_H
#define NOVA_CORE_SENSORS_H

#include <Arduino.h>

struct SensorData {
  // 超音波（正面のみ）
  float distanceRawCm;      // 生値。エコーが返らなければ負
  float distanceCm;         // 有効範囲（config.h の ULTRASONIC_MIN/MAX_CM）内なら距離、そうでなければ負
  bool distanceValid;       // distanceCm が有効か

  // 光センサー（ADC生値 0〜4095）
  int lightAdc;

  // ライントラッキング（bit0=左、bit1=中央、bit2=右。1/0の意味は実測待ち）
  uint8_t track;

  // 電池
  int batteryAdc;
  float batteryV;
};

void Sensors_Setup(void);

// loop() から毎回呼ぶ。各センサーを、それぞれの間隔で読み取る
void Sensors_Update(unsigned long nowMs);

const SensorData &Sensors_Get(void);

#endif // NOVA_CORE_SENSORS_H
