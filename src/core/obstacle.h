// 障害物の判定（共通部品。docs/specs/common_obstacle.md）
// 測距値は1回きりでは信用せず、直近 OBSTACLE_HISTORY 回のうち OBSTACLE_NEAR_COUNT 回以上が
// 閾値未満なら「障害物あり」とする。いちど「あり」になったら、閾値＋余裕まで戻るまで「あり」のまま
// （ヒステリシス。壁ぎわで進む・止まるを繰り返さないため）。
//
// 首が安定しているときに完了した測距だけを使う。首が動いたら履歴を捨てる。
#ifndef NOVA_CORE_OBSTACLE_H
#define NOVA_CORE_OBSTACLE_H

#include <Arduino.h>

#include "sensors.h"

void Obstacle_Setup(void);

// loop() から毎回呼ぶ。新しい測距が完了していれば履歴に入れて判定し直す
void Obstacle_Update(const SensorData &sensors, unsigned long nowMs);

// 履歴を捨てる（首が動いたときなど）。判定は履歴がたまるまで変えない
void Obstacle_Reset(void);

bool Obstacle_IsBlocked(void);     // 障害物あり（ヒステリシス込み）
bool Obstacle_IsEmergency(void);   // 1回の測距で即停止すべきほど近い
bool Obstacle_IsReady(void);       // 履歴がたまっていて、判定を信用してよいか

int Obstacle_NearCount(void);      // 履歴のうち「近い」の数
int Obstacle_SampleCount(void);    // 履歴にたまっている数
float Obstacle_LastCm(void);       // 直近の有効な測距値。まだなければ負

// 接近速度（cm/s。正＝近づいている）。求められなければ false
bool Obstacle_ApproachSpeed(float *cmPerSec);

#endif // NOVA_CORE_OBSTACLE_H
