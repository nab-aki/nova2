// 首（servo1 左右／servo2 上下）の調停（共通部品。docs/specs/common_neck.md）
// 首には目と超音波の両方が載っているため、「安全のために正面へ固定したい」「測るために向けたい」
// 「表情として動かしたい」が同時に起きる。使用権を「安全＞測距＞表情」の優先度で1つに決める。
//
// 首を動かしている間と、止まってから安定するまでの測距値は信用しない（設計原則2）。
// 安定までの待ち時間は実測にもとづく（config.h の NECK_SETTLE_* を参照）。
#ifndef NOVA_CORE_NECK_H
#define NOVA_CORE_NECK_H

#include <Arduino.h>

// 使用権の持ち主。優先度は config.h の NECK_PRIORITY_* で決める
enum NeckOwner {
  NECK_OWNER_NONE,
  NECK_OWNER_EXPRESSION,   // 表情（首かしげ・見上げる）
  NECK_OWNER_RANGE,        // 測距（停止中の見回し）
  NECK_OWNER_SAFETY,       // 安全（走行中の正面固定）
  NECK_OWNER_COUNT
};

void Neck_Setup(void);

// 首を使いたいと申し出て、角度を指定する。
// 自分より優先度の高い持ち主がいれば false を返し、首は動かさない。
// いまと同じ角度なら首を動かさず、安定待ちもやり直さない（毎ループ申し出てよい）。
bool Neck_Request(NeckOwner owner, int panDeg, int tiltDeg, unsigned long nowMs);

// 使用権を返す。自分が持っていなければ何もしない
void Neck_Release(NeckOwner owner);

NeckOwner Neck_Owner(void);
const char *Neck_OwnerName(NeckOwner owner);

// 首が止まっていて、測距値を信用してよいか
bool Neck_IsSteady(unsigned long nowMs);

// 正面・水平か（走り出してよい条件のひとつ）
bool Neck_IsFront(void);

// loop() から毎回呼ぶ。安定したかどうかを見て、変化をシリアルに出す
void Neck_Update(unsigned long nowMs);

#endif // NOVA_CORE_NECK_H
