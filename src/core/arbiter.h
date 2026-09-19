// 振る舞いの調停：レイヤーごとに、優先度が最も高い振る舞いを1つだけ選んで実行する。
//   ・同じ優先度なら、実行中のものを続ける（ふらふら入れ替わらない）。実行中がいなければ登録順が先のもの
//   ・優先度が 0 の振る舞いは選ばれない
#ifndef NOVA_CORE_ARBITER_H
#define NOVA_CORE_ARBITER_H

#include "behavior.h"

#define ARBITER_MAX_BEHAVIORS 16

// 振る舞いを登録する（登録順が同順位のときの優先順）。上限を超えたら false
bool Arbiter_Register(Behavior *behavior);

// loop() から毎回呼ぶ。選び直し、選ばれた振る舞いを1ステップ実行する
void Arbiter_Update(const SensorData &sensors, unsigned long nowMs);

// 現在そのレイヤーで実行中の振る舞い名。なければ "なし"
const char *Arbiter_ActiveName(BehaviorLayer layer);

#endif // NOVA_CORE_ARBITER_H
