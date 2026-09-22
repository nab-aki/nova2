// 試験の集計（共通部品。#0。docs/specs/common_test_stats.md）
// 「1回の試験」＝再開してから一時停止するまで（起動時に一時停止でなければ起動から）。
// ID9「気づく」・ID15「障害物で困る」・ID25「うろうろ」・安全層（持ち上げ）から
// 集計イベントを受け取り、直近3回分を NVS（フラッシュ）に保存する。
//
// 保存のタイミングは3つ（走行中はフラッシュに書かない）：
//   ・一時停止したとき（ただし、なめらかに減速して車体が止まってから）
//   ・シリアルの保存キー（h）
//   ・車体が止まっているときの30秒ごと
// 電源が切れて試験が途中で終わっても、最後に保存された時点までは次の起動に残る
// （走行中の分が失われるのは許容する）。
//
// 一時停止中（再開してから一時停止するまでの外）は、記録の呼び出しをすべて無視する。
#ifndef NOVA_CORE_TEST_STATS_H
#define NOVA_CORE_TEST_STATS_H

#include <Arduino.h>

// NVSから読み込み、一時停止で始まらない設定なら、それ自体を最初の試験の開始として扱う
void TestStats_Setup(bool startPaused, unsigned long nowMs);

// loop() から毎回呼ぶ。止まっているときの保存（一時停止後の確定・周期保存）を行う
void TestStats_Update(unsigned long nowMs);

// 一時停止・再開が切り替わったときに呼ぶ（main.cpp の TogglePause から）。
// paused=true：試験を終える（保存は、止まってから TestStats_Update が行う）
// paused=false：新しい試験を始める
void TestStats_OnPauseToggle(bool paused, unsigned long nowMs);

// ------------------------ 集計イベントの記録 ------------------------ //
// 記録中（再開してから一時停止するまでの間）でなければ、呼んでも何もしない。

// ID9：気づいて停止したときの、停止後の距離（cm）。測れなかったときは負を渡す
void TestStats_RecordNoticeStop(float restCm);

// ID15：立て直しを始めた（StartSequence。あきらめ後の再挑戦・持ち上げ後の再開も含む）
void TestStats_RecordTroubleStart(void);

// ID15：正面が空いて、うろうろに戻った
void TestStats_RecordTroubleCleared(void);

// ID15：一周しても空かず、あきらめた
void TestStats_RecordTroubleGiveup(void);

// ID25：片側旋回を実施した（歩き出す前に、横が近いので向きを変え始めた）
void TestStats_RecordPivotPerformed(void);

// ID25：片側旋回が途中で止められた（交代・安全層のどちらも）
void TestStats_RecordPivotInterrupted(void);

// ID25：後退+その場回転（横がとても近いときの張りつき対策）を実施した
void TestStats_RecordRecoverPerformed(void);

// ID25：後退+その場回転が途中で交代された（優先度を上げてあるので、通常は一時停止のときだけ）
void TestStats_RecordRecoverInterrupted(void);

// 安全層：持ち上げを検知した
void TestStats_RecordLift(void);

// ------------------------ シリアルキー ------------------------ //

// h：手動で保存する。走行中なら何もしない（メッセージだけ出す）
void TestStats_RequestSave(unsigned long nowMs);

// t：集計を表示する
void TestStats_Print(unsigned long nowMs);

// x：消去する（5秒以内に2回押して確定）
void TestStats_HandleClearKey(unsigned long nowMs);

#endif // NOVA_CORE_TEST_STATS_H
