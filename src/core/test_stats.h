// 試験の集計（共通部品。#0。docs/specs/common_test_stats.md）
// 「1回の試験」＝再開してから一時停止するまで（起動時に一時停止でなければ起動から）。
// ID9「気づく」・ID15「障害物で困る」・ID25「うろうろ」・安全層（持ち上げ）から
// 集計イベントを受け取り、直近3回分を NVS（フラッシュ）に保存する。
//
// 保存のタイミングは3つ（走行中はフラッシュに書かない。保存中は CPU が止まり、ToF の読み出しも止まるため）：
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

#include "stuck_watch.h"

// NVSから読み込み、一時停止で始まらない設定なら、それ自体を最初の試験の開始として扱う
void TestStats_Setup(bool startPaused, unsigned long nowMs);

// loop() から毎回呼ぶ。止まっているときの保存（一時停止後の確定・周期保存）を行う
void TestStats_Update(unsigned long nowMs);

// 一時停止・再開が切り替わったときに呼ぶ（main.cpp の TogglePause から）。
// paused=true：試験を終える（保存は、止まってから TestStats_Update が行う）
// paused=false：新しい試験を始める
void TestStats_OnPauseToggle(bool paused, unsigned long nowMs);

// 試験の最中か（再開してから一時停止するまでの間）
bool TestStats_IsRecording(void);

// ------------------------ 集計イベントの記録 ------------------------ //
// 記録中（再開してから一時停止するまでの間）でなければ、呼んでも何もしない。

// ID9：気づいて停止したときの、停止後の距離（cm）。測れなかったときは負を渡す。
// whileTurning：回転中に気づいたか（回転中の検知は stopCount には含めつつ、内訳として別に数える。
// Motion_StartTurn() が速度スムーザーを0に戻すため、走っているときの判定とは別に見る必要がある）
void TestStats_RecordNoticeStop(float restCm, bool whileTurning);

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

// ID25：歩き出した（加速を始めた。途中で止められた回も1回）
void TestStats_RecordWalk(void);

// loop の周期（µs）。毎 loop 呼ぶ（スプリント3.6-3 段0。ToF のタスクの影響を見る）
void TestStats_RecordLoopUs(uint32_t us);

// 段2（common_tof_scan.md 9章）
void TestStats_RecordFarClear(void);        // 上の段は測れず床が写るので「遠くて空き」とした
void TestStats_RecordBlindNoWalk(void);     // 見えていないので歩かなかった
void TestStats_RecordBlindGiveup(void);     // 見えていない向きが続いて、回るのをやめた
void TestStats_RecordBlindStop(void);       // 前進中に見えていない状態が続いて止まった
void TestStats_RecordNeckLook(bool onsetSeen);   // 首を振って見た。onsetSeen が false なら「動かなかった疑い」
void TestStats_RecordStopGap(int gapMm, int sideMm);   // 止まったあとの前端の隙間と横の余裕。なければ負

// ID27（docs/specs/27_steer_avoid.md）
void TestStats_RecordSteerStart(bool toLeft);       // 曲がり始めた。toLeft：左へ曲がる
void TestStats_RecordSteerCleared(void);            // よけきれた（曲がり終えてから STEER_CLEARED_MS 止まらなかった）
void TestStats_RecordSteerInterrupted(void);        // 曲がっている途中で交代した
void TestStats_RecordSteerChained(void);            // 結果が決まる前に、また曲がった
void TestStats_RecordRunKept(void);                 // ID25 v2：巡航の時間が切れたが、通路が空いていて続けた
// ID25 v2：巡航の時間が切れて止まった理由（続ける条件のうち、確かめる順に最初に満たさなかったもの）
enum RunStopReason {
  RUN_STOP_STUCK = 0,   // 詰まりの疑い
  RUN_STOP_TOF,         // ToF が使えない（測距中でない・枚が古い）
  RUN_STOP_BLOCKED,     // 判定が「あり」
  RUN_STOP_NEAR,        // 通路の最も近い物が WANDER_KEEP_CLEAR_MM 未満
  RUN_STOP_BLIND,       // 見えていない
  RUN_STOP_REASONS
};
// ID25 v2：巡航の時間が切れて止まった。nearMm：「近い」のときの通路の最も近い距離（それ以外は負）
void TestStats_RecordRunTimedStop(RunStopReason reason, int nearMm);
void TestStats_RecordKeptThenNotice(void);          // ID25 v2：続けたあと、次の時間切れの前に ID9 が止めた（記録のみ）
void TestStats_RecordSteerAccelMiss(void);          // 加速中にきっかけを満たしたが、巡航ではないので曲がらなかった
// 曲がり終えたあと、止まった。msSinceEnd：曲がり終えてからの時間。withinWindow：STEER_CLEARED_MS より前か
void TestStats_RecordSteerNextStop(unsigned long msSinceEnd, bool withinWindow);

// 安全層：持ち上げを検知した
void TestStats_RecordLift(void);

// ID26：詰まりに気づいた（何で気づいたかの内訳も数える）
void TestStats_RecordStuckDetected(StuckReason reason);

// ID26：回転まで最後までやり切って脱出した（signalA：信号A で気づいた回か）
void TestStats_RecordStuckEscaped(bool signalA);

// ID26：脱出してから STUCK_REPEAT_MS 以内にまた詰まった（大きく回る）
void TestStats_RecordStuckRepeat(void);

// ID26：見回しが「同じ」が続いている時間（ms）。最長を残す
void TestStats_RecordStuckUnchangedMs(unsigned long ms);

// ------------------------ シリアルキー ------------------------ //

// h：手動で保存する。走行中なら何もしない（メッセージだけ出す）
void TestStats_RequestSave(unsigned long nowMs);

// t：集計を表示する
void TestStats_Print(unsigned long nowMs);

// x：消去する（5秒以内に2回押して確定）
void TestStats_HandleClearKey(unsigned long nowMs);

#endif // NOVA_CORE_TEST_STATS_H
