// 車体の動き（ID7 なめらかな加減速の最小版）
// 振る舞いは速度の「目標」だけを指定し、実際の出力はここが Smoother でなめらかにして
// モーターに渡す。急発進・急停止をしない（設計原則3）。直進中はわずかな速度の揺らぎを重ねる。
#ifndef NOVA_CORE_MOTION_H
#define NOVA_CORE_MOTION_H

#include <Arduino.h>

// 回転の種類（docs/specs/25_wander.md「回転」）。
// その場回転＝左右を逆に回す（中心が動かない。壁の前での方向転換）。
// 片側旋回＝片側だけ前進させる（大きく弧を描く。広い場所での向き変え）。
enum TurnKind {
  TURN_ROTATE_LEFT,
  TURN_ROTATE_RIGHT,
  TURN_PIVOT_LEFT,
  TURN_PIVOT_RIGHT
};

void Motion_Setup(void);

// 目標速度（正規化速度 -1.0〜1.0）を設定し、rampMs かけて近づける
void Motion_SetSpeed(float target, unsigned long rampMs, unsigned long nowMs);

// MOTION_DECEL_MS かけてなめらかに止まる
void Motion_Stop(unsigned long nowMs);

// 即座に止める（非常時用。通常の停止には使わない）
void Motion_EmergencyStop(void);

// loop() から毎回呼ぶ。一定間隔でモーター出力を更新する
void Motion_Update(unsigned long nowMs);

float Motion_GetSpeed(void);      // なめらか化後の現在速度（揺らぎ除く）
float Motion_GetTarget(void);
bool Motion_IsAtTarget(void);     // 目標速度に到達したか

// 車体が完全に止まっているか（速度・目標速度・回転のいずれも無し）。
// safety.cpp と core/test_stats.cpp が共通で使う（判定を複製しない）
bool Motion_IsStill(void);

// ------------------------ 回転（その場回転・片側旋回）------------------------ //
// 実測（docs/measurements.md）のとおり、回転・旋回は動き出しに大きなPWMが要るが、
// 回り出せば下げても回り続ける。キック（MOTOR_*_KICK_MS）→ 保持 の2段階で出す。
// キック・保持のPWMと時間は core/turn_tuning.* の値（起動時は config.h。デバッグキーで変えられる）。
// 直進の目標速度は0に戻す（回転中に Motion_SetSpeed を呼ぶと回転は取り消される）。
void Motion_StartTurn(TurnKind kind, unsigned long nowMs);
void Motion_StopTurn(unsigned long nowMs);

// 角度を指示して回る（docs/specs/common_gyro_turn.md）。止めるのは motion が行う。呼ぶ側は Motion_IsTurning() が false になるのを待つ。
//   ・ジャイロが使えるとき：回った角度が「targetDeg − GYRO_TURN_COAST_DEG」に届いたら止める。
//     回っていない（GYRO_TURN_STALL_MS で GYRO_TURN_STALL_DEG 未満）・limitMs がたった・途中でジャイロを失った、でも止める。
//   ・使えないとき（未補正・失敗・片側旋回）：limitMs で止める（今までの時間ベースと同じ動き）。
// limitMs には、今までの時間ベースの値を渡す（予備と上限を兼ねる）。
void Motion_StartTurnDeg(TurnKind kind, float targetDeg, unsigned long limitMs, unsigned long nowMs);
void Motion_PrintTurnStats(void);   // t キー：止め方ごとの回数（起動から）
// 直前の「角度を指示した回転」が、途中で中断されたか（持ち上げ・非常停止・振る舞いの交代・直進の指示）。
// 角度に届いた・時間の上限・回っていない・時間ベース・途中でジャイロを失った、は「終わった」として false
bool Motion_LastTurnAborted(void);
bool Motion_IsTurning(void);
// 時刻 nowMs に、直進・停止の指示（Motion_SetSpeed／Motion_Stop）で回転が取り消されたか。
// 調停は交代時の onStop()・onStart() に同じ nowMs を渡すので、onStart() から
// 「直前の振る舞いが回転中だったか」を判定できる（Motion_StopTurn・非常停止は含まない）
bool Motion_TurnCanceledAt(unsigned long nowMs);
bool Motion_IsPivoting(void);     // 片側旋回か（車体が前へふくらむので、安全層が止める対象）
int Motion_GetTurnPwm(void);      // いま回転に出しているPWM（キック中はキック、そのあとは保持）。回転していなければ0
TurnKind Motion_GetTurnKind(void);
const char *Motion_TurnName(TurnKind kind);

#endif // NOVA_CORE_MOTION_H
