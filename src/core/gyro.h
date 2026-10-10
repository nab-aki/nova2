// ジャイロ（共通部品。#0。スプリント3.5 の第1段階＝測るだけ。docs/specs/common_gyro_checklist.md）
// LSM6DSV16X を初期化し、FIFO にたまった角速度をまとめて読んで、車体の向き（ヨー角）を積算する。
// 回転の制御にはまだ使わない。今の時間ベースの回転のまま、実際に何度回ったかを測って記録する。
//
// 状態：
//   未検出   … WHO_AM_I が読めない／0x70 でない。本体は今までどおり動く
//   設定中   … ソフトウェアリセット → 設定 → 立ち上がり待ち（100ms）→ 読み戻し → FIFO 開始
//   補正待ち … 車体が止まるのを待っている（角速度は読んでいる。ゼロ点は未補正）
//   補正中   … 止まっている間に約2秒ぶんの平均を取っている（動いたらやり直す）
//   測定中   … ゼロ点を引いた角速度を積算している
//   停止     … 読み取りが GYRO_FAIL_LIMIT 回続けて失敗したので、読むのをやめた（z キーで再開を試せる）
//
// delay() は使わない。loop() から Gyro_Update() を毎回呼ぶ。
#ifndef NOVA_CORE_GYRO_H
#define NOVA_CORE_GYRO_H

#include <Arduino.h>

enum GyroState {
  GYRO_NOT_FOUND,
  GYRO_PROBE,        // 設定中：WHO_AM_I の確認
  GYRO_RESET,        // 設定中：ソフトウェアリセットの完了待ち
  GYRO_SETTLE,       // 設定中：立ち上がり待ち
  GYRO_WAIT_CAL,
  GYRO_CALIBRATING,
  GYRO_RUNNING,
  GYRO_FAILED
};

// FIFO から読んだ1件
struct GyroSample {
  float rawDps[3];     // 各軸の角速度（dps。ゼロ点を引く前）
  float yawRateDps;    // 車体の回る速さ（ゼロ点を引き、符号を合わせたあと。正＝上から見て反時計回り）
  uint32_t timeUs;     // この件の時刻（micros。FIFO を読んだ時刻から件数ぶんさかのぼって割り当てる）
  float dtS;           // 1件ぶんの時間（秒。実際の ODR の逆数）
  bool saturated;      // いずれかの軸が測定範囲の端に当たっていたか
};

// 累計（起動から。RAM だけに持つ）
struct GyroCounters {
  uint32_t samples;       // 読んだ件数
  uint32_t overruns;      // FIFO があふれた回数（読むのが間に合わず、古いデータが消えた）
  uint32_t saturated;     // 頭打ちの件数
  uint32_t jumps;         // 値の飛びの件数（1件前との差が GYRO_JUMP_DPS 以上）
  uint32_t otherTags;     // ジャイロ以外のタグが付いた件数（出ないはず）
  uint32_t calDone;       // ゼロ点補正が終わった回数
  uint32_t calRetries;    // ゼロ点補正をやり直した回数（動いていた）
  uint32_t stops;         // 失敗が続いて読むのをやめた回数
};

void Gyro_Setup(unsigned long nowMs);

// loop() から毎回呼ぶ。初期化を1段ずつ進め、GYRO_READ_INTERVAL_MS（回転中は GYRO_READ_INTERVAL_TURN_MS）ごとに FIFO を読む
void Gyro_Update(unsigned long nowMs);

GyroState Gyro_GetState(void);
const char *Gyro_StateName(void);
bool Gyro_IsReading(void);       // 角速度を読めている状態か（補正待ち・補正中・測定中）
bool Gyro_IsCalibrated(void);    // ゼロ点補正が1回でも終わっているか

float Gyro_YawRateDps(void);     // 直近の、車体の回る速さ
float Gyro_YawDeg(void);         // 積算した向き（起動から。正＝反時計回り。未補正の間はずれが大きい）
float Gyro_ZeroDps(int axis);    // いまのゼロ点（未補正なら 0）
float Gyro_OdrHz(void);          // 積算に使っている ODR（補正値から求めた値）
const GyroCounters &Gyro_GetCounters(void);

// z キー：ゼロ点補正をやり直す。読むのをやめていたら、初期化からやり直す
void Gyro_RequestCalibration(unsigned long nowMs);

// 回転の開始・終了（core/motion.* から呼ぶ）。その回転で実際に回った角度を測り、止まったあとに1行出す
void Gyro_OnTurnStart(const char *name, unsigned long nowMs);
void Gyro_OnTurnStop(unsigned long nowMs);

// 回転の記録を取るかどうか（n キーの測定は車輪を浮かせて回すので、記録を取らない）
void Gyro_SetTurnRecording(bool enabled);

// 1件読むたびに呼ばれる関数を登録する（core/gyro_measure.* が使う。NULL で解除）
typedef void (*GyroSampleListener)(const GyroSample &sample);
void Gyro_SetListener(GyroSampleListener listener);

// FIFO を読む途中で出したいログは、ためておいて、ここで1周に1行ずつ出す（loop の最後に呼ぶ）。
// 回転中は出さない（止める判断を遅らせないため）
void Gyro_PrintPending(void);

void Gyro_Print(void);           // t キー：状態・設定・ゼロ点・累計・回転の記録
void Gyro_PrintNow(void);        // j キー：いまの値

#endif // NOVA_CORE_GYRO_H
