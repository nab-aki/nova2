// ジャイロ（LSM6DSV16X。秋月電子 AE-LSM6DSV16X）のレジスタ操作（#0。docs/specs/common_gyro_checklist.md）
// I2C（Wire、アドレス I2C_ADDR_GYRO）につながる4台目の機器。ここはレジスタの読み書きだけを行い、
// 初期化の順番・ゼロ点補正・角度の積算は core/gyro.* が行う。
// 加速度は動かさない（スプリント3.5 の第1段階はジャイロだけ）。
//
// レジスタ・コード表は ST のデータシート DS13510 Rev 4（LSM6DSV16X）で確かめた（節・表の番号は hal_gyro.cpp）。
#ifndef NOVA_HAL_GYRO_H
#define NOVA_HAL_GYRO_H

#include <Arduino.h>

#define GYRO_WHO_AM_I_VALUE   0x70

// FIFO の1件に付くタグ（FIFO_DATA_OUT_TAG の上位5ビット。データシート Table 218）
#define GYRO_FIFO_TAG_GYRO    0x01   // ジャイロ（圧縮なし）

// 設定の読み戻し（起動ログと j キーで表示する）
struct GyroHalSettings {
  uint8_t ifCfg;       // IF_CFG（03h）
  uint8_t fifoCtrl3;   // FIFO_CTRL3（09h）
  uint8_t fifoCtrl4;   // FIFO_CTRL4（0Ah）
  uint8_t ctrl1;       // CTRL1（10h）加速度
  uint8_t ctrl2;       // CTRL2（11h）ジャイロ
  uint8_t ctrl3;       // CTRL3（12h）
  uint8_t ctrl6;       // CTRL6（15h）
  int8_t freqFine;     // INTERNAL_FREQ_FINE（4Fh）。実際の ODR のずれ（0.13% 刻み、2の補数）
};

// WHO_AM_I（0Fh）を読む。読めたら true（値が 0x70 かは呼ぶ側が見る）
bool GyroHal_ReadWhoAmI(uint8_t *value);

// ソフトウェアリセットを始める（すべての設定が既定値に戻る。ESP32 だけがリセットされたあとの状態をそろえるため）
bool GyroHal_StartReset(void);

// リセットが終わったか（SW_RESET ビットが自動で 0 に戻る）
bool GyroHal_IsResetDone(bool *done);

// 測定を始める設定を書く：±500dps・ODR 120Hz（高性能モード）・ノイズ除去フィルター常時有効・加速度は停止。
// FIFO はまだ止めたまま（立ち上がりの間のデータを入れないため。GyroHal_StartFifo で始める）
bool GyroHal_Configure(void);

// FIFO を連続モードで始める（ジャイロだけを 120Hz で入れる）
bool GyroHal_StartFifo(void);

// 設定を読み戻す
bool GyroHal_ReadSettings(GyroHalSettings *settings);

// 読み戻した設定が、書いたとおりか（BDU と IF_INC が 1 のままかも見る）
bool GyroHal_SettingsMatch(const GyroHalSettings &settings);

// INTERNAL_FREQ_FINE から、実際の ODR（Hz）を求める（データシート 9.52節の式）
float GyroHal_ActualOdrHz(int8_t freqFine);

// FIFO にたまっている件数と、あふれ（読む前に古いデータが上書きされた）があったか
bool GyroHal_ReadFifoStatus(uint16_t *words, bool *overrun);

// FIFO から1件（タグ1バイト＋データ6バイト）読む。xyz は生値（下位が先・2の補数）
bool GyroHal_ReadFifoWord(uint8_t *tag, int16_t xyz[3]);

#endif // NOVA_HAL_GYRO_H
