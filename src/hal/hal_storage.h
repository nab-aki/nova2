// フラッシュ（NVS、Preferencesライブラリ）への小さなデータの保存
// 中身の意味は関知しない。バイト列をそのまま保存・読み出し・消去するだけ。
// 電源を切っても消えない。書き込みは摩耗するので、呼ぶ側が「止まっているときだけ」
// 「一定間隔ごと」などタイミングを選ぶこと（頻繁に呼ばない）。
//
// 用途ごとにキーを分ける（名前空間は共通の "nova"。キーは15文字までという NVS の制限がある）。
#ifndef NOVA_HAL_STORAGE_H
#define NOVA_HAL_STORAGE_H

#include <Arduino.h>

#define STORAGE_KEY_TEST_STATS  "teststats"   // core/test_stats.*：試験の集計
#define STORAGE_KEY_TRACE       "trace"       // core/trace.*：落ちる直前の足あと

void Storage_Setup(void);

// buf に len バイト読み込む。保存されていない・サイズが違うときは false（buf は変更しない）
bool Storage_Load(const char *key, void *buf, size_t len);

// buf の len バイトをそのまま保存する（上書き）
void Storage_Save(const char *key, const void *buf, size_t len);

// 保存済みのデータを消す
void Storage_Clear(const char *key);

#endif // NOVA_HAL_STORAGE_H
