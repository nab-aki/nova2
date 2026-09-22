// フラッシュ（NVS、Preferencesライブラリ）への小さなデータの保存
// 中身の意味は関知しない。バイト列をそのまま保存・読み出し・消去するだけ。
// 電源を切っても消えない。書き込みは摩耗するので、呼ぶ側が「止まっているときだけ」
// 「一定間隔ごと」などタイミングを選ぶこと（頻繁に呼ばない）。
#ifndef NOVA_HAL_STORAGE_H
#define NOVA_HAL_STORAGE_H

#include <Arduino.h>

void Storage_Setup(void);

// buf に len バイト読み込む。保存されていない・サイズが違うときは false（buf は変更しない）
bool Storage_Load(void *buf, size_t len);

// buf の len バイトをそのまま保存する（上書き）
void Storage_Save(const void *buf, size_t len);

// 保存済みのデータを消す
void Storage_Clear(void);

#endif // NOVA_HAL_STORAGE_H
