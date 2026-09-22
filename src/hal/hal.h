// HAL（ハードウェア操作の窓口）の一括初期化
// ハードウェアの操作は必ず src/hal/ を経由する。core / behaviors からピン操作を直接しない。
#ifndef NOVA_HAL_H
#define NOVA_HAL_H

#include "hal_battery.h"
#include "hal_buzzer.h"
#include "hal_ir.h"
#include "hal_light.h"
#include "hal_matrix.h"
#include "hal_motor.h"
#include "hal_reset.h"
#include "hal_servo.h"
#include "hal_storage.h"
#include "hal_track.h"
#include "hal_ultrasonic.h"
#include "hal_ws2812.h"

// 全ハードウェアを初期化する（WS2812 は GPIO32 の共用があるため含めない。hal_ws2812.h 参照）。
// 戻り値：ライントラッキングセンサーが応答したか（応答しなくても起動は続ける）
bool Hal_Setup(void);

#endif // NOVA_HAL_H
