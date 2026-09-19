// Nova 調整値の集約
// 速度・時間・距離などの調整値はすべてここに置く（ピン番号・I2Cアドレスは hal/hal_pins.h）。
// 「実測」の印がある値は docs/measurements.md の実測値。値を変えたら measurements.md / decisions.md も更新する。
#ifndef NOVA_CONFIG_H
#define NOVA_CONFIG_H

// ------------------------ シリアル ------------------------ //
#define SERIAL_BAUD                 115200
#define STATUS_PRINT_INTERVAL_MS    1000    // センサー値と状態の定期表示の間隔

// ------------------------ PCA9685（モーター・サーボ共用）------------------------ //
// PCA9685 はチップ全体で1つのPWM周波数しか持てない。
// 首サーボを保持したまま車体を動かすため、公式サンプルと同じ 50Hz に共通化している。
// 【要再測定】最低PWM（下記 1300）は tools/pwm_test の 1000Hz で測った値。50Hz での値は未測定。
#define PCA9685_FREQUENCY_HZ        50

// ------------------------ モーター ------------------------ //
#define MOTOR_PWM_MIN               1300    // 実測：これ未満では動き出さない（前進・後退・回転とも1300）
#define MOTOR_PWM_LIMIT             2000    // 安全のための上限（tools/pwm_test と同じ）
#define MOTOR_SPEED_EPSILON         0.01f   // 正規化速度（-1〜1）がこれ未満なら停止として扱う

// ------------------------ 首サーボ ------------------------ //
#define SERVO1_FRONT_DEG            84      // 実測：servo1（左右）の正面
#define SERVO1_MIN_DEG              20      // 実測：servo1 の可動範囲（仕様上は 0〜180）
#define SERVO1_MAX_DEG              140
#define SERVO2_LEVEL_DEG            90      // 実測：servo2（上下）の水平
#define SERVO2_MIN_DEG              90      // 実測：servo2 の可動範囲（仕様上は 90〜150）
#define SERVO2_MAX_DEG              140

// ------------------------ 超音波 ------------------------ //
#define ULTRASONIC_INTERVAL_MS      100     // 測距の間隔
#define ULTRASONIC_TIMEOUT_MS       30      // エコーが返らないとみなす時間（300cm往復で約18ms）
#define ULTRASONIC_MIN_CM           2.0f    // 実測：最小検出距離
#define ULTRASONIC_MAX_CM           80.0f   // 実測：安定して測れる最大距離。これより遠い値は「不明」扱い

// ------------------------ センサー読み取り間隔 ------------------------ //
#define LIGHT_READ_INTERVAL_MS      100
#define TRACK_READ_INTERVAL_MS      100
#define BATTERY_READ_INTERVAL_MS    500
#define BATTERY_FULL_V              8.0f    // 実測：満充電時の電圧（現状は表示の参考値）

// ------------------------ WS2812 ------------------------ //
#define WS2812_BRIGHTNESS           20      // 0〜255。派手にしない

// ------------------------ 目（LEDマトリクス）------------------------ //
#define MATRIX_BRIGHTNESS           6       // 0〜15。控えめにする
#define BLINK_INTERVAL_MIN_MS       2000    // ID1：まばたきの間隔（ランダム）
#define BLINK_INTERVAL_MAX_MS       6000
#define BLINK_DOUBLE_PERCENT        20      // ID1：ときどき2回連続になる確率（%）
#define BLINK_FRAME_HALF_MS         60      // 半分閉じている時間（閉じる側・開く側それぞれ）
#define BLINK_FRAME_CLOSED_MS       70      // 閉じきっている時間
#define BLINK_DOUBLE_GAP_MS         140     // 2回連続のときの1回目と2回目の間（目が開いている時間）

// ------------------------ なめらかな加減速（ID7）------------------------ //
#define MOTION_UPDATE_INTERVAL_MS   20      // モーター出力を更新する間隔
#define MOTION_ACCEL_MS             1600    // 停止→巡航にかける時間
#define MOTION_DECEL_MS             1300    // 巡航→停止にかける時間
#define MOTION_WOBBLE_AMPLITUDE     0.03f   // 直進時の速度の揺らぎの大きさ（正規化速度）
#define MOTION_WOBBLE_PERIOD_MS     2300    // 揺らぎの周期（2つの周期を重ねて単調に見せない）
#define MOTION_WOBBLE_PERIOD2_MS    3700
#define MOTION_WOBBLE_MIN_SPEED     0.15f   // これ未満の速度では揺らぎを入れない（最低PWM割れ防止）

// ------------------------ 調停の優先度 ------------------------ //
// 大きいほど優先。安全 > 反応 > 気まま の順で、数値の基準は docs/decisions.md に積み上げる。
#define PRIORITY_IDLE_BLINK         10      // 目：何もなければまばたきする
#define PRIORITY_DEMO_DRIVE         10      // 車体：スプリント0の動作デモ

// ------------------------ 動作デモ（スプリント0）------------------------ //
#define DEMO_CRUISE_SPEED           0.5f    // 巡航の速さ（正規化速度。0.5 なら PWM 約1650）
#define DEMO_CRUISE_MS              1500    // 巡航を保つ時間
#define DEMO_REST_MS                3000    // 止まっている時間
#define DEMO_STOP_DISTANCE_CM       25.0f   // 正面にこれより近い物があれば、走らない・走行中なら減速して止まる

#endif // NOVA_CONFIG_H
