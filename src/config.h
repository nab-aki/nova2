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
// 最低PWMなどは 50Hz で測定済み（下記。docs/measurements.md）。
#define PCA9685_FREQUENCY_HZ        50

// ------------------------ モーター ------------------------ //
// 最低PWM・動き出し・回り続ける値は、tools/measure_m2 で 50Hz・床（フローリング）・電池7.8〜7.9V のときに
// 測った実測値（docs/measurements.md）に、電池の低下への余裕を足したもの。
// 電池が下がると同じPWMでは動かなくなる。必要なPWMは電圧に反比例すると見て、測定時 7.85V から 7.0V まで
// 使えるよう、実測値に ×1.12（7.85÷7.0）を掛けて、前進・後退は25刻み、回転・旋回は50刻みに切り上げた。
// 7.0V は仮の値（電池の動作下限は未調査。ID22 で決めたら見直す）。
#define MOTOR_PWM_MIN               675     // 前進の最低PWM。実測：動き出し 600 × 1.12 ≒ 672 → 675
#define MOTOR_PWM_MIN_BACKWARD      650     // 後退の最低PWM。実測：動き出し 575 × 1.12 ≒ 644 → 650（HALは未対応。ID25で使う）
#define MOTOR_PWM_LIMIT             2000    // 安全のための上限（tools/pwm_test と同じ）
#define MOTOR_SPEED_EPSILON         0.01f   // 正規化速度（-1〜1）がこれ未満なら停止として扱う

// その場回転・片側旋回（ID25）は、4輪の横滑りのため動き出しに大きな力が要る。ただし回り出せば下げても回り続ける。
// そこで「動き出し（キック）」のPWMを短時間出してから、「回り続ける」PWMに下げる。
// 値は正規化速度ではなく生のPWM（HAL に生PWMで出す経路は ID25 の実装で足す）。
#define MOTOR_ROTATE_KICK_PWM       1150    // その場回転の動き出し。実測 左1000・右950 の大きい方 × 1.12 ≒ 1120 → 1150（左右同じ値）
#define MOTOR_ROTATE_HOLD_PWM       900     // その場回転の回り続ける値。実測 800（左右とも）× 1.12 ≒ 896 → 900
#define MOTOR_PIVOT_KICK_PWM        1000    // 片側旋回の動き出し。実測 左右とも 850 × 1.12 ≒ 952 → 1000
#define MOTOR_PIVOT_HOLD_PWM        850     // 片側旋回の回り続ける値。実測 左750・右700 の大きい方 × 1.12 ≒ 840 → 850
#define MOTOR_TURN_KICK_MS          300     // 【仮値・未測定】キックを出す時間。ID25 の実装時に目で見て詰める

// ------------------------ 首サーボ ------------------------ //
#define SERVO1_FRONT_DEG            84      // 実測：servo1（左右）の正面
#define SERVO1_MIN_DEG              20      // 実測：servo1 の可動範囲（仕様上は 0〜180）
#define SERVO1_MAX_DEG              140
#define SERVO2_LEVEL_DEG            98      // 実測：超音波が水平になる servo2 の角度（M2案で測定）
#define SERVO2_MIN_DEG              90      // 実測：servo2 の可動範囲（仕様上は 90〜150）
#define SERVO2_MAX_DEG              140

// ------------------------ 超音波 ------------------------ //
// 測距の間隔は、実測（60ms間隔×20回で全距離・全対象とも有効20/20）に合わせて 60ms にしている。
// 障害物の判定が「直近5回のうち3回」なので、間隔が短いほど検知が早い（100ms→300ms、60ms→180ms）。
#define ULTRASONIC_INTERVAL_MS      60      // 測距の間隔
#define ULTRASONIC_TIMEOUT_MS       30      // エコーが返らないとみなす時間（300cm往復で約18ms）
#define ULTRASONIC_MIN_CM           2.0f    // 実測：最小検出距離
#define ULTRASONIC_MAX_CM           80.0f   // 実測：安定して測れる最大距離。これより遠い値は「不明」扱い
#define ULTRASONIC_NO_ECHO_WARN_MS  10000   // エコーなしがこれだけ続いたら警告を1行出す（動作は変えない）

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

// ------------------------ 首の調停（共通部品）------------------------ //
// 首を動かしてから測距値が信用できるまでの待ち時間：
//   待ち時間 = NECK_SETTLE_BASE_MS + 動かした角度 × NECK_SETTLE_PER_DEG_MS
// 実測（docs/measurements.md。0ms は「動かす命令を出した時刻」なので移動時間を含む）：
//   ±20°→最長60ms、±40°→最長60ms、±55°→最長160ms
// この式では 20°→100ms、40°→140ms、55°→170ms となり、いずれも実測の最長を上回る。
// ±20°で「0ms」が多く出たのは、超音波の指向角が広く首を振っても同じ壁の最近点を見ていたため
// と考えられる（首が一瞬で止まった証拠ではない）ので、小さい動きでも BASE は必ず待つ。
#define NECK_SETTLE_BASE_MS         60      // 首を動かしたら必ず待つ時間
#define NECK_SETTLE_PER_DEG_MS      2       // 動かした角度1°あたりの追加の待ち時間
#define NECK_PRIORITY_SAFETY        30      // 使用権：安全（走行中の正面固定）
#define NECK_PRIORITY_RANGE         20      // 使用権：測距（停止中の見回し）
#define NECK_PRIORITY_EXPRESSION    10      // 使用権：表情（首かしげなど）

// ------------------------ 障害物の判定（共通部品）------------------------ //
// 停止閾値 36cm の根拠（docs/specs/common_obstacle.md。2026-09-21 の ID9 実機評価で見直した）：
//   実機の実測（壁0°・フローリング・巡航PWM999）：接近速度 59.6cm/s、気づいた距離 25.3cm、
//   停止後の距離 約10cm。ここから内訳が分解できる。
//     判定の遅れ：35 - 25.3 = 9.7cm（163ms 相当。多数決の最悪 180ms の範囲内）
//     滑走：25.3 - 10 = 15.3cm
//   巡航を PWM726 に落とすと、速度は 43.3cm/s、
//   滑走は速度の2乗に比例するので 15.3 × (726/999)^2 ≒ 8.1cm になる。
//   停止後の距離を 20cm にする閾値：20 + 43.3×0.180(判定の遅れ) + 8.1(滑走) ≒ 36cm
//   参考：巡航を下げずに 20cm を取るには閾値 46cm が必要だった。
//   上の 59.6cm/s・15.3cm は手で確かめた値。床・正対した壁・1mスタートで確認したところ
//   接近速度 41.1cm/s・滑走 7.7cm・停止後 21.4cm（気づいた距離 29.1cm）で、いずれも予想と
//   5%以内で一致した。36cm はこのまま据え置く（docs/specs/09_notice.md）。
// 判定の遅れ 180ms は多数決そのもの（1回目の「近い」まで最大60ms＋2回で120ms）で、
// 測距の完了から停止までに余分な遅れはない（同じ loop 内で安全層が Motor_Stop を直接呼ぶ）。
#define OBSTACLE_HISTORY            5       // 判定に使う直近の測距回数
#define OBSTACLE_NEAR_COUNT         3       // このうち何回が閾値未満なら「障害物あり」か
#define OBSTACLE_STOP_CM            36.0f   // 停止閾値
#define OBSTACLE_CLEAR_MARGIN_CM    10.0f   // ヒステリシス：36+10=46cm 以上に戻ったら「なくなった」
#define OBSTACLE_EMERGENCY_CM       12.0f   // 1回でもこれ未満なら多数決を待たず即停止
#define OBSTACLE_SPEED_MIN_MS       150     // 接近速度を求めるのに必要な最小の時間差

// ------------------------ 巡航 ------------------------ //
// 巡航は PWM726 を据え置く。実測（床・正対した壁・1mスタート）で接近速度 41.1cm/s・滑走 7.7cm・
// 停止後 21.4cm だった値で、停止閾値 36cm もこの実測に合わせている。
// ねらいの 35〜40cm/s は取り下げた（必要なPWMの根拠にした数値が誤りだったため。docs/decisions.md）。
// MOTOR_PWM_MIN を 700→675 にしたので、PWM が 726 のまま変わらないよう計算し直した：
//   MOTOR_PWM_MIN + (MOTOR_PWM_LIMIT - MOTOR_PWM_MIN) × 0.0385 = 675 + 1325 × 0.0385 ≒ 726
//   （726 - 675）÷ 1325 = 0.0385。MOTOR_PWM_MIN か MOTOR_PWM_LIMIT を変えたら計算し直すこと。
// 【注意】巡航PWMは最低値 675 の近く（余裕 約50）。加速の「ため」（ID7）が効くのは speed 0.01〜0.0385
//   （PWM 688〜726）の間だけで、speed<0.01 は PWM0。MOTION_WOBBLE_MIN_SPEED 0.15 も下回るので
//   速度の揺らぎは入らない。
#define CRUISE_SPEED                0.0385f // 巡航の正規化速度（PWM 726 相当）

// ------------------------ 調停の優先度 ------------------------ //
// 大きいほど優先。安全 > 反応 > 気まま の順で、数値の基準は docs/decisions.md に積み上げる。
#define PRIORITY_IDLE_BLINK         10      // 目：何もなければまばたきする
#define PRIORITY_NOTICE             20      // 車体：ID9 気づく
#define PRIORITY_NOTICE_EYES        30      // 目：ID9 気づいて見開く（まばたきより優先）

// ------------------------ ID9 気づく ------------------------ //
#define NOTICE_REST_MS              3000    // 止まっている時間
#define NOTICE_CRUISE_MS            1500    // 巡航を保つ時間
#define NOTICE_WIDE_MS              1200    // 気づいたとき目を見開いている時間
#define NOTICE_HOLD_MS              800     // 障害物がなくなってから、再発進を考え始めるまでの間
#define NOTICE_STOP_REPORT_MS       800     // 気づいてから、停止後の距離をシリアルに出すまでの待ち

#endif // NOVA_CONFIG_H
