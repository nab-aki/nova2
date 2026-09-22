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
#define MOTOR_PWM_MIN_BACKWARD      650     // 後退の最低PWM。実測：動き出し 575 × 1.12 ≒ 644 → 650
                                            // 参考値。正規化速度の換算は前後とも MOTOR_PWM_MIN(675) を下限にするが、
                                            // 675 > 650 なので後退も必ず動く（ID15 の後退はこの換算のまま使う）
#define MOTOR_PWM_LIMIT             2000    // 安全のための上限（tools/pwm_test と同じ）
#define MOTOR_SPEED_EPSILON         0.01f   // 正規化速度（-1〜1）がこれ未満なら停止として扱う

// その場回転・片側旋回（ID25）は、4輪の横滑りのため動き出しに大きな力が要る。ただし回り出せば下げても回り続ける。
// そこで「動き出し（キック）」のPWMを短時間出してから、「回り続ける」PWMに下げる。
// 値は正規化速度ではなく生のPWM。Motor_DrivePwm() で出し、段階の管理は core/motion.* が行う。
// 【一旦の決定値（2026-09-21）】床（フローリング）で、デバッグの調整キー（q a w s e d r f）を使って探した値。
// 最低PWMの実測＋余裕（その場回転 キック1150・保持1000、片側旋回 キック1000・保持900、キック300ms）では、
// 1ステップ500ms で その場回転は約6°、片側旋回はほとんど回らなかったため、時間・キック・PWMを上げた。
// 1ステップの角度：その場回転 左右とも約30°、片側旋回 左右とも約20°（docs/measurements.md）。
// 5分間の試験で動きの印象を見て見直す。見直し候補：片側旋回は保持（1450）がキック（1400）より大きく、
// キックの意味が薄い（実質 1450 一定）。
#define MOTOR_ROTATE_KICK_PWM       1600    // その場回転の動き出し（一旦の決定値）
#define MOTOR_ROTATE_HOLD_PWM       1000    // その場回転の回り続ける値（同上）
#define MOTOR_PIVOT_KICK_PWM        1400    // 片側旋回の動き出し（同上。保持のほうが大きい点は見直し候補）
#define MOTOR_PIVOT_HOLD_PWM        1450    // 片側旋回の回り続ける値（同上）
// 【2026-09-22 変更】1000→300ms。満充電での5分間試験で、後退+その場回転（張りつき対策）のあと
// ブラウンアウト（電圧低下によるリセット）が発生した。キック PWM1600 を1000msも出し続けるのが
// 電流の谷を深くしていると見て、キックの時間だけを短くする（PWM・1ステップの長さ1300msは変えない）。
// 回転角が小さくなる可能性があるが、見直しは「片側旋回の値の見直し」（decisions.md）でまとめて行う。
#define MOTOR_ROTATE_KICK_MS        300     // その場回転のキックを出す時間（1000から変更。ブラウンアウト対策）
#define MOTOR_PIVOT_KICK_MS         300     // 片側旋回のキックを出す時間（同上）

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

// ------------------------ IR受信（リモコン）------------------------ //
#define IR_DEBOUNCE_MS              500     // 同じコードがこの時間以内に続いたら、押しっぱなし（または二重受信）とみなして捨てる
#define IR_BUTTON_PAUSE             0xFFA857  // 一時停止・再開に割り当てるボタン（▶。Freenove のリモコン。docs/specs/common_ir.md）

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

// ------------------------ 安全層：持ち上げ ------------------------ //
// 実測：床では 000、持ち上げると 111（docs/measurements.md）。
// 止めるのは1回で即座に、動いてよくするのは 111 以外が続いてから、と非対称にする
// （宙に浮いたまま車輪が回るのを避けるため）。数えるのは「読み取った回数」で、
// ライントラッキングは TRACK_READ_INTERVAL_MS ごとにしか読まない。
#define SAFETY_LIFT_TRACK           0x07    // 111（左・中央・右すべて）
#define SAFETY_LIFT_CLEAR_COUNT     3       // 床に戻ったと認めるまでの連続読み取り回数（約300ms）

// ------------------------ 調停の優先度 ------------------------ //
// 大きいほど優先。レイヤー（車体／目）ごとに比べるので、車体と目で同じ数値があってもよい。
// 車体：デバッグ回転 > 困る > うろうろの後退+その場回転 > 気づく > うろうろ（安全 > 立て直し > 張りつき対策 > 反応 > 気まま）。
// 数値の基準は docs/decisions.md に積み上げる。
#define PRIORITY_WANDER             20      // 車体：ID25 うろうろ（既定の振る舞い）
#define PRIORITY_NOTICE             30      // 車体：ID9 気づく（20 から変更）
#define PRIORITY_WANDER_RECOVER     35      // 車体：ID25 の後退+その場回転（横がとても近いときの張りつき対策）。
                                            // ID15 の立て直しと同じく、気づく（30）に割り込まれず最後までやり切る
#define PRIORITY_TROUBLE            40      // 車体：ID15 障害物で困る
#define PRIORITY_DEBUG_TURN         50      // 車体：回転角を測るデバッグキー（3〜6）

// ------------------------ デバッグ ------------------------ //
// 1 にすると、起動直後から「一時停止」の状態で始める（回転角を測るとき用。p キーで再開）。
// 0 が既定（今までどおり、起動したらすぐうろうろを始める）。
// 【いまは 1】回転角の測定中のため。測り終えたら 0 に戻すこと（5分間の試験の前など）。
#define DEBUG_START_PAUSED          1
#define DEBUG_KEY_REPEAT_MS         250     // 同じキーがこれより短い間隔で続いたら、押しっぱなしのリピートとみなして捨てる
#define DEBUG_TUNE_REPEAT_MS        60      // 回転の調整キー（q a w s e d r f）用。連続で押せるよう短くしてある
// 回転の調整キー（一時停止中だけ効く）の刻みと範囲。値は実行時の変更で、書き込み直すと元に戻る
#define DEBUG_TUNE_STEP_MS_MIN      100     // 1ステップの時間
#define DEBUG_TUNE_STEP_MS_MAX      3000
#define DEBUG_TUNE_STEP_MS_STEP     100
#define DEBUG_TUNE_KICK_MS_MIN      0       // キックの時間（0 ならキックなしで保持PWMから始める）
#define DEBUG_TUNE_KICK_MS_MAX      2000
#define DEBUG_TUNE_KICK_MS_STEP     50
#define DEBUG_TUNE_PWM_MIN          500     // キック・保持のPWM（実測の最低は片側旋回 750。下も試せるよう広げた）
#define DEBUG_TUNE_PWM_MAX          MOTOR_PWM_LIMIT
#define DEBUG_TUNE_PWM_STEP         50
#define PRIORITY_IDLE_BLINK         10      // 目：何もなければまばたきする
#define PRIORITY_NOTICE_EYES        30      // 目：ID9 気づいて見開く（まばたきより優先）
#define PRIORITY_PAUSE_CUE          40      // 目：一時停止・再開の合図（リモコンの ▶ や p キー）

// ------------------------ 一時停止・再開の合図（目）------------------------ //
#define PAUSE_CUE_PAUSE_MS          1000    // 一時停止：目を細めている時間
#define PAUSE_CUE_RESUME_HALF_MS    150     // 再開：ゆっくり閉じて開く（細める→閉じる→細める）の、細めている時間（前後それぞれ）
#define PAUSE_CUE_RESUME_CLOSED_MS  200     // 同、閉じている時間（合計 150+200+150 = 500ms）

// ------------------------ ID9 気づく ------------------------ //
// 走行ループ（停止→加速→巡航→減速）は ID25 に移したので、NOTICE_REST_MS・NOTICE_CRUISE_MS は
// 使わなくなった（ID25 の WANDER_* が対応する）。
#define NOTICE_WIDE_MS              1200    // 気づいたとき目を見開いている時間
#define NOTICE_HOLD_MS              800     // 障害物がなくなってから、うろうろに戻るまでの間
#define NOTICE_STOP_REPORT_MS       800     // 気づいてから、停止後の距離をシリアルに出すまでの待ち
#define NOTICE_REACT_MS             1200    // 気づいた反応が終わるまで（この後 ID15 が引き継ぐ）。
                                            // 見開き 1200ms と停止後の報告 800ms の長いほう

// ------------------------ ID25 うろうろ ------------------------ //
// 見回しの角度：servo1 は 20〜140°・正面 84° なので、左右対称に取れるのは ±56° まで。
// 実測の安定時間（±55°で最長160ms）にも収まる ±50° にした（docs/specs/25_wander.md）。
#define WANDER_REST_MS              800     // 止まってから見回しを始めるまでの「ため」
#define WANDER_SCAN_PAN_DEG         50      // 左右を見る角度（正面±）。servo1 は 34〜134°
#define WANDER_SCAN_SAMPLES         3       // 1方向あたりの測距回数（最も近い値を採る）
#define WANDER_SIDE_NEAR_CM         40.0f   // 横がこれより近ければ、反対側へ向きを変える
                                            // 斜め50°の距離なので、壁が平行なら横の実距離は 40×sin50°≒31cm
// 【仮の値（2026-09-22）】5分間試験で、浅い角度の壁に片側旋回が止められ続けて張りつく問題が見つかった
// （旋回は車体が前へふくらむので、壁までの余裕が少ないと安全層に止められやすい）。
// 正面の停止閾値 OBSTACLE_STOP_CM（36cm）より内側の 25cm を「とても近い」とし、
// このときは片側旋回ではなく、後退してその場回転（安全層に止められない）で避ける。
// WANDER_SIDE_NEAR_CM（40cm）との間に余裕を持たせてある。実機で見回しのログ（横の距離）を見ながら詰める。
#define WANDER_SIDE_VERY_NEAR_CM    25.0f   // 横がこれより近ければ、片側旋回でなく後退+その場回転で避ける（仮の値）
// 【仮の値（2026-09-22）】満充電でも、後退の直後にその場回転を始めるとブラウンアウトが起きた
// （その場回転のキックを1000→300msにしても再発。decisions.md）。なめらか加減速の出力が0になっても
// 車体は慣性で少し動き続けている可能性があり、間を置かず逆向きの回転（キック）を重ねると
// 大電流になると見て、後退が止まってから回転を始めるまでに「ため」を挟む。ID15はこの間に
// 左右確認（数百ms、モーター出力0）が自然に入るため同じ問題が出ていなかった。
#define WANDER_RECOVER_PAUSE_MS     500     // 後退→回転の間の「ため」（仮の値。実機でブラウンアウトが
                                            // 再発しないか見ながら詰める）
#define WANDER_AVOID_PIVOT_MS       1200    // 横を避けるための片側旋回の時間（うちキック MOTOR_PIVOT_KICK_MS 300ms。約20°。一旦の決定値）
#define WANDER_RUN_MIN_MS           1000    // 巡航を保つ時間（ランダムの下限）
#define WANDER_RUN_MAX_MS           3000    // 同（上限）

// ------------------------ ID15 障害物で困る ------------------------ //
#define TROUBLE_BACK_SPEED          (-CRUISE_SPEED)  // 後退の速度（巡航と同じ速さで逆向き）
#define TROUBLE_BACK_MS             500     // 後退する時間（推定10〜15cm。実機で測って詰める）
#define TROUBLE_BACK_RAMP_MS        300     // 後退の加速・減速にかける時間
#define TROUBLE_SIDE_DIFF_CM        10.0f   // 左右の差がこれ未満なら「甲乙つけがたい」
#define TROUBLE_CLEAR_CM            46.0f   // 正面が「空いた」と認める距離。
                                            // OBSTACLE_STOP_CM + OBSTACLE_CLEAR_MARGIN_CM と同じ値にしてある
                                            // （ずらすと ID25 に戻れなくなる。docs/specs/15_trouble.md）
#define TROUBLE_TURN_STEP_MS        1300    // 1回に回る時間（うちキック MOTOR_ROTATE_KICK_MS 1000ms・保持300ms。約30°。一旦の決定値）
#define TROUBLE_CHECK_SETTLE_MS     200     // 回転を止めてから測り直すまでの待ち（車体の揺れが収まるまで）
#define TROUBLE_MAX_STEPS           12      // 「一周した」とみなす回転の回数。360° ÷ 30°/ステップ（実測）＝ 12
#define TROUBLE_GIVEUP_REST_MS      5000    // 一周しても空かないときに休む時間

// ------------------------ 試験の集計（共通部品。NVS保存）------------------------ //
// 「1回の試験」＝再開してから一時停止するまで（起動時に一時停止でなければ起動から）。
// 直近3回分だけ NVS（フラッシュ）に保存する。走行中は書かない。
#define TEST_STATS_SAVE_INTERVAL_MS   30000  // 止まっている間、フラッシュへ保存し直す間隔
#define TEST_STATS_CLEAR_CONFIRM_MS   5000   // x キーの確定待ち時間（この間にもう一度 x で消去）

#endif // NOVA_CONFIG_H
