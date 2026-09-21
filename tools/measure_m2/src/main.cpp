// Freenove 4WD Car Kit for ESP32（FNK0053）
// M2案（首に目＝LEDマトリクスと超音波を搭載）の測定ツール（本体プロジェクトとは独立）
//
// モーター駆動・サーボ駆動・超音波測距・電池電圧読み取りの定義は、Freenove公式リポジトリの
// サンプルスケッチに従っている（tools/pwm_test と同じ）。
//   https://github.com/Freenove/Freenove_4WD_Car_Kit_for_ESP32
//   Sketches/01.1_Car_Move_and_Turn   （モーター制御）
//   Sketches/01.2_Servo               （サーボ制御）
//   Sketches/01.4_Battery_level       （電池電圧読み取り）
//   Sketches/02.1_Ultrasonic_Ranging  （超音波測距）
//
// シリアル（115200bps）からの1文字コマンドで、次の5つを測る。
// ・水平角度の探索：servo2 を1°ずつ動かし、現在角度と距離を表示する
// ・首の安定時間：首を左右に振って正面に戻した直後から、20ms間隔で1秒間測距する（5回）
// ・測距のばらつき：20回測って 最小・最大・平均・中央値・標準偏差 を出す
// ・停止距離：指定PWMで前進し、正面の障害物が閾値に達したら即停止して、停止までの滑走距離を測る
// ・最低PWM：前進・後退・その場回転（左右）・片側旋回（左右）を、本体と同じ 50Hz で2秒だけ動かしてすぐ止める。
//   前進・後退は PWM450〜700（25刻み）、回転・片側旋回は PWM700〜2000（50刻み）。
//   前進・後退は前方の壁との距離の変化から、動いたかと速度（cm/s）も出す
// ・回り続ける最低PWM：回転・片側旋回を動かし続けたまま、キーで50ずつ下げて、止まる直前の値を探す
// 結果は、そのまま docs/measurements.md に貼れる表の行（| 項目 | 値 | 条件・備考 | 測定日 |）でも出力する
// （最低PWMは動いたかを目で判断するので、行は出さず、各回の結果の行を貼ってもらう）。
//
// 【走行の安全策】超音波が10cm未満になったら即停止／走行中はどのキーでも即停止／
// 走行中は首を正面・水平に固定／測距が途切れたら停止／一定時間で停止。

#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>
#include <driver/gpio.h>
#include "esp_adc_cal.h"

// ======================== ピン・アドレス（Freenove公式サンプル準拠）======================== //
#define PCA9685_SDA         13     // ESP32側 I2C SDA
#define PCA9685_SCL         14     // ESP32側 I2C SCL
#define PCA9685_ADDRESS     0x5F
#define PIN_MOTOR_M1_IN1    15
#define PIN_MOTOR_M1_IN2    14
#define PIN_MOTOR_M2_IN1    9
#define PIN_MOTOR_M2_IN2    8
#define PIN_MOTOR_M3_IN1    12
#define PIN_MOTOR_M3_IN2    13
#define PIN_MOTOR_M4_IN1    10
#define PIN_MOTOR_M4_IN2    11
#define PCA9685_PWM_MAX     4095   // PCA9685のパルス幅の上限（公式サンプル準拠）
#define PCA9685_CHANNEL_SERVO1  0  // servo1（左右）
#define PCA9685_CHANNEL_SERVO2  1  // servo2（上下）
#define PIN_SONIC_TRIG      12     // 超音波はPCA9685を経由せずGPIOに直結
#define PIN_SONIC_ECHO      15
#define PIN_BATTERY         32
#define DEFAULT_VREF        1100

// ======================== 調整値（測定条件はここで変える）======================== //
// PCA9685 はチップ全体で1つのPWM周波数しか持てない。首を保持したまま走るため、
// 本体プロジェクトと同じ 50Hz に共通化している（docs/decisions.md 参照）。
// 【注意】最低PWM 1300 は 1000Hz での測定値。50Hz で動き出す最低PWMは別途確認が必要。
#define PCA9685_FREQUENCY_HZ    50

// 首サーボ（実測値。docs/measurements.md）
#define SERVO1_FRONT_DEG        84     // servo1（左右）の正面
#define SERVO1_MIN_DEG          20     // servo1 の可動範囲（実測）
#define SERVO1_MAX_DEG          140
#define SERVO2_LEVEL_DEG        98     // servo2（上下）の水平（実測。超音波が水平になる角度）。'l' コマンドで測り直して更新できる
#define SERVO2_MIN_DEG          90     // servo2 の可動範囲（実測）
#define SERVO2_MAX_DEG          140
#define NECK_SETTLE_MS          700    // 首を正面・水平にしてから測距や走行を始めるまでの待ち

// 超音波
#define SONAR_TIMEOUT_MS        18     // エコーが返らないとみなす時間（300cm往復で約17.6ms）
#define SOUND_VELOCITY_M_S      340    // 公式サンプルと同じ

// 水平角度の探索
#define TILT_PROBE_WAIT_MS      400    // 角度を変えてから距離を測り始めるまでの待ち
#define TILT_PROBE_COUNT        5      // 距離の中央値を出すための測距回数
#define TILT_PROBE_INTERVAL_MS  60

// 首の安定時間
#define SETTLE_TRIALS           5      // 繰り返し回数
#define SETTLE_INTERVAL_MS      20     // 測距間隔
#define SETTLE_WINDOW_MS        1000   // 正面に戻した直後から測る時間
#define SETTLE_SLOTS            (SETTLE_WINDOW_MS / SETTLE_INTERVAL_MS)
#define SWING_DWELL_MS          600    // 左右に振ったあと、次へ動かすまでの待ち（首が着くのを待つ）
#define SETTLE_FINAL_SAMPLES    10     // 「最終値」を決めるために使う、終盤のサンプル数
#define SETTLE_FINAL_MIN_VALID  5      // 最終値を決めるのに必要な有効サンプル数
#define SETTLE_TOLERANCE_CM     1.0f   // 最終値からこの範囲内なら「安定」とみなす
#define SETTLE_HOLD_SAMPLES     5      // 「安定」とみなすのに必要な連続サンプル数（20ms×5=100ms）

// 測距のばらつき
#define VARIANCE_COUNT          20     // 測距回数（COLLECT_MAX 以下）
#define VARIANCE_INTERVAL_MS    60

// 停止距離
#define DRIVE_PWM_DEFAULT       1650   // 初期PWM（本体の巡航 0.5 相当）
#define DRIVE_PWM_LIMIT         2000   // 安全のためのPWM上限（tools/pwm_test と同じ）
#define DRIVE_PWM_STEP          50
#define STOP_THRESHOLD_DEFAULT_CM  30  // 初期の停止閾値
#define STOP_THRESHOLD_MIN_CM   15     // 非常停止（10cm）と区別できる下限
#define STOP_THRESHOLD_MAX_CM   80     // 安定して測れる最大距離（実測）
#define STOP_THRESHOLD_STEP_CM  5
#define SAFETY_STOP_CM          10.0f  // 走行中にこれ未満が測れたら即停止
#define DRIVE_ARM_MS            700    // 首を固定して、キー入力を待つ準備時間（この間はモーターは動かない）
#define DRIVE_RAMP_MS           500    // 0 から指定PWMまでの加速時間（急発進しない）
#define DRIVE_MOTOR_INTERVAL_MS 20     // 加速中にモーター出力を更新する間隔
#define DRIVE_RANGE_INTERVAL_MS 30     // 走行中・停止後の測距間隔
#define DRIVE_STALE_MS          300    // 有効な測距がこの時間途切れたら停止
#define DRIVE_MAX_MS            8000   // この時間走っても止まらなければ停止
#define DRIVE_BATTERY_LOAD_MS   300    // 走行開始からこの時間後に、負荷中の電池電圧を読む
#define DRIVE_STOP_WINDOW       5      // 完全停止の判定に使う連続測距の数
#define DRIVE_STOP_TOLERANCE_CM 0.5f   // 連続した測距値がこの範囲内なら停止したとみなす
#define DRIVE_STOP_MIN_MS       200    // 停止命令からこの時間は「停止」と判定しない
#define DRIVE_STOP_MAX_MS       2500   // 停止命令からこの時間で判定を打ち切る

// 最低PWM（前進・後退・回転・片側旋回で、50Hz で動き出す最低のPWMを探す）
// 前進・後退：700 は前進で動くことを実測済み。回転・片側旋回：700 では回り始めなかったため 2000 まで広げた
#define MINPWM_LINEAR_LO        450    // 前進・後退で試すPWMの範囲
#define MINPWM_LINEAR_HI        700
#define MINPWM_LINEAR_STEP      25
#define MINPWM_TURN_LO          700    // 回転・片側旋回で試すPWMの範囲（上限は DRIVE_PWM_LIMIT と同じ）
#define MINPWM_TURN_HI          2000
#define MINPWM_TURN_STEP        50
#define MINPWM_RUN_MS           2000   // 各PWMで動かす時間（すぐ止まる）
#define MINPWM_REST_MS          800    // 停止後、車体が止まりきるまで待って最終の距離を測る時間
#define MINPWM_ARM_MS           700    // 首を固定して、キー入力を待つ準備時間（この間はモーターは動かない）
#define MINPWM_SPEED_FROM_MS    500    // 速度は、動かし始めてこの時間以降の測距から求める（立ち上がりを除く）
#define MINPWM_SPEED_SPAN_MS    400    // 速度を求めるのに必要な、測距の最小の時間幅
#define MINPWM_MOVED_CM         2.0f   // 前方の距離がこれ以上変わったら「前進／後退した」目安（σ0.2cm以下の実測より十分大きい）
#define MINPWM_SAMPLES_MAX      96     // 1回の走行で記録する測距の最大数（30ms間隔で約2.9秒）

// 回り続ける最低PWM（回転・片側旋回を動かし続けたまま、PWMをキーで変えて、止まる直前の値を探す）
#define HOLD_STEP               50     // 1回のキーで変えるPWM
#define HOLD_PWM_MIN            100    // 下げられる下限
#define HOLD_IDLE_MS            20000  // キー入力がこの時間なければ自動停止

#define COLLECT_MAX             20     // ばらつき測定の最大サンプル数

// ======================== モーター・首サーボ（PCA9685経由）======================== //
PCA9685 pca9685;

// PCA9685の全体アドレス（MODE1レジスタ）を初期化する
// （公式サンプル Sketches/01.4_Battery_level の PCA9685_Close_Com_Address と同じ処理）
void PCA9685_Close_Com_Address(void) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(0x00);
  Wire.write(0x00);
  Wire.endTransmission();
}

void Pca9685_Setup(void) {
  Wire.begin(PCA9685_SDA, PCA9685_SCL);
  pca9685.setupSingleDevice(Wire, PCA9685_ADDRESS);
  PCA9685_Close_Com_Address();
  pca9685.setToFrequency(PCA9685_FREQUENCY_HZ);
}

// 1輪分の出力。正転は IN1、逆転は IN2 にPWMを出し、もう一方は0にする（pwm の符号で向きが決まる）
static void SetWheel(uint8_t chIn1, uint8_t chIn2, int pwm) {
  if (pwm >= 0) {
    pca9685.setChannelPulseWidth(chIn1, pwm);
    pca9685.setChannelPulseWidth(chIn2, 0);
  } else {
    pca9685.setChannelPulseWidth(chIn1, 0);
    pca9685.setChannelPulseWidth(chIn2, -pwm);
  }
}

static int lastLeftPwm = 0;    // 直前に出力した左側（M1・M2）のPWM（同じ値ならI2Cへ書き込まない）
static int lastRightPwm = 0;   // 同・右側（M3・M4）。前進が正、後退が負

// 左側（M1・M2）と右側（M3・M4）を別々のPWM（符号付き）で駆動する。
// 前進 (+p,+p)、後退 (-p,-p)、左回転 (-p,+p)、右回転 (+p,-p)
void Motor_Move(int left, int right) {
  left = constrain(left, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
  right = constrain(right, -DRIVE_PWM_LIMIT, DRIVE_PWM_LIMIT);
  if (left == lastLeftPwm && right == lastRightPwm) return;
  lastLeftPwm = left;
  lastRightPwm = right;
  SetWheel(PIN_MOTOR_M1_IN1, PIN_MOTOR_M1_IN2, left);
  SetWheel(PIN_MOTOR_M2_IN1, PIN_MOTOR_M2_IN2, left);
  SetWheel(PIN_MOTOR_M3_IN1, PIN_MOTOR_M3_IN2, right);
  SetWheel(PIN_MOTOR_M4_IN1, PIN_MOTOR_M4_IN2, right);
}

// 4輪を同じPWMで前進させる
void Motor_Forward(int pwm) {
  Motor_Move(pwm, pwm);
}

// 停止。安全のため、直前の値に関わらず必ず書き込む
void Motor_Stop(void) {
  lastLeftPwm = 0;
  lastRightPwm = 0;
  SetWheel(PIN_MOTOR_M1_IN1, PIN_MOTOR_M1_IN2, 0);
  SetWheel(PIN_MOTOR_M2_IN1, PIN_MOTOR_M2_IN2, 0);
  SetWheel(PIN_MOTOR_M3_IN1, PIN_MOTOR_M3_IN2, 0);
  SetWheel(PIN_MOTOR_M4_IN1, PIN_MOTOR_M4_IN2, 0);
}

// 角度（0〜180°）をPCA9685のパルス幅に変換して出力する（公式サンプルと同じ map(0-180 -> 102-512)、50Hz動作）
static void Servo_Write(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  pca9685.setChannelPulseWidth(channel, map(angle, 0, 180, 102, 512));
}

static int servo1Angle = SERVO1_FRONT_DEG;
static int servo2Angle = SERVO2_LEVEL_DEG;
static int tiltLevelDeg = SERVO2_LEVEL_DEG;  // 水平とみなす servo2 の角度（'l' で更新）

void Neck_SetPan(int deg) {
  servo1Angle = constrain(deg, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
  Servo_Write(PCA9685_CHANNEL_SERVO1, servo1Angle);
}

void Neck_SetTilt(int deg) {
  servo2Angle = constrain(deg, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
  Servo_Write(PCA9685_CHANNEL_SERVO2, servo2Angle);
}

// 首を正面・水平にする
void Neck_Front(void) {
  Neck_SetPan(SERVO1_FRONT_DEG);
  Neck_SetTilt(tiltLevelDeg);
}

// ======================== 電池電圧（公式サンプル準拠）======================== //
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_atten_t atten = ADC_ATTEN_DB_12;
static const adc_unit_t unit = ADC_UNIT_1;
static const float batteryCoefficient = 4;  // 分圧比（公式サンプルの初期値）

void Battery_Setup(void) {
  analogSetWidth(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  adc_chars = (esp_adc_cal_characteristics_t *)calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

float Get_Battery_Voltage(void) {
  long batteryADC = 0;
  for (int i = 0; i < 5; i++) {
    batteryADC += analogRead(PIN_BATTERY);
  }
  uint32_t voltage_at_pin_mv = esp_adc_cal_raw_to_voltage(batteryADC / 5, adc_chars);
  return (voltage_at_pin_mv / 1000.0f) * batteryCoefficient;
}

// ======================== 超音波（割り込み方式のノンブロッキング測距）======================== //
// pulseIn は最大数十msループを止め、「どのキーでも即停止」の反応を遅らせるため使わない。
// 測距値は 公式サンプル 02.1_Ultrasonic_Ranging と同じ計算（μs × 340 / 2 / 10000 = cm）。
struct SonarSample {
  bool valid;             // false = 反応なし
  float cm;
  unsigned long trigMs;   // トリガーを出した時刻
};

static volatile unsigned long echoRiseUs = 0;
static volatile unsigned long echoFallUs = 0;
static volatile bool echoRose = false;
static volatile bool echoDone = false;

static bool sonarSeriesOn = false;
static unsigned long sonarIntervalMs = 0;
static unsigned long sonarNextMs = 0;
static bool sonarBusy = false;
static bool sonarDiscard = false;   // 系列を止めたとき、測距中のものは結果を捨てる
static unsigned long sonarTrigMs = 0;

// エコー信号の変化ごとに時刻を記録する
static void IRAM_ATTR OnEchoChange(void) {
  if (gpio_get_level((gpio_num_t)PIN_SONIC_ECHO)) {
    echoRiseUs = micros();
    echoRose = true;
  } else if (echoRose) {
    echoFallUs = micros();
    echoDone = true;
  }
}

void Sonar_Setup(void) {
  pinMode(PIN_SONIC_TRIG, OUTPUT);
  digitalWrite(PIN_SONIC_TRIG, LOW);
  pinMode(PIN_SONIC_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SONIC_ECHO), OnEchoChange, CHANGE);
}

// 一定間隔での連続測距を始める。最初の測距はすぐ行う
void Sonar_StartSeries(unsigned long intervalMs, unsigned long now) {
  sonarSeriesOn = true;
  sonarIntervalMs = intervalMs;
  sonarNextMs = now;
}

void Sonar_StopSeries(void) {
  sonarSeriesOn = false;
  if (sonarBusy) sonarDiscard = true;
}

// 毎ループ呼ぶ。測距が1回終わったら true を返し、結果を out に入れる
bool Sonar_Tick(unsigned long now, SonarSample *out) {
  if (sonarBusy) {
    bool finished = false;
    out->valid = false;
    out->cm = 0.0f;
    out->trigMs = sonarTrigMs;
    if (echoDone) {
      unsigned long pulseUs = echoFallUs - echoRiseUs;
      if (pulseUs > 0) {
        out->valid = true;
        out->cm = (float)pulseUs * SOUND_VELOCITY_M_S / 2 / 10000.0f;
      }
      finished = true;
    } else if (now - sonarTrigMs >= SONAR_TIMEOUT_MS) {
      finished = true;
    }
    if (!finished) return false;
    sonarBusy = false;
    if (sonarDiscard) {
      sonarDiscard = false;
      return false;
    }
    return true;
  }

  if (sonarSeriesOn && (long)(now - sonarNextMs) >= 0) {
    echoRose = false;
    echoDone = false;
    // digitalWrite間の10μs待ちはHC-SR04のトリガー生成に必要な最小待ち時間で、delay()には該当しない
    digitalWrite(PIN_SONIC_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SONIC_TRIG, LOW);
    sonarBusy = true;
    sonarTrigMs = now;
    sonarNextMs += sonarIntervalMs;
    if ((long)(now - sonarNextMs) >= 0) sonarNextMs = now + sonarIntervalMs;  // 遅れたら詰め直す
  }
  return false;
}

// ======================== 統計 ======================== //
struct Stats {
  int validCount;
  float minV, maxV, mean, median, sd;
};

// 昇順に並べ替えて中央値を返す（配列は破壊される）
static float MedianInPlace(float *a, int n) {
  for (int i = 1; i < n; i++) {
    float key = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > key) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = key;
  }
  return (n % 2 == 1) ? a[n / 2] : (a[n / 2 - 1] + a[n / 2]) / 2.0f;
}

// 反応なし（NAN）を除いて統計を出す。sd は標本標準偏差（n-1）
static Stats ComputeStats(const float *raw, int n) {
  Stats s = {0, 0, 0, 0, 0, 0};
  float tmp[COLLECT_MAX];
  double sum = 0;
  for (int i = 0; i < n && s.validCount < COLLECT_MAX; i++) {
    if (isnan(raw[i])) continue;
    tmp[s.validCount++] = raw[i];
    sum += raw[i];
  }
  if (s.validCount == 0) return s;
  s.mean = (float)(sum / s.validCount);
  double sq = 0;
  s.minV = s.maxV = tmp[0];
  for (int i = 0; i < s.validCount; i++) {
    if (tmp[i] < s.minV) s.minV = tmp[i];
    if (tmp[i] > s.maxV) s.maxV = tmp[i];
    sq += (tmp[i] - s.mean) * (tmp[i] - s.mean);
  }
  s.sd = (s.validCount > 1) ? (float)sqrt(sq / (s.validCount - 1)) : 0.0f;
  s.median = MedianInPlace(tmp, s.validCount);
  return s;
}

// ======================== 測定条件の設定 ======================== //
static const char *const TARGET_NAMES[] = {"手", "服", "壁 0°", "壁 30°", "壁 45°"};
static const int TARGET_COUNT = sizeof(TARGET_NAMES) / sizeof(TARGET_NAMES[0]);
static const int TARGET_DISTANCES_CM[] = {10, 30, 60};
static const int TARGET_DISTANCE_COUNT = sizeof(TARGET_DISTANCES_CM) / sizeof(TARGET_DISTANCES_CM[0]);
static const int SWING_AMPLITUDES_DEG[] = {20, 40, 55};   // 正面から左右へ振る幅（55°は可動範囲の限界近く）
static const int SWING_AMPLITUDE_COUNT = sizeof(SWING_AMPLITUDES_DEG) / sizeof(SWING_AMPLITUDES_DEG[0]);

static int targetIndex = 0;
static int distanceIndex = 1;
static int amplitudeIndex = 1;
static int drivePwm = DRIVE_PWM_DEFAULT;
static int stopThresholdCm = STOP_THRESHOLD_DEFAULT_CM;

// 最低PWM測定の設定。モードは 前進・後退・左回転・右回転・左片側旋回・右片側旋回（'1'〜'6' で選ぶ）
// 片側旋回：片側の車輪だけを前進させ、もう片側は止める（0）。左片側旋回は右輪だけ前進して左へ曲がる
enum MinPwmMode { MP_FORWARD, MP_BACKWARD, MP_ROTATE_LEFT, MP_ROTATE_RIGHT, MP_PIVOT_LEFT, MP_PIVOT_RIGHT };

// 前進・後退は前方の壁との距離で動きを見られる。回転・片側旋回は目で見る
static bool MinPwmModeIsLinear(MinPwmMode m) {
  return m == MP_FORWARD || m == MP_BACKWARD;
}
static int MinPwmLo(MinPwmMode m)   { return MinPwmModeIsLinear(m) ? MINPWM_LINEAR_LO : MINPWM_TURN_LO; }
static int MinPwmHi(MinPwmMode m)   { return MinPwmModeIsLinear(m) ? MINPWM_LINEAR_HI : MINPWM_TURN_HI; }
static int MinPwmStep(MinPwmMode m) { return MinPwmModeIsLinear(m) ? MINPWM_LINEAR_STEP : MINPWM_TURN_STEP; }

static MinPwmMode minPwmMode = MP_FORWARD;
static int minPwm = MINPWM_LINEAR_LO;

static bool TargetIsWall(void) {
  return targetIndex >= 2;
}

// ======================== 実行中の測定（同時に1つだけ）======================== //
enum Job { JOB_IDLE, JOB_SETTLE, JOB_COLLECT, JOB_DRIVE, JOB_MINPWM, JOB_HOLD };
static Job job = JOB_IDLE;
static unsigned long phaseMs = 0;   // 現在のフェーズを始めた時刻

static void Job_Finish(void) {
  Sonar_StopSeries();
  job = JOB_IDLE;
}

// 受信バッファに溜まった入力を捨てる（走行を止めたあと、溜まったキーで次の操作が始まらないように）
static void FlushInput(void) {
  while (Serial.available() > 0) Serial.read();
}

// ------------------------ 水平角度の探索・測距のばらつき（複数回測って統計を出す）------------------------ //
enum CollectPurpose { COLLECT_TILT, COLLECT_VARIANCE };
enum CollectPhase { CP_WAIT, CP_MEASURE };

static CollectPurpose collectPurpose = COLLECT_TILT;
static CollectPhase collectPhase = CP_WAIT;
static int collectTarget = 0;
static unsigned long collectWaitMs = 0;
static unsigned long collectIntervalMs = 0;
static unsigned long collectStartMs = 0;
static int collectTotal = 0;
static float collectRaw[COLLECT_MAX];   // 測距値（cm）。反応なしは NAN

static void Collect_Begin(CollectPurpose purpose, int count, unsigned long waitMs, unsigned long intervalMs) {
  Sonar_StopSeries();
  job = JOB_COLLECT;
  collectPurpose = purpose;
  collectPhase = CP_WAIT;
  collectTarget = min(count, COLLECT_MAX);
  collectWaitMs = waitMs;
  collectIntervalMs = intervalMs;
  collectTotal = 0;
  phaseMs = millis();
}

// servo2 を delta° 動かし、角度を表示する。少し待ってから距離の中央値も表示する
static void Tilt_Step(int delta) {
  int next = constrain(servo2Angle + delta, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
  if (next == servo2Angle) {
    Serial.printf("servo2 は可動範囲の端です（%d〜%d°）\n", SERVO2_MIN_DEG, SERVO2_MAX_DEG);
  } else {
    Neck_SetTilt(next);
  }
  Serial.printf("servo2=%d°（servo1=%d° 水平角の設定=%d°）\n", servo2Angle, servo1Angle, tiltLevelDeg);
  Collect_Begin(COLLECT_TILT, TILT_PROBE_COUNT, TILT_PROBE_WAIT_MS, TILT_PROBE_INTERVAL_MS);
}

static void Variance_Begin(void) {
  Neck_Front();
  Serial.printf("[ばらつき] 対象:%s 想定距離:%dcm  首を正面・水平にします（servo1=%d° servo2=%d°）\n",
                TARGET_NAMES[targetIndex], TARGET_DISTANCES_CM[distanceIndex], servo1Angle, servo2Angle);
  Serial.printf("  %dms間隔で%d回測ります（キー入力で中断）\n", VARIANCE_INTERVAL_MS, VARIANCE_COUNT);
  Collect_Begin(COLLECT_VARIANCE, VARIANCE_COUNT, NECK_SETTLE_MS, VARIANCE_INTERVAL_MS);
}

static void PrintRawLine(const float *raw, int n) {
  Serial.print("  生データ(cm、--=反応なし):");
  for (int i = 0; i < n; i++) {
    if (isnan(raw[i])) {
      Serial.print(" --");
    } else {
      Serial.printf(" %.1f", raw[i]);
    }
  }
  Serial.println();
}

static void Collect_ReportTilt(void) {
  Stats s = ComputeStats(collectRaw, collectTotal);
  if (s.validCount == 0) {
    Serial.printf("  距離: 有効なデータなし（%d回とも反応なし）\n", collectTotal);
    return;
  }
  Serial.printf("  距離 中央値 %.1fcm（平均 %.1f 最小 %.1f〜最大 %.1f 有効 %d/%d）\n",
                s.median, s.mean, s.minV, s.maxV, s.validCount, collectTotal);
}

static void Collect_ReportVariance(void) {
  Stats s = ComputeStats(collectRaw, collectTotal);
  int nominalCm = TARGET_DISTANCES_CM[distanceIndex];
  const char *target = TARGET_NAMES[targetIndex];

  Serial.printf("[ばらつき結果] 対象:%s 想定距離:%dcm servo1=%d° servo2=%d° %dms間隔×%d回\n",
                target, nominalCm, servo1Angle, servo2Angle, VARIANCE_INTERVAL_MS, collectTotal);
  PrintRawLine(collectRaw, collectTotal);
  if (s.validCount == 0) {
    Serial.printf("  有効なデータなし（%d回とも反応なし）\n", collectTotal);
    return;
  }
  Serial.printf("  有効 %d/%d  最小 %.1f  最大 %.1f  平均 %.1f  中央値 %.1f  標準偏差 %.2f  (cm)\n",
                s.validCount, collectTotal, s.minV, s.maxV, s.mean, s.median, s.sd);
  Serial.printf("  平均と想定距離の差 %+.1fcm\n", s.mean - nominalCm);

  Serial.println("--- measurements.md 用（この行をそのまま貼れます）---");
  if (TargetIsWall()) {
    Serial.printf("| %s（想定%dcm） ", target, nominalCm);
  } else {
    Serial.printf("| 静止時ばらつき：%s（想定%dcm） ", target, nominalCm);
  }
  Serial.printf("| 平均%.1fcm（中央値%.1f／最小%.1f〜最大%.1f／σ%.2f／有効%d/%d） ",
                s.mean, s.median, s.minV, s.maxV, s.sd, s.validCount, collectTotal);
  Serial.printf("| M2案 servo1=%d° servo2=%d°、%dms間隔×%d回、実距離との差%+.1fcm | |\n",
                servo1Angle, servo2Angle, VARIANCE_INTERVAL_MS, collectTotal, s.mean - nominalCm);
}

static void Collect_Update(unsigned long now, bool hasEv, const SonarSample &ev) {
  if (collectPhase == CP_WAIT) {
    if (now - phaseMs < collectWaitMs) return;
    collectStartMs = now;
    collectTotal = 0;
    collectPhase = CP_MEASURE;
    Sonar_StartSeries(collectIntervalMs, now);
    return;
  }

  if (hasEv && ev.trigMs >= collectStartMs && collectTotal < collectTarget) {
    collectRaw[collectTotal++] = ev.valid ? ev.cm : NAN;
  }
  if (collectTotal < collectTarget) return;

  Job_Finish();
  if (collectPurpose == COLLECT_TILT) {
    Collect_ReportTilt();
  } else {
    Collect_ReportVariance();
  }
}

// ------------------------ 首の安定時間 ------------------------ //
enum SettlePhase { SP_NECK, SP_LEFT, SP_RIGHT, SP_MEASURE };
enum SlotState : uint8_t { SLOT_EMPTY, SLOT_VALID, SLOT_NONE };

static SettlePhase settlePhase = SP_NECK;
static int settleTrial = 0;                      // 実施中の試行（0始まり）
static unsigned long settleT0 = 0;               // 正面に戻す命令を出した時刻（測定の0ms）
static float settleCm[SETTLE_SLOTS];
static SlotState settleState[SETTLE_SLOTS];
static int settleResultMs[SETTLE_TRIALS];        // 試行ごとの安定時間。負の値は判定不可
static float settleFinalCm[SETTLE_TRIALS];
static const int SETTLE_UNSETTLED = -1;          // 1秒以内に安定しなかった
static const int SETTLE_UNKNOWN = -2;            // 終盤の有効値が足りず、最終値を決められない

static void Settle_StartSwing(unsigned long now) {
  int amp = SWING_AMPLITUDES_DEG[amplitudeIndex];
  Serial.printf("[安定時間 %d/%d] 左へ%d° → 右へ%d° → 正面 → 測距%dms\n",
                settleTrial + 1, SETTLE_TRIALS, amp, amp, SETTLE_WINDOW_MS);
  Neck_SetPan(SERVO1_FRONT_DEG - amp);
  settlePhase = SP_LEFT;
  phaseMs = now;
}

static void Settle_Begin(void) {
  int amp = SWING_AMPLITUDES_DEG[amplitudeIndex];
  Neck_Front();
  Serial.printf("[安定時間] 首を正面・水平にして、左右に±%d°振って正面に戻した直後から測ります\n", amp);
  Serial.printf("  %dms間隔で%dms、%d回。正面に戻す命令を出した時点を0msとします（キー入力で中断）\n",
                SETTLE_INTERVAL_MS, SETTLE_WINDOW_MS, SETTLE_TRIALS);
  Serial.println("  正面の壁などの一定距離の対象を用意してください");
  job = JOB_SETTLE;
  settlePhase = SP_NECK;
  settleTrial = 0;
  phaseMs = millis();
}

// 1回分の測定を解析して、安定時間（ms）を返す。負の値は SETTLE_UNSETTLED / SETTLE_UNKNOWN
// 「最終値」＝終盤10サンプルの中央値。「安定」＝最終値±許容内が連続5サンプル続き始めた最初の時刻。
static int Settle_Analyze(float *finalCm, int *validCount, int *outliers) {
  *validCount = 0;
  *outliers = 0;
  float tail[SETTLE_FINAL_SAMPLES];
  int tailCount = 0;
  for (int i = 0; i < SETTLE_SLOTS; i++) {
    if (settleState[i] == SLOT_VALID) (*validCount)++;
    if (i >= SETTLE_SLOTS - SETTLE_FINAL_SAMPLES && settleState[i] == SLOT_VALID) {
      tail[tailCount++] = settleCm[i];
    }
  }
  if (tailCount < SETTLE_FINAL_MIN_VALID) {
    *finalCm = NAN;
    return SETTLE_UNKNOWN;
  }
  *finalCm = MedianInPlace(tail, tailCount);

  for (int k = 0; k + SETTLE_HOLD_SAMPLES <= SETTLE_SLOTS; k++) {
    bool held = true;
    for (int j = k; j < k + SETTLE_HOLD_SAMPLES; j++) {
      if (settleState[j] != SLOT_VALID || fabsf(settleCm[j] - *finalCm) > SETTLE_TOLERANCE_CM) {
        held = false;
        break;
      }
    }
    if (!held) continue;
    for (int j = k + SETTLE_HOLD_SAMPLES; j < SETTLE_SLOTS; j++) {
      if (settleState[j] != SLOT_VALID || fabsf(settleCm[j] - *finalCm) > SETTLE_TOLERANCE_CM) (*outliers)++;
    }
    return k * SETTLE_INTERVAL_MS;
  }
  return SETTLE_UNSETTLED;
}

static void Settle_ReportTrial(void) {
  float finalCm;
  int validCount, outliers;
  int ms = Settle_Analyze(&finalCm, &validCount, &outliers);
  settleResultMs[settleTrial] = ms;
  settleFinalCm[settleTrial] = finalCm;

  Serial.printf("  結果 %d/%d: ", settleTrial + 1, SETTLE_TRIALS);
  if (ms >= 0) {
    Serial.printf("安定まで %dms  最終値 %.1fcm  有効 %d/%d  安定後の外れ値 %d\n",
                  ms, finalCm, validCount, SETTLE_SLOTS, outliers);
  } else if (ms == SETTLE_UNSETTLED) {
    Serial.printf("%dms以内に安定せず  最終値 %.1fcm  有効 %d/%d\n",
                  SETTLE_WINDOW_MS, finalCm, validCount, SETTLE_SLOTS);
  } else {
    Serial.printf("判定不可（終盤の有効値が不足）  有効 %d/%d\n", validCount, SETTLE_SLOTS);
  }

  Serial.printf("  %dms刻みの測距(cm、--=反応なし、..=未測定):", SETTLE_INTERVAL_MS);
  for (int i = 0; i < SETTLE_SLOTS; i++) {
    if (settleState[i] == SLOT_VALID) {
      Serial.printf(" %.1f", settleCm[i]);
    } else {
      Serial.print(settleState[i] == SLOT_NONE ? " --" : " ..");
    }
  }
  Serial.println();
}

static void Settle_ReportAll(void) {
  int amp = SWING_AMPLITUDES_DEG[amplitudeIndex];
  int okCount = 0, minMs = 0, maxMs = 0;
  long sumMs = 0;
  float finalSum = 0;
  for (int i = 0; i < SETTLE_TRIALS; i++) {
    if (settleResultMs[i] < 0) continue;
    if (okCount == 0 || settleResultMs[i] < minMs) minMs = settleResultMs[i];
    if (okCount == 0 || settleResultMs[i] > maxMs) maxMs = settleResultMs[i];
    sumMs += settleResultMs[i];
    finalSum += settleFinalCm[i];
    okCount++;
  }

  Serial.printf("[安定時間まとめ] 振り幅±%d° 許容±%.1fcm 連続%dサンプル(%dms)で安定\n",
                amp, SETTLE_TOLERANCE_CM, SETTLE_HOLD_SAMPLES, SETTLE_HOLD_SAMPLES * SETTLE_INTERVAL_MS);
  Serial.print("  試行ごとの安定時間(ms):");
  for (int i = 0; i < SETTLE_TRIALS; i++) {
    if (settleResultMs[i] >= 0) {
      Serial.printf(" %d", settleResultMs[i]);
    } else {
      Serial.print(settleResultMs[i] == SETTLE_UNSETTLED ? " 安定せず" : " 判定不可");
    }
  }
  Serial.println();
  if (okCount == 0) {
    Serial.println("  安定した試行がありませんでした");
    return;
  }
  Serial.printf("  平均 %ldms  最短 %dms  最長 %dms（%d/%d回で安定）\n",
                sumMs / okCount, minMs, maxMs, okCount, SETTLE_TRIALS);

  Serial.println("--- measurements.md 用（この行をそのまま貼れます）---");
  Serial.printf("| 首を止めてから測距値が安定するまでの時間 | 平均%ldms（最短%d〜最長%dms、%d/%d回） ",
                sumMs / okCount, minMs, maxMs, okCount, SETTLE_TRIALS);
  Serial.printf("| servo1 正面±%d°の左右振り→正面、servo2=%d°、対象の距離%.1fcm、%dms間隔、最終値±%.1fcmが%dms続いたら安定、各回(ms):",
                amp, servo2Angle, finalSum / okCount, SETTLE_INTERVAL_MS, SETTLE_TOLERANCE_CM,
                SETTLE_HOLD_SAMPLES * SETTLE_INTERVAL_MS);
  for (int i = 0; i < SETTLE_TRIALS; i++) {
    if (settleResultMs[i] >= 0) {
      Serial.printf(" %d", settleResultMs[i]);
    } else {
      Serial.print(settleResultMs[i] == SETTLE_UNSETTLED ? " 安定せず" : " 判定不可");
    }
  }
  Serial.println(" | |");
}

static void Settle_Update(unsigned long now, bool hasEv, const SonarSample &ev) {
  int amp = SWING_AMPLITUDES_DEG[amplitudeIndex];

  switch (settlePhase) {
    case SP_NECK:
      if (now - phaseMs >= NECK_SETTLE_MS) Settle_StartSwing(now);
      break;

    case SP_LEFT:
      if (now - phaseMs >= SWING_DWELL_MS) {
        Neck_SetPan(SERVO1_FRONT_DEG + amp);
        settlePhase = SP_RIGHT;
        phaseMs = now;
      }
      break;

    case SP_RIGHT:
      if (now - phaseMs >= SWING_DWELL_MS) {
        // 正面に戻す命令を出した時点を 0ms として、その直後から測距を始める
        for (int i = 0; i < SETTLE_SLOTS; i++) settleState[i] = SLOT_EMPTY;
        Neck_SetPan(SERVO1_FRONT_DEG);
        settleT0 = now;
        Sonar_StartSeries(SETTLE_INTERVAL_MS, now);
        settlePhase = SP_MEASURE;
      }
      break;

    case SP_MEASURE:
      if (hasEv) {
        long offset = (long)(ev.trigMs - settleT0);
        if (offset >= 0 && offset < SETTLE_WINDOW_MS) {
          int slot = constrain((int)((offset + SETTLE_INTERVAL_MS / 2) / SETTLE_INTERVAL_MS), 0, SETTLE_SLOTS - 1);
          settleState[slot] = ev.valid ? SLOT_VALID : SLOT_NONE;
          settleCm[slot] = ev.cm;
        }
      }
      if (now - settleT0 >= SETTLE_WINDOW_MS + SONAR_TIMEOUT_MS) {
        Sonar_StopSeries();
        Settle_ReportTrial();
        settleTrial++;
        if (settleTrial >= SETTLE_TRIALS) {
          Settle_ReportAll();
          Job_Finish();
        } else {
          Settle_StartSwing(now);
        }
      }
      break;
  }
}

// ------------------------ 停止距離（前進して閾値で停止）------------------------ //
enum DrivePhase { DR_ARM, DR_RUN, DR_STOPPING };
enum DriveReason { REASON_THRESHOLD, REASON_EMERGENCY, REASON_KEY, REASON_NO_ECHO, REASON_TIMEOUT };

static const char *DriveReasonName(DriveReason r) {
  switch (r) {
    case REASON_THRESHOLD: return "閾値に到達";
    case REASON_EMERGENCY: return "非常停止（10cm未満）";
    case REASON_KEY:       return "キー入力";
    case REASON_NO_ECHO:   return "測距が途切れた";
    default:               return "走行時間の上限";
  }
}

static DrivePhase drivePhase = DR_ARM;
static DriveReason driveReason = REASON_THRESHOLD;
static unsigned long driveRunStartMs = 0;
static unsigned long driveStopMs = 0;
static unsigned long driveRunMs = 0;                // 走行開始から停止命令までの時間
static unsigned long driveLastValidMs = 0;          // 最後に有効な測距が得られた時刻（0=まだない）
static unsigned long driveLastMotorMs = 0;
static float driveLastCm = 0.0f;                    // 最後の有効な測距値
static float driveCmdCm = 0.0f;                     // 停止命令時の距離
static float driveBatteryIdle = NAN;                // 走行前（無負荷）の電池電圧
static float driveBatteryLoad = NAN;                // 走行中（負荷あり）の電池電圧
static int driveWindowCount = 0;                    // 停止後の連続した有効測距（古い順）
static float driveWindowCm[DRIVE_STOP_WINDOW];
static unsigned long driveWindowMs[DRIVE_STOP_WINDOW];
static int driveRunPwm = 0;                         // この走行で使ったPWM
static int driveRunThresholdCm = 0;                 // この走行で使った閾値

static void Drive_Abort(const char *message) {
  Motor_Stop();
  Job_Finish();
  Serial.printf("[停止距離] 走行しません: %s\n", message);
}

static void Drive_Begin(void) {
  if (drivePwm <= 0) {
    Serial.println("[停止距離] PWMが0のため走行しません（+ でPWMを上げてください）");
    return;
  }
  driveRunPwm = drivePwm;
  driveRunThresholdCm = stopThresholdCm;

  Neck_Front();  // 走行中は首を正面・水平に固定する
  driveBatteryIdle = Get_Battery_Voltage();
  driveBatteryLoad = NAN;
  driveLastValidMs = 0;
  driveLastCm = 0.0f;
  job = JOB_DRIVE;
  drivePhase = DR_ARM;
  phaseMs = millis();
  Sonar_StartSeries(DRIVE_RANGE_INTERVAL_MS, phaseMs);

  Serial.printf("[停止距離] PWM=%d 閾値=%dcm 電池=%.2fV  首を正面・水平に固定（servo1=%d° servo2=%d°）\n",
                driveRunPwm, driveRunThresholdCm, driveBatteryIdle, servo1Angle, servo2Angle);
  Serial.printf("  %dms後に前進を始めます。どのキーでも即停止／%.0fcm未満でも即停止\n",
                DRIVE_ARM_MS, SAFETY_STOP_CM);
}

// 停止命令。モーターを止め、停止後の測距の待ちに入る
static void Drive_Stop(DriveReason reason, float cmdCm) {
  Motor_Stop();
  driveStopMs = millis();
  driveRunMs = driveStopMs - driveRunStartMs;
  driveReason = reason;
  driveCmdCm = cmdCm;
  driveWindowCount = 0;
  drivePhase = DR_STOPPING;
  Serial.printf("停止命令: %s  距離 %.1fcm  走行 %lums\n", DriveReasonName(reason), cmdCm, driveRunMs);
}

static void Drive_Report(bool judged, float finalCm, unsigned long restMs) {
  bool rampDone = driveRunMs >= DRIVE_RAMP_MS;
  float slide = driveCmdCm - finalCm;

  Serial.printf("[停止距離結果] PWM=%d 閾値=%dcm 停止理由:%s\n",
                driveRunPwm, driveRunThresholdCm, DriveReasonName(driveReason));
  Serial.printf("  停止命令時の距離 %.1fcm → 完全停止後の距離 %.1fcm  滑走距離 %.1fcm\n",
                driveCmdCm, finalCm, slide);
  if (judged) {
    Serial.printf("  停止命令から完全停止まで 約%lums\n", restMs);
  } else {
    Serial.println("  完全停止を判定できなかったため、最後の測距値を使っています（参考値）");
  }
  Serial.printf("  走行 %lums（加速%s） 電池 走行前%.2fV／走行中", driveRunMs, rampDone ? "完了後" : "中に停止");
  if (isnan(driveBatteryLoad)) {
    Serial.println("-");
  } else {
    Serial.printf("%.2fV\n", driveBatteryLoad);
  }

  if (driveReason != REASON_THRESHOLD) {
    Serial.println("  ※閾値による停止ではないため、measurements.md 用の行は出しません");
    return;
  }
  if (slide < 0) {
    Serial.println("  ※滑走距離が負です（測距のばらつきか、停止後に動いた可能性）。繰り返して確認してください");
  }

  Serial.println("--- measurements.md 用（この行をそのまま貼れます）---");
  Serial.printf("| 巡航速度での停止距離（PWM%d・閾値%dcm） | 滑走%.1fcm（命令時%.1f→停止後%.1fcm",
                driveRunPwm, driveRunThresholdCm, slide, driveCmdCm, finalCm);
  if (judged) Serial.printf("、停止まで約%lums", restMs);
  Serial.printf("） | 電池%.2fV（走行前）/", driveBatteryIdle);
  if (isnan(driveBatteryLoad)) {
    Serial.print("-");
  } else {
    Serial.printf("%.2fV（走行中）", driveBatteryLoad);
  }
  Serial.printf("、PCA9685 %dHz、走行%lums・加速%s、床材：（記入） | |\n",
                PCA9685_FREQUENCY_HZ, driveRunMs, rampDone ? "完了後" : "中に停止");
}

// 走行中・準備中にキーが押されたときの処理
static void Drive_OnKey(char c) {
  bool isEol = (c == '\r' || c == '\n');
  if (drivePhase == DR_ARM) {
    // ターミナルが改行を自動付加する場合、開始コマンドの直後に届く改行は無視する
    if (isEol) return;
    Drive_Abort("開始前にキー入力があったため中止しました");
    FlushInput();
  } else if (drivePhase == DR_RUN) {
    Drive_Stop(REASON_KEY, driveLastCm);   // 改行を含め、どのキーでも即停止
    FlushInput();
  }
  // 停止後の待ち中は、モーターは止まっているので入力は読み捨てる
}

static void Drive_Update(unsigned long now, bool hasEv, const SonarSample &ev) {
  if (hasEv && ev.valid) {
    driveLastCm = ev.cm;
    driveLastValidMs = now;
  }

  switch (drivePhase) {
    case DR_ARM:
      if (now - phaseMs < DRIVE_ARM_MS) return;
      if (driveLastValidMs == 0 || now - driveLastValidMs > DRIVE_STALE_MS) {
        Drive_Abort("測距できません（対象が近すぎる・遠すぎる・センサーを確認）");
      } else if (driveLastCm < SAFETY_STOP_CM) {
        Drive_Abort("すでに10cm未満です");
      } else if (driveLastCm <= driveRunThresholdCm) {
        Drive_Abort("すでに閾値以下です。もっと離して置いてください");
      } else {
        Serial.printf("走行開始 距離 %.1fcm\n", driveLastCm);
        driveRunStartMs = now;
        driveLastMotorMs = now;
        drivePhase = DR_RUN;
      }
      return;

    case DR_RUN: {
      // 安全確認を最優先にする（非常停止 → 閾値 → 測距途切れ → 時間切れ）
      if (hasEv && ev.valid) {
        if (ev.cm < SAFETY_STOP_CM) {
          Drive_Stop(REASON_EMERGENCY, ev.cm);
        } else if (ev.cm <= driveRunThresholdCm) {
          Drive_Stop(REASON_THRESHOLD, ev.cm);
        }
      } else if (now - driveLastValidMs > DRIVE_STALE_MS) {
        Drive_Stop(REASON_NO_ECHO, driveLastCm);
      } else if (now - driveRunStartMs >= DRIVE_MAX_MS) {
        Drive_Stop(REASON_TIMEOUT, driveLastCm);
      }

      if (hasEv) {
        if (ev.valid) {
          Serial.printf("  走行 t=%lums 距離=%.1fcm\n", now - driveRunStartMs, ev.cm);
        } else {
          Serial.printf("  走行 t=%lums 距離=反応なし\n", now - driveRunStartMs);
        }
      }
      if (drivePhase != DR_RUN) return;   // 停止命令を出した

      // 加速：0 から指定PWMまで直線的に上げる
      if (now - driveLastMotorMs >= DRIVE_MOTOR_INTERVAL_MS) {
        driveLastMotorMs = now;
        unsigned long elapsed = now - driveRunStartMs;
        int pwm = (elapsed >= DRIVE_RAMP_MS) ? driveRunPwm : (int)((long)driveRunPwm * elapsed / DRIVE_RAMP_MS);
        Motor_Forward(pwm);
      }
      if (isnan(driveBatteryLoad) && now - driveRunStartMs >= DRIVE_BATTERY_LOAD_MS) {
        driveBatteryLoad = Get_Battery_Voltage();
      }
      return;
    }

    case DR_STOPPING: {
      if (hasEv && ev.trigMs >= driveStopMs) {
        if (ev.valid) {
          if (driveWindowCount == DRIVE_STOP_WINDOW) {   // 古いものから捨てる
            for (int i = 1; i < DRIVE_STOP_WINDOW; i++) {
              driveWindowCm[i - 1] = driveWindowCm[i];
              driveWindowMs[i - 1] = driveWindowMs[i];
            }
            driveWindowCount--;
          }
          driveWindowCm[driveWindowCount] = ev.cm;
          driveWindowMs[driveWindowCount] = ev.trigMs;
          driveWindowCount++;
          Serial.printf("  停止後 t=%lums 距離=%.1fcm\n", now - driveStopMs, ev.cm);
        } else {
          driveWindowCount = 0;   // 連続した有効値だけで判定する
          Serial.printf("  停止後 t=%lums 距離=反応なし\n", now - driveStopMs);
        }
      }

      bool timedOut = now - driveStopMs >= DRIVE_STOP_MAX_MS;
      bool judged = false;
      if (driveWindowCount == DRIVE_STOP_WINDOW && now - driveStopMs >= DRIVE_STOP_MIN_MS) {
        float lo = driveWindowCm[0], hi = driveWindowCm[0];
        for (int i = 1; i < DRIVE_STOP_WINDOW; i++) {
          lo = min(lo, driveWindowCm[i]);
          hi = max(hi, driveWindowCm[i]);
        }
        judged = (hi - lo <= DRIVE_STOP_TOLERANCE_CM);
      }
      if (!judged && !timedOut) return;

      float finalCm = driveLastCm;
      unsigned long restMs = 0;
      if (driveWindowCount > 0) {
        float tmp[DRIVE_STOP_WINDOW];
        for (int i = 0; i < driveWindowCount; i++) tmp[i] = driveWindowCm[i];
        finalCm = MedianInPlace(tmp, driveWindowCount);
        restMs = driveWindowMs[0] - driveStopMs;
      }
      Job_Finish();
      Drive_Report(judged, finalCm, restMs);
      return;
    }
  }
}

// ------------------------ 最低PWM（動き出す最低のPWMを探す）------------------------ //
// 指定PWMを 0 から一気に出し、2秒だけ動かして止める（本体は 50Hz。加速のなめらかさは付けない）。
// 動いたかどうかの判断：前進・後退は、前方の壁との距離の変化（目安）と目視。回転は目視。
enum MinPwmPhase { MPP_ARM, MPP_RUN, MPP_REST };
enum MinPwmReason { MPR_DONE, MPR_EMERGENCY, MPR_KEY, MPR_NO_ECHO };

static const char *MinPwmModeName(MinPwmMode m) {
  switch (m) {
    case MP_FORWARD:      return "前進";
    case MP_BACKWARD:     return "後退";
    case MP_ROTATE_LEFT:  return "左回転";
    case MP_ROTATE_RIGHT: return "右回転";
    case MP_PIVOT_LEFT:   return "左片側旋回";
    default:              return "右片側旋回";
  }
}

static const char *MinPwmReasonName(MinPwmReason r) {
  switch (r) {
    case MPR_DONE:      return "時間どおり";
    case MPR_EMERGENCY: return "非常停止（10cm未満）";
    case MPR_KEY:       return "キー入力";
    default:            return "測距が途切れた";
  }
}

static MinPwmPhase minPwmPhase = MPP_ARM;
static MinPwmReason minPwmReason = MPR_DONE;
static MinPwmMode minPwmRunMode = MP_FORWARD;       // この走行のモード・PWM
static int minPwmRunPwm = 0;
static unsigned long minPwmStartMs = 0;
static unsigned long minPwmStopMs = 0;
static unsigned long minPwmLastValidMs = 0;         // 最後に有効な測距が得られた時刻（0=まだない）
static float minPwmLastCm = 0.0f;
static float minPwmStartCm = NAN;                   // 走行開始時の前方の距離（測れなければ NAN）
static float minPwmBatteryIdle = NAN;
static int minPwmSampleCount = 0;
static unsigned long minPwmSampleMs[MINPWM_SAMPLES_MAX];   // 走行開始からの経過時間
static float minPwmSampleCm[MINPWM_SAMPLES_MAX];

static void MinPwm_Drive(MinPwmMode mode, int pwm) {
  switch (mode) {
    case MP_FORWARD:      Motor_Move(pwm, pwm);   break;
    case MP_BACKWARD:     Motor_Move(-pwm, -pwm); break;
    case MP_ROTATE_LEFT:  Motor_Move(-pwm, pwm);  break;   // 左輪が後退・右輪が前進
    case MP_ROTATE_RIGHT: Motor_Move(pwm, -pwm);  break;
    case MP_PIVOT_LEFT:   Motor_Move(0, pwm);     break;   // 右輪だけ前進（左輪は止める）
    case MP_PIVOT_RIGHT:  Motor_Move(pwm, 0);     break;   // 左輪だけ前進（右輪は止める）
  }
}

static void MinPwm_Abort(const char *message) {
  Motor_Stop();
  Job_Finish();
  Serial.printf("[最低PWM] 走行しません: %s\n", message);
}

static void MinPwm_Begin(void) {
  minPwmRunMode = minPwmMode;
  minPwmRunPwm = minPwm;

  Neck_Front();  // 走行中は首を正面・水平に固定する
  minPwmBatteryIdle = Get_Battery_Voltage();
  minPwmLastValidMs = 0;
  minPwmLastCm = 0.0f;
  minPwmStartCm = NAN;
  minPwmSampleCount = 0;
  job = JOB_MINPWM;
  minPwmPhase = MPP_ARM;
  phaseMs = millis();
  Sonar_StartSeries(DRIVE_RANGE_INTERVAL_MS, phaseMs);

  Serial.printf("[最低PWM] %s PWM=%d %dms 電池=%.2fV PCA9685 %dHz  首を正面・水平に固定（servo1=%d° servo2=%d°）\n",
                MinPwmModeName(minPwmRunMode), minPwmRunPwm, MINPWM_RUN_MS, minPwmBatteryIdle,
                PCA9685_FREQUENCY_HZ, servo1Angle, servo2Angle);
  Serial.printf("  %dms後に動きます。どのキーでも即停止", MINPWM_ARM_MS);
  if (minPwmRunMode == MP_FORWARD) Serial.printf("／%.0fcm未満でも即停止", SAFETY_STOP_CM);
  Serial.println();
}

// 停止。停止後も少し測距を続けて、最終の距離を記録する
static void MinPwm_Stop(MinPwmReason reason) {
  Motor_Stop();
  minPwmStopMs = millis();
  minPwmReason = reason;
  minPwmPhase = MPP_REST;
}

// 走行中（停止命令まで）の測距から、進んだ向きの速度（cm/s）を求める。求められなければ NAN
// 前進は前方の壁に近づく速さ、後退は遠ざかる速さ。動き始めの立ち上がりを除くため MINPWM_SPEED_FROM_MS 以降を使う
static float MinPwm_Speed(void) {
  if (!MinPwmModeIsLinear(minPwmRunMode)) return NAN;
  unsigned long runMs = minPwmStopMs - minPwmStartMs;
  int first = -1, last = -1;
  for (int i = 0; i < minPwmSampleCount; i++) {
    if (minPwmSampleMs[i] < MINPWM_SPEED_FROM_MS || minPwmSampleMs[i] > runMs) continue;
    if (first < 0) first = i;
    last = i;
  }
  if (first < 0 || minPwmSampleMs[last] - minPwmSampleMs[first] < MINPWM_SPEED_SPAN_MS) return NAN;
  float approach = (minPwmSampleCm[first] - minPwmSampleCm[last]) * 1000.0f
                   / (float)(minPwmSampleMs[last] - minPwmSampleMs[first]);   // 近づく向きが正
  return (minPwmRunMode == MP_FORWARD) ? approach : -approach;
}

static void MinPwm_Report(void) {
  bool linear = MinPwmModeIsLinear(minPwmRunMode);
  bool endValid = (minPwmLastValidMs != 0 && minPwmLastValidMs >= minPwmStopMs);
  float endCm = endValid ? minPwmLastCm : NAN;

  Serial.printf("[最低PWM結果] %s PWM=%d %dms 電池=%.2fV（走行前） PCA9685 %dHz 停止理由:%s\n",
                MinPwmModeName(minPwmRunMode), minPwmRunPwm, MINPWM_RUN_MS, minPwmBatteryIdle,
                PCA9685_FREQUENCY_HZ, MinPwmReasonName(minPwmReason));

  if (isnan(minPwmStartCm) || isnan(endCm)) {
    Serial.println("  前方の距離: 開始か終了のどちらかが測れなかったため変化は不明（目で見て判断してください）");
  } else {
    float change = endCm - minPwmStartCm;
    Serial.printf("  前方の距離 %.1fcm → %.1fcm（変化 %+.1fcm）", minPwmStartCm, endCm, change);
    if (linear) {
      float moved = (minPwmRunMode == MP_FORWARD) ? -change : change;   // 進んだ向きへの移動量
      if (moved >= MINPWM_MOVED_CM) {
        Serial.printf("  → %sした（目安）\n", MinPwmModeName(minPwmRunMode));
      } else {
        Serial.printf("  → 動かなかった（目安。変化が%.1fcm未満）\n", MINPWM_MOVED_CM);
      }
    } else {
      Serial.println("  （回転は参考。目で見て判断してください）");
    }
  }

  float speed = MinPwm_Speed();
  if (!isnan(speed)) {
    Serial.printf("  速度 %.1fcm/s（動かし始め%dms以降の測距から）\n", speed, MINPWM_SPEED_FROM_MS);
  } else if (linear) {
    Serial.println("  速度: 求められませんでした（測距が足りない）");
  }
  Serial.printf("  次: u/d でPWM ±%d（いま %d）、g で実行、1〜6 でモード変更", MinPwmStep(minPwmMode), minPwm);
  if (!linear) Serial.print("、回り始めたら h で回り続ける最低PWMを探す");
  Serial.println();
}

// 走行中・準備中にキーが押されたときの処理
static void MinPwm_OnKey(char c) {
  bool isEol = (c == '\r' || c == '\n');
  if (minPwmPhase == MPP_ARM) {
    // ターミナルが改行を自動付加する場合、開始コマンドの直後に届く改行は無視する
    if (isEol) return;
    MinPwm_Abort("開始前にキー入力があったため中止しました");
    FlushInput();
  } else if (minPwmPhase == MPP_RUN) {
    MinPwm_Stop(MPR_KEY);   // 改行を含め、どのキーでも即停止
    FlushInput();
  }
  // 停止後の待ち中は、モーターは止まっているので入力は読み捨てる
}

static void MinPwm_Update(unsigned long now, bool hasEv, const SonarSample &ev) {
  if (hasEv && ev.valid) {
    minPwmLastCm = ev.cm;
    minPwmLastValidMs = now;
    if (minPwmPhase != MPP_ARM && ev.trigMs >= minPwmStartMs && minPwmSampleCount < MINPWM_SAMPLES_MAX) {
      minPwmSampleMs[minPwmSampleCount] = ev.trigMs - minPwmStartMs;
      minPwmSampleCm[minPwmSampleCount] = ev.cm;
      minPwmSampleCount++;
    }
  }

  switch (minPwmPhase) {
    case MPP_ARM: {
      if (now - phaseMs < MINPWM_ARM_MS) return;
      bool haveCm = (minPwmLastValidMs != 0 && now - minPwmLastValidMs <= DRIVE_STALE_MS);
      if (minPwmRunMode == MP_FORWARD) {
        // 前進は前方の壁で止める安全策が要るので、測距できることが条件
        if (!haveCm) {
          MinPwm_Abort("測距できません（前方に壁を置く。近すぎる・遠すぎる・センサーを確認）");
          return;
        }
        if (minPwmLastCm < SAFETY_STOP_CM) {
          MinPwm_Abort("すでに10cm未満です。もっと離して置いてください");
          return;
        }
      }
      minPwmStartCm = haveCm ? minPwmLastCm : NAN;
      minPwmStartMs = now;
      MinPwm_Drive(minPwmRunMode, minPwmRunPwm);
      minPwmPhase = MPP_RUN;
      if (haveCm) {
        Serial.printf("動き始め 前方 %.1fcm\n", minPwmStartCm);
      } else {
        Serial.println("動き始め 前方の距離は測れていません");
      }
      return;
    }

    case MPP_RUN:
      // 安全確認を最優先にする。前進のみ、壁に近づくので 非常停止 → 測距途切れ を見る
      if (minPwmRunMode == MP_FORWARD) {
        if (hasEv && ev.valid && ev.cm < SAFETY_STOP_CM) {
          MinPwm_Stop(MPR_EMERGENCY);
          return;
        }
        if (now - minPwmLastValidMs > DRIVE_STALE_MS) {
          MinPwm_Stop(MPR_NO_ECHO);
          return;
        }
      }
      if (now - minPwmStartMs >= MINPWM_RUN_MS) MinPwm_Stop(MPR_DONE);
      return;

    case MPP_REST:
      if (now - minPwmStopMs < MINPWM_REST_MS) return;
      Job_Finish();
      MinPwm_Report();
      return;
  }
}

// ------------------------ 回り続ける最低PWM（動かし続けたままPWMを下げる）------------------------ //
// 回転・片側旋回を、いま選んでいるPWMで回し始め、そのまま u/d で PWM を50ずつ変える。
// 止まる直前（まだ回り続けていた最後）のPWMを目で見て探す。一度止まったら回り出しの値に戻るので、
// 止まったら s かスペースで止めて、g の測定（回り始める最低PWM）に戻る。
// 安全策：回転・片側旋回は壁に近づかないので測距は使わない。開始前のキー入力で中止／
// d・u 以外のキー（s・スペースなど）で即停止（改行だけは無視。ターミナルが自動付加する場合に備える）／
// HOLD_IDLE_MS キー入力がなければ自動停止。
enum HoldPhase { HP_ARM, HP_RUN };

static HoldPhase holdPhase = HP_ARM;
static MinPwmMode holdMode = MP_ROTATE_LEFT;
static int holdPwm = 0;
static int holdStartPwm = 0;
static int holdLowestPwm = 0;               // 回している間に下げた最低のPWM
static unsigned long holdStartMs = 0;
static unsigned long holdLastKeyMs = 0;
static float holdBatteryIdle = NAN;

static void Hold_Begin(void) {
  if (MinPwmModeIsLinear(minPwmMode)) {
    Serial.println("[回り続ける最低PWM] 回転・片側旋回（3〜6）を選んでから h を押してください");
    return;
  }
  holdMode = minPwmMode;
  holdPwm = minPwm;
  holdStartPwm = minPwm;
  holdLowestPwm = minPwm;

  Neck_Front();  // 首を正面・水平に固定する
  holdBatteryIdle = Get_Battery_Voltage();
  job = JOB_HOLD;
  holdPhase = HP_ARM;
  phaseMs = millis();
  Serial.printf("[回り続ける最低PWM] %s PWM=%d から回し続けます 電池=%.2fV PCA9685 %dHz  首を正面・水平に固定\n",
                MinPwmModeName(holdMode), holdPwm, holdBatteryIdle, PCA9685_FREQUENCY_HZ);
  Serial.printf("  %dms後に動きます。回っている間は d:PWM-%d  u:PWM+%d  s かスペース:停止（開始前はどのキーでも中止）\n",
                MINPWM_ARM_MS, HOLD_STEP, HOLD_STEP);
}

static void Hold_Stop(const char *reason) {
  Motor_Stop();
  Serial.printf("[回り続ける最低PWM結果] %s 開始PWM=%d 最後のPWM=%d 下げた最低PWM=%d 経過%lums 停止理由:%s\n",
                MinPwmModeName(holdMode), holdStartPwm, holdPwm, holdLowestPwm, millis() - holdStartMs, reason);
  Serial.println("  ※「回り続けた最低PWM」は、止まった1つ前のPWM（上の t=…ms の行）から読み取ってください");
  Job_Finish();
}

// キーで PWM を変えて、そのまま出力する
static void Hold_SetPwm(int pwm, unsigned long now) {
  holdPwm = constrain(pwm, HOLD_PWM_MIN, DRIVE_PWM_LIMIT);
  if (holdPwm < holdLowestPwm) holdLowestPwm = holdPwm;
  MinPwm_Drive(holdMode, holdPwm);
  Serial.printf("  t=%lums PWM=%d%s\n", now - holdStartMs, holdPwm,
                holdPwm == HOLD_PWM_MIN ? "（下限）" : (holdPwm == DRIVE_PWM_LIMIT ? "（上限）" : ""));
}

static void Hold_OnKey(char c) {
  bool isEol = (c == '\r' || c == '\n');
  if (isEol) return;
  if (holdPhase == HP_ARM) {
    Motor_Stop();
    Job_Finish();
    FlushInput();
    Serial.println("[回り続ける最低PWM] 走行しません: 開始前にキー入力があったため中止しました");
    return;
  }
  unsigned long now = millis();
  holdLastKeyMs = now;
  if (c == 'd') {
    Hold_SetPwm(holdPwm - HOLD_STEP, now);
  } else if (c == 'u') {
    Hold_SetPwm(holdPwm + HOLD_STEP, now);
  } else {
    Hold_Stop("キー入力");
    FlushInput();
  }
}

static void Hold_Update(unsigned long now) {
  if (holdPhase == HP_ARM) {
    if (now - phaseMs < MINPWM_ARM_MS) return;
    holdStartMs = now;
    holdLastKeyMs = now;
    holdPhase = HP_RUN;
    MinPwm_Drive(holdMode, holdPwm);
    Serial.printf("回り始め PWM=%d\n", holdPwm);
    return;
  }
  if (now - holdLastKeyMs >= HOLD_IDLE_MS) Hold_Stop("キー入力がなかったため自動停止");
}

// ======================== 表示 ======================== //
static void PrintHelp(void) {
  Serial.println("=== Nova M2案 測定ツール ===");
  Serial.println("[水平角度の探索]");
  Serial.println("  ]:servo2 +1°  [:servo2 -1°  （角度を表示し、少し待って距離の中央値も表示）");
  Serial.println("  l:現在のservo2角度を「水平角」に設定  c:首を正面・水平に戻す");
  Serial.println("[首の安定時間]  t:開始（正面に戻した直後から20ms間隔で1秒×5回）  a:振り幅を切り替え(20/40/55°)");
  Serial.println("[測距のばらつき]  v:20回測定  o:対象物を切り替え(手/服/壁0°/壁30°/壁45°)  n:想定距離を切り替え(10/30/60cm)");
  Serial.println("[停止距離]  f:前進して閾値で停止  +/-:PWM ±50  >/<:停止閾値 ±5cm");
  Serial.println("[最低PWM]  1:前進 2:後退 3:左回転 4:右回転 5:左片側旋回 6:右片側旋回（選ぶと範囲の下限に戻る）");
  Serial.printf("            u/d:PWM ±刻み（1,2は%d〜%d・%d刻み／3〜6は%d〜%d・%d刻み）  g:%dms動かして止める\n",
                MINPWM_LINEAR_LO, MINPWM_LINEAR_HI, MINPWM_LINEAR_STEP, MINPWM_TURN_LO, MINPWM_TURN_HI, MINPWM_TURN_STEP,
                MINPWM_RUN_MS);
  Serial.printf("[回り続ける最低PWM]  h:いまのモード（3〜6）とPWMで回し続ける  u/d:PWM ±%d（回したまま）  s かスペース:停止（%d秒無入力でも停止）\n",
                HOLD_STEP, HOLD_IDLE_MS / 1000);
  Serial.println("[その他]  p:現在の設定を表示  ?:この一覧を表示");
  Serial.println("測定中にキーを押すと中断します。結果は measurements.md 用の表の行でも出力します");
  Serial.printf("走行の安全策: 超音波%.0fcm未満で即停止／走行中はどのキーでも即停止／首は正面・水平に固定／\n", SAFETY_STOP_CM);
  Serial.printf("            測距が%dms途切れたら停止／%dms走っても止まらなければ停止\n", DRIVE_STALE_MS, DRIVE_MAX_MS);
  Serial.println("※ターミナルが改行を自動付加する設定でも、開始直後の改行は無視します");
}

static void PrintStatus(void) {
  Serial.printf("首: servo1=%d°(正面%d°) servo2=%d°(水平角%d°、範囲%d〜%d°)\n",
                servo1Angle, SERVO1_FRONT_DEG, servo2Angle, tiltLevelDeg, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
  Serial.printf("測距条件: 対象=%s 想定距離=%dcm  首の振り幅=±%d°\n",
                TARGET_NAMES[targetIndex], TARGET_DISTANCES_CM[distanceIndex], SWING_AMPLITUDES_DEG[amplitudeIndex]);
  Serial.printf("停止距離: PWM=%d(上限%d) 閾値=%dcm  PCA9685=%dHz 電池=%.2fV\n",
                drivePwm, DRIVE_PWM_LIMIT, stopThresholdCm, PCA9685_FREQUENCY_HZ, Get_Battery_Voltage());
  Serial.printf("最低PWM: モード=%s PWM=%d（範囲%d〜%d、%d刻み）\n",
                MinPwmModeName(minPwmMode), minPwm, MinPwmLo(minPwmMode), MinPwmHi(minPwmMode), MinPwmStep(minPwmMode));
}

// servo2 の角度を「水平角」として採用し、measurements.md 用の行を出す
static void Level_Set(void) {
  tiltLevelDeg = servo2Angle;
  Serial.printf("水平角を servo2=%d° に設定しました（以降の測定・走行でこの角度を使います）\n", tiltLevelDeg);
  Serial.println("--- measurements.md 用（この行をそのまま貼れます）---");
  Serial.printf("| servo2 水平（超音波が水平になる角度） | %d | M2案で測定。servo2 の基準角と一致するか確認 | |\n", tiltLevelDeg);
  Serial.println("※本体の src/config.h の SERVO2_LEVEL_DEG への反映は別途行ってください");
}

// ======================== コマンド処理 ======================== //
static void HandleCommand(char c) {
  switch (c) {
    case '?':
      PrintHelp();
      PrintStatus();
      break;
    case 'p':
      PrintStatus();
      break;
    case ']':
      Tilt_Step(+1);
      break;
    case '[':
      Tilt_Step(-1);
      break;
    case 'l':
      Level_Set();
      break;
    case 'c':
      Neck_Front();
      Serial.printf("首を正面・水平に戻しました（servo1=%d° servo2=%d°）\n", servo1Angle, servo2Angle);
      break;
    case 't':
      Settle_Begin();
      break;
    case 'a':
      amplitudeIndex = (amplitudeIndex + 1) % SWING_AMPLITUDE_COUNT;
      Serial.printf("首の振り幅を±%d°にしました\n", SWING_AMPLITUDES_DEG[amplitudeIndex]);
      break;
    case 'v':
      Variance_Begin();
      break;
    case 'o':
      targetIndex = (targetIndex + 1) % TARGET_COUNT;
      Serial.printf("対象物を「%s」にしました（想定距離 %dcm）\n", TARGET_NAMES[targetIndex], TARGET_DISTANCES_CM[distanceIndex]);
      break;
    case 'n':
      distanceIndex = (distanceIndex + 1) % TARGET_DISTANCE_COUNT;
      Serial.printf("想定距離を%dcmにしました（対象物 %s）\n", TARGET_DISTANCES_CM[distanceIndex], TARGET_NAMES[targetIndex]);
      break;
    case 'f':
      Drive_Begin();
      break;
    case '+':
      drivePwm = min(drivePwm + DRIVE_PWM_STEP, DRIVE_PWM_LIMIT);
      Serial.printf("PWM=%d\n", drivePwm);
      break;
    case '-':
      drivePwm = max(drivePwm - DRIVE_PWM_STEP, 0);
      Serial.printf("PWM=%d\n", drivePwm);
      break;
    case '>':
      stopThresholdCm = min(stopThresholdCm + STOP_THRESHOLD_STEP_CM, STOP_THRESHOLD_MAX_CM);
      Serial.printf("停止閾値=%dcm\n", stopThresholdCm);
      break;
    case '<':
      stopThresholdCm = max(stopThresholdCm - STOP_THRESHOLD_STEP_CM, STOP_THRESHOLD_MIN_CM);
      Serial.printf("停止閾値=%dcm\n", stopThresholdCm);
      break;
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
      minPwmMode = (MinPwmMode)(c - '1');
      minPwm = MinPwmLo(minPwmMode);
      Serial.printf("最低PWM: モードを「%s」にしました（PWM=%d に戻しました）\n", MinPwmModeName(minPwmMode), minPwm);
      break;
    case 'u':
      minPwm = min(minPwm + MinPwmStep(minPwmMode), MinPwmHi(minPwmMode));
      Serial.printf("最低PWM: %s PWM=%d%s\n", MinPwmModeName(minPwmMode), minPwm, minPwm == MinPwmHi(minPwmMode) ? "（上限）" : "");
      break;
    case 'd':
      minPwm = max(minPwm - MinPwmStep(minPwmMode), MinPwmLo(minPwmMode));
      Serial.printf("最低PWM: %s PWM=%d%s\n", MinPwmModeName(minPwmMode), minPwm, minPwm == MinPwmLo(minPwmMode) ? "（下限）" : "");
      break;
    case 'g':
      MinPwm_Begin();
      break;
    case 'h':
      Hold_Begin();
      break;
    case 's':
      Serial.println("停止しています（走行中はどのキーでも停止します）");
      break;
    case '\r':
    case '\n':
      break;
    default:
      Serial.printf("不明なコマンド: '%c'  （? でコマンド一覧）\n", c);
      break;
  }
}

// 1文字受信したときの入口。走行中はここで最優先に停止する
static void OnKey(char c) {
  bool isEol = (c == '\r' || c == '\n');

  switch (job) {
    case JOB_DRIVE:
      Drive_OnKey(c);
      return;

    case JOB_MINPWM:
      MinPwm_OnKey(c);
      return;

    case JOB_HOLD:
      Hold_OnKey(c);
      return;

    case JOB_SETTLE:
      if (isEol) return;
      Job_Finish();
      Neck_Front();   // 首を振っている途中で止めたときは正面に戻す
      Serial.println("キー入力で中断しました");
      return;

    case JOB_COLLECT:
      if (isEol) return;
      if (collectPurpose == COLLECT_TILT) {
        // 角度の探索中は、続けて角度を動かせる。それ以外のキーは探索を終えてコマンドとして処理する
        Job_Finish();
        HandleCommand(c);
      } else {
        Job_Finish();
        Serial.println("キー入力で中断しました");
      }
      return;

    case JOB_IDLE:
      HandleCommand(c);
      return;
  }
}

void setup() {
  Serial.begin(115200);
  Pca9685_Setup();
  Motor_Stop();
  Battery_Setup();
  Sonar_Setup();
  Neck_Front();
  PrintHelp();
  PrintStatus();
}

void loop() {
  // 入力を最優先で処理する（走行中の即停止のため、測距より先に読む）
  while (Serial.available() > 0) {
    OnKey((char)Serial.read());
  }

  unsigned long now = millis();
  SonarSample ev = {false, 0.0f, 0};
  bool hasEv = Sonar_Tick(now, &ev);

  switch (job) {
    case JOB_SETTLE:
      Settle_Update(now, hasEv, ev);
      break;
    case JOB_COLLECT:
      Collect_Update(now, hasEv, ev);
      break;
    case JOB_DRIVE:
      Drive_Update(now, hasEv, ev);
      break;
    case JOB_MINPWM:
      MinPwm_Update(now, hasEv, ev);
      break;
    case JOB_HOLD:
      Hold_Update(now);
      break;
    case JOB_IDLE:
      break;
  }
}
