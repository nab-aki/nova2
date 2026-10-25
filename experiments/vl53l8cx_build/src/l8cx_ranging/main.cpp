// VL53L8CX：測距の時間・失敗の回数・区画の表と、首を振りながらの測距を測る（スプリント3.6-1。単体試験）
// ライブラリは Nova の lib/（STM32duino_VL53L8CX は NOVA_PATCH.md の改変版、PCA9685 は失敗を数える改変版）
//
// 起動時：is_alive → init → 測距の頻度を設定 → 解像度 8×8 → start_ranging。どれかが失敗したら、その旨を出して測距しない
// 測距：check_data_ready が真のときだけ get_ranging_data を呼び、かかった時間（µs）と失敗を数える
// キー（シリアルに1文字送る。a・v は数字を続けて Enter）：
//   g … 距離（mm）と区画の状態（target_status）を1回だけ表で出す
//   r … 4×4 と 8×8 を切り替える（stop_ranging → set_resolution → 頻度の設定 → start_ranging）
//   s … ここまでの集計を出して、数え直す（試験中は数え直さない）
//   n … 首を本体のキー u と同じ動き（正面 → 左＋上 → 右 → 正面）で 50 往復。終わったら集計を出す
//   a … 左右だけを 50 往復（正面 → 左 → 右 → 正面）。上下は数字で指定（正面＝水平からの角度。上が +）。
//       例 a0（水平のまま）・a40。可動範囲（servo2 90〜140°＝水平 98° から -8〜+42°）の外なら実行しない
//   v … 上下だけを 50 往復（水平 ⇔ 水平＋数字。数字なしは +15°）。左右は正面のまま
//   h … サーボの PWM を止めて測り続ける（手で首を動かす・線を揺らす）。Enter で終わり、首を正面へ戻す。失敗した時刻をその場で出す
//   c … Wire1 の速さを 400kHz と 100kHz で切り替える（今の値を出す）
//   x … 首の試験を中断して集計を出す
//   b … 電池の電圧だけを出す（起動時・キー s・首の試験の開始時にも出す）
//   n・a・v は電池が 6.0V 未満なら受け付けない（サーボの電源は電池から。本体の hal_battery と同じ読み方）
// Wire のエラーの行（[E][Wire.cpp:...] ...）は画面に出さず、種類ごとに数えて集計に出す（ets_printf の出口を横取りする）
// 失敗したら、そのときの首の状態（推定の角度・段階・時刻）を記録する
// 測距が止まった（フレームが TOF_STALL_MS 来ない）ら、is_alive・解像度の読み出しで確かめ、start_ranging をやり直す
//   （解像度が読めない・違う＝リセットの疑い → init からやり直す。init は約2.7秒止まる）
// モーターは動かさない：PCA9685 のモーターのチャンネルには何も書かない
//   （起動時の setupSingleDevice のソフトウェアリセットで、全チャンネルが止まった状態になる。本体と同じ）
// delay() は使わない（ライブラリの中の待ちは除く）
#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>
#include <vl53l8cx.h>
#include <string.h>
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "esp32/rom/ets_sys.h"

// ---- ToF（第2バス）----
#define PIN_TOF_SDA         26        // ESP32 基板のヘッダー P3-10
#define PIN_TOF_SCL         27        // ESP32 基板のヘッダー P3-11
#define TOF_I2C_HZ          400000    // 400kHz（線 13cm。2026-10-24 の起動試験で成功）
#define TOF_I2C_HZ_SLOW     100000    // キー c で切り替える遅いほう
#define TOF_FREQ_HZ         10        // 測距の頻度（4×4・8×8 の両方で使える値。8×8 の上限は 15Hz）
#define TOF_POLL_MS         5         // check_data_ready を呼ぶ間隔
#define TOF_STALL_MS        700       // フレームがこれだけ来なければ「測距が止まった」とみなして確かめる（10Hz で7回分）
#define TOF_RECOVER_RETRY_MS 1000     // 確かめ・やり直しの最短の間隔
// 表の向き：区画の番号 0 がどこに来るかは、取り付けの向きで変わる（実物で確かめる。未確認）
#define TOF_FLIP_ROWS       0         // 1 にすると行の順を逆にする
#define TOF_FLIP_COLS       0         // 1 にすると列の順を逆にする

// ---- 首（本体の src/hal_pins.h・config.h と同じ値）----
#define PIN_I2C_SDA         13
#define PIN_I2C_SCL         14
#define I2C_FREQUENCY_HZ    100000
#define I2C_TIMEOUT_MS      5
#define I2C_ADDR_PCA9685    0x5F
#define PCA9685_FREQUENCY_HZ 50
#define PCA9685_CH_SERVO1   0         // 左右
#define PCA9685_CH_SERVO2   1         // 上下
#define SERVO1_FRONT_DEG    84
#define SERVO1_MIN_DEG      20
#define SERVO1_MAX_DEG      140
#define SERVO1_LEFT_SIGN    (+1)
#define SERVO2_LEVEL_DEG    98
#define SERVO2_MIN_DEG      90
#define SERVO2_MAX_DEG      140       // 上が +（キー u・n の 113° は水平から 15° 上）
#define SCAN_PAN_DEG        50        // 左右（正面 ±50°。WANDER_SCAN_PAN_DEG）
#define SCAN_TILT_DEG       113       // キー u の上下（DEBUG_SCAN_TILT_DEG）
#define TILT_TEST_DEFAULT   15        // キー v で数字がないときの振れ幅（水平から上へ）
#define NECK_SETTLE_BASE_MS 60        // 首を動かしたら待つ時間（本体と同じ式）
#define NECK_SETTLE_PER_DEG_MS 2
#define NECK_DWELL_MS       300       // 各向きで止まっている時間（本体の超音波3回の測距の代わり。試験用）
#define NECK_ROUNDS         50        // 往復の回数
#define KEY_NUMBER_TIMEOUT_MS 3000    // a・v の数字の入力を待つ時間（Enter がなくてもこの時間で実行する）
#define HAND_ENTER_IGNORE_MS 300      // キー h の直後の Enter は無視する（行単位で送るモニタの改行で、すぐ終わらないように）
#define HAND_PRINT_MIN_MS   200       // キー h の失敗の行を出す最短の間隔（間の失敗は次の行にまとめて数える）
#define HAND_STATUS_MS      5000      // キー h の途中経過を出す間隔

// ---- 電池（本体の src/hal/hal_battery.cpp・hal_pins.h と同じピン・換算）----
#define PIN_BATTERY            32
#define BATTERY_DEFAULT_VREF   1100   // 公式サンプルと同じ
#define BATTERY_DIVIDER_RATIO  4.0f   // 分圧比（公式サンプルの batteryCoefficient）
#define BATTERY_SAMPLE_COUNT   5
#define BATTERY_SERVO_MIN_V    6.0f   // これ未満なら、サーボに電池の電源が来ていない（または低い）とみなす（試験用）

static VL53L8CX sensor(&Wire1, -1);
static PCA9685 pca;
static VL53L8CX_ResultsData results;   // 約1.4KB。スタックに置かない

static bool tofStarted = false;        // 起動時の設定がすべて成功した（やり直しはこのときだけ）
static bool tofOk = false;             // いま測距中（やり直しに失敗したら false）
static uint8_t resolution = VL53L8CX_RESOLUTION_8X8;
static uint32_t tofHz = TOF_I2C_HZ;
static uint32_t lastPollMs = 0;
static uint32_t lastFrameMs = 0;
static uint32_t lastRecoverMs = 0;

// ---- Wire のエラーの行を数える ----
// log_e は ets_printf を通って putc1 に1文字ずつ来る。行にためて、種類（「] 」より後ろの文）ごとに数える
#define LOG_TYPE_MAX  10
#define LOG_KEY_LEN   60
struct LogType {
  char key[LOG_KEY_LEN];
  uint32_t count;
};
static LogType logTypes[LOG_TYPE_MAX];
static uint32_t logOverflow = 0;       // 種類の表があふれた分
static volatile uint32_t logLines = 0; // 横取りした行の累計（呼ぶ前後で比べて、その呼び出しでバスのエラーが出たかを見る）
static volatile uint8_t curBus = 0;    // いま通信しているバス（0＝Wire・首、1＝Wire1・ToF）。種類の頭に付ける
static char logBuf[128];
static uint16_t logLen = 0;

// ---- 最初の失敗の記録 ----
// 測距が順調な状態（開始直後・やり直しが成功した直後）から、最初にエラーが出た瞬間を残す。
// エラー＝Wire1 のエラーの行／check_data_ready・get_ranging_data の失敗。エラーなしでフレームが止まったときは、見つけた時点で残す
enum FirstFailSrc { FF_BUS, FF_READY, FF_DATA, FF_STALL };
struct FirstFail {
  uint32_t tMs;          // 集計の開始からの時刻
  int16_t pan, tilt;     // 推定の角度（正面・水平から。手動のときは不明）
  uint8_t phase;
  uint8_t src;
  uint8_t status;
  bool fromRecovery;     // 測った起点が「やり直しの終了」か「集計の開始」か
  uint32_t durMs;        // 起点から最初の失敗まで（FF_STALL は最後のフレームまで）
  uint32_t frames;       // その間のフレーム数
};
static FirstFail ff;
static bool ffValid = false;            // ff に今の期間の記録がある
static bool ffPrinted = false;          // [rec] に出し済み
static volatile bool ffPending = false; // 順調な状態で、最初の失敗を待っている
static uint32_t healthyStartMs = 0;
static uint32_t healthyFrames0 = 0;
static bool healthyFromRecovery = false;
static void CaptureFirstFail(uint32_t nowMs, uint8_t src, uint8_t status);

// 通信の間だけ curBus を切り替える
struct BusScope {
  uint8_t prev;
  explicit BusScope(uint8_t bus) : prev(curBus) { curBus = bus; }
  ~BusScope() { curBus = prev; }
};

static void LogClassify(const char *line) {
  if (curBus == 1 && ffPending) {
    CaptureFirstFail(millis(), FF_BUS, 0);   // 行が出た瞬間（通信の途中）が最初の失敗
  }
  // 例 "[ 12345][E][Wire.cpp:513] requestFrom(): i2cRead returned Error -1" → "requestFrom(): i2cRead returned Error -1"
  // ESP-IDF の行 "E (1234) i2c: ..." → "i2c: ..."
  const char *body = strstr(line, "] ");
  if (body != nullptr) {
    body += 2;
  } else if ((body = strstr(line, ") ")) != nullptr) {
    body += 2;
  } else {
    body = line;
  }
  char key[LOG_KEY_LEN];
  snprintf(key, sizeof(key), "W%u %s", (unsigned)curBus, body);
  for (int i = 0; i < LOG_TYPE_MAX; i++) {
    if (logTypes[i].count == 0) {
      strncpy(logTypes[i].key, key, LOG_KEY_LEN - 1);
      logTypes[i].key[LOG_KEY_LEN - 1] = '\0';
      logTypes[i].count = 1;
      return;
    }
    if (strcmp(logTypes[i].key, key) == 0) {
      logTypes[i].count++;
      return;
    }
  }
  logOverflow++;
}

static void LogPutc(char c) {
  if (c == '\r') {
    return;
  }
  if (c == '\n') {
    logBuf[logLen] = '\0';
    if (logLen > 0) {
      logLines++;
      LogClassify(logBuf);
    }
    logLen = 0;
    return;
  }
  if (logLen < sizeof(logBuf) - 1) {
    logBuf[logLen++] = c;
  }
}

// ESP-IDF の esp_log の行も同じように数える
static int LogVprintf(const char *fmt, va_list args) {
  char buf[128];
  int n = vsnprintf(buf, sizeof(buf), fmt, args);
  for (const char *p = buf; *p; p++) {
    LogPutc(*p);
  }
  return n;
}

static void LogClear(void) {
  for (int i = 0; i < LOG_TYPE_MAX; i++) {
    logTypes[i].count = 0;
    logTypes[i].key[0] = '\0';
  }
  logOverflow = 0;
}

// ---- 首の試験の種類・段階 ----
enum NeckMode { MODE_IDLE, MODE_N, MODE_A, MODE_V, MODE_H };
static NeckMode mode = MODE_IDLE;
static int testTiltOff = 0;            // a：上下（水平から）、v：振れ幅

enum Phase { PH_ONSET, PH_MOVING, PH_AFTER, PH_HAND, PH_COUNT };
static const char *const PHASE_NAMES[PH_COUNT] = {"動き出し", "移動中", "停止後", "手動"};

// ---- 集計 ----
#define PAN_BIN_N     13     // 左右（正面から。左が +）：-60〜+60 を 10° ごと（中央の 0 は -4〜+4）
#define TILT_BIN_N    7      // 上下（水平から。上が +）：-10〜+50 を 10° ごと
#define FAIL_REC_MAX  40     // 1件ずつ残す失敗の数
#define FAIL_REC_PRINT 20    // 集計に出す件数
#define REC_PRINT_MAX 10     // やり直しの行を出す回数（試験ごと）
#define ANGLE_UNKNOWN INT16_MIN

enum FailKind { FK_READY, FK_DATA };

struct FailRec {
  uint32_t tMs;        // 試験の開始からの時刻
  int16_t pan, tilt;   // 推定の角度（正面・水平から。手動のときは不明）
  uint8_t phase;
  uint8_t kind;
  uint8_t status;      // ライブラリの戻り値
  uint8_t busLines;    // その呼び出しの間に出た Wire のエラーの行の数（0＝バスは成功、センサーが返したエラー）
};

struct Stats {
  uint32_t frames;        // get_ranging_data が成功した回数
  uint32_t readyFail;     // check_data_ready の失敗
  uint32_t dataFail;      // get_ranging_data の失敗
  uint32_t noBusFail;     // 失敗のうち、Wire のエラーの行が出なかったもの
  uint32_t minUs, maxUs;
  uint64_t sumUs;
  uint32_t startMs;
  uint32_t pcaTotal0, pcaFail0;
  float batV;             // 開始時の電池の電圧
  uint32_t failPhase[PH_COUNT];
  uint32_t failPan[PAN_BIN_N];
  uint32_t failTilt[TILT_BIN_N];
  uint32_t recStall;      // 測距が止まったのを見つけた回数
  uint32_t recAliveFail;  // そのとき is_alive が失敗した回数
  uint32_t recRestartOk;  // start_ranging のやり直しが成功した回数
  uint32_t recRestartFail;
  uint32_t recReinit;     // init からやり直した回数（リセットの疑い）
  uint32_t recPrinted;
  uint32_t okN;           // 測れていた時間（やり直しの終了 → 最初の失敗）の回数・合計・最小・最大
  uint64_t okSum;
  uint32_t okMin, okMax;
  uint32_t ok0Ms;         // 集計の開始 → 最初の失敗（UINT32_MAX＝まだ失敗なし）
  FailRec rec[FAIL_REC_MAX];
  uint32_t recCount;
};
static Stats st;

// ---- 電池 ----
static esp_adc_cal_characteristics_t adcChars;

static void BatterySetup(void) {
  analogSetWidth(12);
  analogSetPinAttenuation(PIN_BATTERY, ADC_11db);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, BATTERY_DEFAULT_VREF, &adcChars);
}

static float BatteryReadVoltage(void) {
  long sum = 0;
  for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    sum += analogRead(PIN_BATTERY);
  }
  uint32_t mv = esp_adc_cal_raw_to_voltage(sum / BATTERY_SAMPLE_COUNT, &adcChars);
  return (mv / 1000.0f) * BATTERY_DIVIDER_RATIO;
}

// 電圧を1行で出す。6.0V 未満なら注意も出し、false を返す
static bool PrintBattery(const char *label) {
  float v = BatteryReadVoltage();
  Serial.printf("[bat] %s %.2fV\n", label, v);
  if (v < BATTERY_SERVO_MIN_V) {
    Serial.println("[bat] 電池の電源なし（または低い）。サーボは動かない可能性");
    return false;
  }
  return true;
}

// 「測れていた時間」の起点を決める（集計の開始・やり直しの成功）。測距中でなければ待たない
static void HealthyBegin(uint32_t nowMs, bool fromRecovery) {
  healthyStartMs = nowMs;
  healthyFrames0 = st.frames;
  healthyFromRecovery = fromRecovery;
  ffValid = false;
  ffPrinted = false;
  ffPending = tofOk;
}

static void ResetStats(void) {
  memset(&st, 0, sizeof(st));
  st.minUs = UINT32_MAX;
  st.okMin = UINT32_MAX;
  st.ok0Ms = UINT32_MAX;
  st.startMs = millis();
  HealthyBegin(st.startMs, false);
  st.pcaTotal0 = pca.nova_i2c_total;
  st.pcaFail0 = pca.nova_i2c_fail;
  st.batV = BatteryReadVoltage();
  LogClear();
}

static const char *ModeName(NeckMode m) {
  switch (m) {
    case MODE_N: return "n（左右＋上下）";
    case MODE_A: return "a（左右だけ）";
    case MODE_V: return "v（上下だけ）";
    case MODE_H: return "h（PWM 停止・手動）";
    default:     return "なし";
  }
}

static void PrintStats(const char *label) {
  uint32_t avg = st.frames ? (uint32_t)(st.sumUs / st.frames) : 0;
  uint32_t mn = st.frames ? st.minUs : 0;
  Serial.printf("[tof] %s res=%s i2c=%lukHz frames=%lu sec=%.1f us min=%lu avg=%lu max=%lu fail ready=%lu data=%lu pca=%lu/%lu\n",
                label, resolution == VL53L8CX_RESOLUTION_8X8 ? "8x8" : "4x4", (unsigned long)(tofHz / 1000),
                (unsigned long)st.frames, (millis() - st.startMs) / 1000.0f,
                (unsigned long)mn, (unsigned long)avg, (unsigned long)st.maxUs,
                (unsigned long)st.readyFail, (unsigned long)st.dataFail,
                (unsigned long)(pca.nova_i2c_fail - st.pcaFail0), (unsigned long)(pca.nova_i2c_total - st.pcaTotal0));
  if (mode == MODE_A || mode == MODE_V) {
    Serial.printf("[tof]  試験=%s 上下%+d° 電池(開始)=%.2fV\n", ModeName(mode), testTiltOff, st.batV);
  } else {
    Serial.printf("[tof]  試験=%s 電池(開始)=%.2fV\n", ModeName(mode), st.batV);
  }
  // Wire のエラーの種類
  bool anyLog = false;
  for (int i = 0; i < LOG_TYPE_MAX; i++) {
    if (logTypes[i].count > 0) {
      Serial.printf("[tof]  バスのエラー ×%lu  %s\n", (unsigned long)logTypes[i].count, logTypes[i].key);
      anyLog = true;
    }
  }
  if (logOverflow > 0) {
    Serial.printf("[tof]  バスのエラー ×%lu  （種類の表があふれた分）\n", (unsigned long)logOverflow);
  }
  if (!anyLog) {
    Serial.println("[tof]  バスのエラー なし");
  }
  uint32_t fails = st.readyFail + st.dataFail;
  if (fails > 0) {
    Serial.printf("[tof]  失敗の段階 動き出し=%lu 移動中=%lu 停止後=%lu 手動=%lu／バスのエラーなしの失敗=%lu\n",
                  (unsigned long)st.failPhase[PH_ONSET], (unsigned long)st.failPhase[PH_MOVING],
                  (unsigned long)st.failPhase[PH_AFTER], (unsigned long)st.failPhase[PH_HAND], (unsigned long)st.noBusFail);
    char line[160];
    int len = 0;
    for (int b = 0; b < PAN_BIN_N; b++) {
      len += snprintf(line + len, sizeof(line) - len, " %+d:%lu", (b - 6) * 10, (unsigned long)st.failPan[b]);
    }
    Serial.printf("[tof]  失敗の左右（推定。正面から、左が+）%s\n", line);
    len = 0;
    for (int b = 0; b < TILT_BIN_N; b++) {
      len += snprintf(line + len, sizeof(line) - len, " %+d:%lu", (b - 1) * 10, (unsigned long)st.failTilt[b]);
    }
    Serial.printf("[tof]  失敗の上下（推定。水平から、上が+）%s\n", line);
  }
  Serial.printf("[tof]  やり直し 止まった=%lu is_alive失敗=%lu start成功=%lu start失敗=%lu init=%lu\n",
                (unsigned long)st.recStall, (unsigned long)st.recAliveFail, (unsigned long)st.recRestartOk,
                (unsigned long)st.recRestartFail, (unsigned long)st.recReinit);
  if (st.okN > 0) {
    Serial.printf("[tof]  測れていた時間（やり直しの終了 → 最初の失敗） %lu回 min=%lums avg=%lums max=%lums\n", (unsigned long)st.okN,
                  (unsigned long)st.okMin, (unsigned long)(st.okSum / st.okN), (unsigned long)st.okMax);
  }
  if (st.ok0Ms != UINT32_MAX) {
    Serial.printf("[tof]  集計の開始 → 最初の失敗 %lums\n", (unsigned long)st.ok0Ms);
  }
  uint32_t n = min(st.recCount, (uint32_t)FAIL_REC_PRINT);
  for (uint32_t i = 0; i < n; i++) {
    const FailRec &r = st.rec[i];
    if (r.pan == ANGLE_UNKNOWN) {
      Serial.printf("[tof]  #%lu t=%.3fs %s st=%u バス%u 角度不明 %s\n", (unsigned long)(i + 1), r.tMs / 1000.0f,
                    r.kind == FK_READY ? "ready" : "data", r.status, r.busLines, PHASE_NAMES[r.phase]);
    } else {
      Serial.printf("[tof]  #%lu t=%.3fs %s st=%u バス%u 左右%+d 上下%+d %s\n", (unsigned long)(i + 1), r.tMs / 1000.0f,
                    r.kind == FK_READY ? "ready" : "data", r.status, r.busLines, r.pan, r.tilt, PHASE_NAMES[r.phase]);
    }
  }
  if (fails > n) {
    Serial.printf("[tof]  （失敗の記録は最初の %lu 件だけ出した。全部で %lu 件）\n", (unsigned long)n, (unsigned long)fails);
  }
}

// ---- 首 ----
static uint16_t AngleToPulseWidth(int deg) {
  return (uint16_t)map(constrain(deg, 0, 180), 0, 180, 102, 512);   // 本体の hal_servo と同じ
}

static int panDeg = SERVO1_FRONT_DEG;
static int tiltDeg = SERVO2_LEVEL_DEG;
// 最後の指示（段階と推定の角度に使う）
static uint32_t moveStartMs = 0;
static int moveFromPan = SERVO1_FRONT_DEG;
static int moveFromTilt = SERVO2_LEVEL_DEG;
static uint32_t moveSettleMs = 0;

// 首を動かし、動かした角度の大きいほうから待ち時間（ms）を返す
static uint32_t NeckMove(int pan, int tilt) {
  BusScope bus(0);
  pan = constrain(pan, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
  tilt = constrain(tilt, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
  int moved = max(abs(pan - panDeg), abs(tilt - tiltDeg));
  pca.setChannelPulseWidth(PCA9685_CH_SERVO1, AngleToPulseWidth(pan));
  pca.setChannelPulseWidth(PCA9685_CH_SERVO2, AngleToPulseWidth(tilt));
  moveStartMs = millis();
  moveFromPan = panDeg;
  moveFromTilt = tiltDeg;
  moveSettleMs = NECK_SETTLE_BASE_MS + (uint32_t)moved * NECK_SETTLE_PER_DEG_MS;
  if (moved == 0) {
    moveSettleMs = 0;   // 動いていないので、すぐ「停止後」
  }
  panDeg = pan;
  tiltDeg = tilt;
  return NECK_SETTLE_BASE_MS + (uint32_t)moved * NECK_SETTLE_PER_DEG_MS;
}

// いまの段階と推定の角度（正面・水平からの差。左・上が +）
// 指示から NECK_SETTLE_BASE_MS までは「動き出し」（角度は動く前のまま）、待ち時間の終わりまでは「移動中」（直線で補う）、そのあとは「停止後」
static Phase NeckPhase(uint32_t nowMs, int *panRel, int *tiltRel) {
  if (mode == MODE_H) {
    *panRel = ANGLE_UNKNOWN;
    *tiltRel = ANGLE_UNKNOWN;
    return PH_HAND;
  }
  uint32_t el = nowMs - moveStartMs;
  int pan = panDeg;
  int tilt = tiltDeg;
  Phase ph = PH_AFTER;
  if (el < moveSettleMs) {
    if (el < NECK_SETTLE_BASE_MS) {
      ph = PH_ONSET;
      pan = moveFromPan;
      tilt = moveFromTilt;
    } else {
      ph = PH_MOVING;
      int32_t span = (int32_t)(moveSettleMs - NECK_SETTLE_BASE_MS);
      int32_t k = (int32_t)(el - NECK_SETTLE_BASE_MS);
      pan = moveFromPan + (panDeg - moveFromPan) * k / span;
      tilt = moveFromTilt + (tiltDeg - moveFromTilt) * k / span;
    }
  }
  *panRel = (pan - SERVO1_FRONT_DEG) * SERVO1_LEFT_SIGN;
  *tiltRel = tilt - SERVO2_LEVEL_DEG;
  return ph;
}

// 最初の失敗を残す（順調な状態から最初の1回だけ。あとは何もしない）。時刻・推定の角度・段階・測れていた時間とフレーム数
static void CaptureFirstFail(uint32_t nowMs, uint8_t src, uint8_t status) {
  if (!ffPending) {
    return;
  }
  ffPending = false;
  int panRel, tiltRel;
  Phase ph = NeckPhase(nowMs, &panRel, &tiltRel);
  uint32_t endMs = (src == FF_STALL && lastFrameMs >= healthyStartMs) ? lastFrameMs : nowMs;   // 止まった場合は最後のフレームまで
  ff.tMs = nowMs - st.startMs;
  ff.pan = (int16_t)panRel;
  ff.tilt = (int16_t)tiltRel;
  ff.phase = (uint8_t)ph;
  ff.src = src;
  ff.status = status;
  ff.fromRecovery = healthyFromRecovery;
  ff.durMs = endMs - healthyStartMs;
  ff.frames = st.frames - healthyFrames0;
  ffValid = true;
  ffPrinted = false;
  if (healthyFromRecovery) {
    st.okN++;
    st.okSum += ff.durMs;
    if (ff.durMs < st.okMin) st.okMin = ff.durMs;
    if (ff.durMs > st.okMax) st.okMax = ff.durMs;
  } else {
    st.ok0Ms = ff.durMs;
  }
}

// 本体と同じ PCA9685 の設定（ソフトウェアリセット → ALLCALL を切る → 50Hz）
static void PcaSetup(void) {
  BusScope bus(0);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQUENCY_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  pca.setupSingleDevice(Wire, I2C_ADDR_PCA9685);
  Wire.beginTransmission(I2C_ADDR_PCA9685);   // MODE1（0x00）を読み、bit0（ALLCALL）だけ落として書き戻す
  Wire.write(0x00);
  if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)I2C_ADDR_PCA9685, (uint8_t)1) == 1) {
    uint8_t mode1 = Wire.read() & ~0x01;
    Wire.beginTransmission(I2C_ADDR_PCA9685);
    Wire.write(0x00);
    Wire.write(mode1);
    Wire.endTransmission();
  }
  pca.setToFrequency(PCA9685_FREQUENCY_HZ);
  NeckMove(SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG);
}

// ---- 失敗の記録 ----
static uint32_t handLastPrintMs = 0;
static uint32_t handSuppressed = 0;

static void RecordFail(uint32_t nowMs, FailKind kind, uint8_t status, uint32_t busLines) {
  CaptureFirstFail(nowMs, kind == FK_READY ? FF_READY : FF_DATA, status);   // Wire1 のエラーの行が先に出ていれば、そちらが記録済み
  if (kind == FK_READY) {
    st.readyFail++;
  } else {
    st.dataFail++;
  }
  if (busLines == 0) {
    st.noBusFail++;
  }
  int panRel, tiltRel;
  Phase ph = NeckPhase(nowMs, &panRel, &tiltRel);
  st.failPhase[ph]++;
  if (panRel != ANGLE_UNKNOWN) {
    int pb = (panRel >= 0 ? (panRel + 5) / 10 : -((-panRel + 5) / 10)) + 6;
    int tb = (tiltRel >= 0 ? (tiltRel + 5) / 10 : -((-tiltRel + 5) / 10)) + 1;
    st.failPan[constrain(pb, 0, PAN_BIN_N - 1)]++;
    st.failTilt[constrain(tb, 0, TILT_BIN_N - 1)]++;
  }
  if (st.recCount < FAIL_REC_MAX) {
    FailRec &r = st.rec[st.recCount];
    r.tMs = nowMs - st.startMs;
    r.pan = (int16_t)panRel;
    r.tilt = (int16_t)tiltRel;
    r.phase = (uint8_t)ph;
    r.kind = (uint8_t)kind;
    r.status = status;
    r.busLines = (uint8_t)min(busLines, (uint32_t)255);
  }
  st.recCount++;
  // 手で動かす試験では、失敗した時刻をその場で出す（続けて出たものは次の行にまとめる）
  if (mode == MODE_H) {
    if (nowMs - handLastPrintMs >= HAND_PRINT_MIN_MS) {
      Serial.printf("[hand] 失敗 t=%.3fs %s st=%u バス%lu（前の行からの間に +%lu 件）\n", (nowMs - st.startMs) / 1000.0f,
                    kind == FK_READY ? "ready" : "data", status, (unsigned long)busLines, (unsigned long)handSuppressed);
      handLastPrintMs = nowMs;
      handSuppressed = 0;
    } else {
      handSuppressed++;
    }
  }
}

// ---- ToF ----
// 測距の頻度・解像度を設定して始める。失敗したら false
static bool TofConfigureAndStart(uint8_t res, bool verbose) {
  BusScope bus(1);
  uint8_t s = sensor.set_resolution(res);
  if (verbose) Serial.printf("[tof] set_resolution(%s)=%u\n", res == VL53L8CX_RESOLUTION_8X8 ? "8x8" : "4x4", s);
  if (s != 0) return false;
  s = sensor.set_ranging_frequency_hz(TOF_FREQ_HZ);
  if (verbose) Serial.printf("[tof] set_ranging_frequency_hz(%d)=%u\n", TOF_FREQ_HZ, s);
  if (s != 0) return false;
  s = sensor.start_ranging();
  if (verbose) Serial.printf("[tof] start_ranging=%u\n", s);
  if (s != 0) return false;
  resolution = res;
  lastFrameMs = millis();
  return true;
}

static void TofSetup(void) {
  BusScope bus(1);
  Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL, tofHz);
  sensor.begin();
  uint8_t alive = 0;
  uint32_t t0 = millis();
  uint8_t s = sensor.is_alive(&alive);
  Serial.printf("[tof] is_alive=%u alive=%u %lums\n", s, alive, (unsigned long)(millis() - t0));
  if (s != 0 || alive == 0) {
    Serial.println("[tof] センサーなし：測距しない");
    return;
  }
  t0 = millis();
  s = sensor.init();
  Serial.printf("[tof] init=%u %lums\n", s, (unsigned long)(millis() - t0));
  if (s != 0) {
    Serial.println("[tof] init 失敗：測距しない");
    return;
  }
  tofOk = TofConfigureAndStart(VL53L8CX_RESOLUTION_8X8, true);
  tofStarted = tofOk;
  if (!tofOk) {
    Serial.println("[tof] 設定 失敗：測距しない");
  }
}

// 最初の失敗（と、その前に測れていた時間）を出す（改行なし）
static void PrintFirstFail(void) {
  static const char *const SRC_NAMES[] = {"W1エラー", "ready失敗", "data失敗", "エラーなしで停止"};
  if (ff.pan == ANGLE_UNKNOWN) {
    Serial.printf("／最初の失敗 t=%.3fs %s st=%u 角度不明 %s", ff.tMs / 1000.0f, SRC_NAMES[ff.src], ff.status, PHASE_NAMES[ff.phase]);
  } else {
    Serial.printf("／最初の失敗 t=%.3fs %s st=%u 左右%+d 上下%+d %s", ff.tMs / 1000.0f, SRC_NAMES[ff.src], ff.status,
                  ff.pan, ff.tilt, PHASE_NAMES[ff.phase]);
  }
  Serial.printf("／測れていた %lums frames=%lu（%s）", (unsigned long)ff.durMs, (unsigned long)ff.frames,
                ff.fromRecovery ? "前のやり直しの終了から" : "開始から");
}

// 測距が止まったときに確かめて、やり直す
// is_alive → stop_ranging → get_resolution。解像度が読めない・違う＝設定が消えた（リセットの疑い）→ init から。そのあと start_ranging
static void TofRecover(uint32_t nowMs) {
  BusScope bus(1);
  lastRecoverMs = nowMs;
  st.recStall++;
  CaptureFirstFail(nowMs, FF_STALL, 0);   // エラーが出ないまま止まった場合（エラーがあれば、すでに記録済みで何もしない）
  bool print = st.recPrinted < REC_PRINT_MAX;
  if (print) {
    st.recPrinted++;
    Serial.printf("[rec] t=%.3fs 測距が止まった（最後のフレームから %lums）", (nowMs - st.startMs) / 1000.0f,
                  (unsigned long)(nowMs - lastFrameMs));
    if (ffValid && !ffPrinted) {
      PrintFirstFail();
      Serial.printf("／見つけるまで +%lums", (unsigned long)(nowMs - st.startMs - ff.tMs));
    }
    Serial.println();
  }
  ffPrinted = true;   // やり直しに失敗してくり返すときは、最初の失敗を2度出さない
  uint8_t alive = 0;
  uint32_t t0 = millis();
  uint8_t s = sensor.is_alive(&alive);
  uint32_t msAlive = millis() - t0;
  if (s != 0 || alive == 0) {
    st.recAliveFail++;
    tofOk = false;
    if (print) Serial.printf("[rec]  is_alive=%u alive=%u(%lums)：応答なし。%dms 後にまた確かめる\n", s, alive, (unsigned long)msAlive, TOF_RECOVER_RETRY_MS);
    return;
  }
  t0 = millis();
  uint8_t sStop = sensor.stop_ranging();
  uint32_t msStop = millis() - t0;
  uint8_t res = 0xFF;
  t0 = millis();
  uint8_t sRes = sensor.get_resolution(&res);
  uint32_t msRes = millis() - t0;
  bool lost = (sRes != 0 || res != resolution);
  uint8_t sInit = 0;
  uint32_t msInit = 0;
  if (lost) {
    st.recReinit++;
    t0 = millis();
    sInit = sensor.init();
    msInit = millis() - t0;
  }
  uint32_t msStart = 0;
  bool ok = false;
  if (sInit == 0) {
    t0 = millis();
    ok = TofConfigureAndStart(resolution, false);
    msStart = millis() - t0;
  }
  uint32_t endMs = millis();
  tofOk = ok;
  if (ok) {
    st.recRestartOk++;
  } else {
    st.recRestartFail++;
  }
  if (print) {
    Serial.printf("[rec]  is_alive=0(%lums) stop=%u(%lums) get_resolution=%u(%u)(%lums) %s", (unsigned long)msAlive, sStop,
                  (unsigned long)msStop, sRes, res, (unsigned long)msRes, lost ? "→ 設定が消えた（リセットの疑い）" : "→ 設定は残っている");
    if (lost) {
      Serial.printf(" init=%u(%lums)", sInit, (unsigned long)msInit);
    }
    Serial.printf(" start %s(%lums) 終了 t=%.3fs 合計%lums\n", ok ? "成功" : "失敗", (unsigned long)msStart,
                  (endMs - st.startMs) / 1000.0f, (unsigned long)(endMs - nowMs));
  }
  lastRecoverMs = endMs;
  if (ok) {
    HealthyBegin(endMs, true);   // 次の最初の失敗までの時間をここから測る
  }
}

static bool printTableNext = false;

static void PrintTable(void) {
  int n = (resolution == VL53L8CX_RESOLUTION_8X8) ? 8 : 4;
  Serial.printf("[tof] table %dx%d 距離mm:状態（5・9＝有効、255＝なし）flip行=%d 列=%d\n", n, n, TOF_FLIP_ROWS, TOF_FLIP_COLS);
  for (int r = 0; r < n; r++) {
    int row = TOF_FLIP_ROWS ? (n - 1 - r) : r;
    char line[128];
    int len = 0;
    for (int c = 0; c < n; c++) {
      int col = TOF_FLIP_COLS ? (n - 1 - c) : c;
      int z = row * n + col;   // 区画の番号（VL53L8CX_NB_TARGET_PER_ZONE は 1）
      len += snprintf(line + len, sizeof(line) - len, " %4d:%-3u",
                      (int)results.distance_mm[z * VL53L8CX_NB_TARGET_PER_ZONE],
                      (unsigned)results.target_status[z * VL53L8CX_NB_TARGET_PER_ZONE]);
    }
    Serial.printf("[tof] r%d%s\n", r, line);
  }
}

static void TofUpdate(uint32_t nowMs) {
  if (!tofStarted) {
    return;
  }
  // 止まっていたら確かめる（やり直しに失敗したあとも、間隔をあけて続ける）
  if (nowMs - lastFrameMs >= TOF_STALL_MS && nowMs - lastRecoverMs >= TOF_RECOVER_RETRY_MS) {
    TofRecover(nowMs);
    return;
  }
  if (!tofOk || nowMs - lastPollMs < TOF_POLL_MS) {
    return;
  }
  BusScope bus(1);
  lastPollMs = nowMs;
  uint8_t ready = 0;
  uint32_t lines0 = logLines;
  uint8_t s = sensor.check_data_ready(&ready);
  if (s != 0) {
    RecordFail(nowMs, FK_READY, s, logLines - lines0);
    return;
  }
  if (!ready) {
    return;
  }
  lines0 = logLines;
  uint32_t t0 = micros();
  s = sensor.get_ranging_data(&results);
  uint32_t us = micros() - t0;
  if (s != 0) {
    RecordFail(millis(), FK_DATA, s, logLines - lines0);
    return;
  }
  lastFrameMs = millis();
  st.frames++;
  st.sumUs += us;
  if (us < st.minUs) st.minUs = us;
  if (us > st.maxUs) st.maxUs = us;
  if (printTableNext) {
    printTableNext = false;
    PrintTable();
  }
}

// 4×4 と 8×8 を切り替える（停止中なので stop_ranging を使ってよい）
static void ToggleResolution(void) {
  BusScope bus(1);
  uint8_t next = (resolution == VL53L8CX_RESOLUTION_8X8) ? VL53L8CX_RESOLUTION_4X4 : VL53L8CX_RESOLUTION_8X8;
  uint8_t s = sensor.stop_ranging();
  Serial.printf("[tof] stop_ranging=%u\n", s);
  tofOk = (s == 0) && TofConfigureAndStart(next, true);
  if (!tofOk) {
    Serial.println("[tof] 切り替え 失敗：測距を止めた（止まったときのやり直しが続く）");
  }
  ResetStats();
}

// Wire1 の速さを 400kHz ⇔ 100kHz
static void ToggleI2cSpeed(void) {
  tofHz = (tofHz == TOF_I2C_HZ) ? TOF_I2C_HZ_SLOW : TOF_I2C_HZ;
  Wire1.setClock(tofHz);
  Serial.printf("[tof] Wire1 %lukHz（getClock=%lu）\n", (unsigned long)(tofHz / 1000), (unsigned long)Wire1.getClock());
  ResetStats();
}

// ---- 首の試験（n・a・v）----
static int neckRound = 0;
static int neckStep = 0;
static uint32_t neckStepStartMs = 0;
static uint32_t neckWaitMs = 0;

static int NeckStepsPerRound(void) {
  return (mode == MODE_V) ? 2 : 3;
}

// 各手順の向き
//   n：左＋上 → 右（上のまま）→ 正面＋水平
//   a：左 → 右 → 正面（上下は水平＋testTiltOff のまま）
//   v：水平＋testTiltOff → 水平（左右は正面のまま）
static void NeckTarget(int step, int *pan, int *tilt) {
  const int left = SERVO1_FRONT_DEG + SERVO1_LEFT_SIGN * SCAN_PAN_DEG;
  const int right = SERVO1_FRONT_DEG - SERVO1_LEFT_SIGN * SCAN_PAN_DEG;
  switch (mode) {
    case MODE_N:
      *pan = (step == 0) ? left : (step == 1) ? right : SERVO1_FRONT_DEG;
      *tilt = (step <= 1) ? SCAN_TILT_DEG : SERVO2_LEVEL_DEG;
      break;
    case MODE_A:
      *pan = (step == 0) ? left : (step == 1) ? right : SERVO1_FRONT_DEG;
      *tilt = SERVO2_LEVEL_DEG + testTiltOff;
      break;
    default:   // MODE_V
      *pan = SERVO1_FRONT_DEG;
      *tilt = (step == 0) ? SERVO2_LEVEL_DEG + testTiltOff : SERVO2_LEVEL_DEG;
      break;
  }
}

static void NeckStart(NeckMode m, int tiltOff) {
  mode = m;
  testTiltOff = tiltOff;
  neckRound = 0;
  neckStep = 0;
  ResetStats();
  // a は先に上下だけ合わせて、待ち時間＋止まる時間のあとで始める（n・v はすぐ始める）
  neckWaitMs = (m == MODE_A) ? NeckMove(SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG + tiltOff) + NECK_DWELL_MS : 0;
  neckStepStartMs = millis();
  if (m == MODE_N) {
    Serial.printf("[neck] 開始 n %d往復 左右±%d 上下%d 止まる%dms\n", NECK_ROUNDS, SCAN_PAN_DEG, SCAN_TILT_DEG, NECK_DWELL_MS);
  } else if (m == MODE_A) {
    Serial.printf("[neck] 開始 a %d往復 左右±%d 上下%+d°（servo2 %d°）止まる%dms\n", NECK_ROUNDS, SCAN_PAN_DEG, tiltOff,
                  SERVO2_LEVEL_DEG + tiltOff, NECK_DWELL_MS);
  } else {
    Serial.printf("[neck] 開始 v %d往復 上下 0 ⇔ %+d°（servo2 %d⇔%d°）止まる%dms\n", NECK_ROUNDS, tiltOff, SERVO2_LEVEL_DEG,
                  SERVO2_LEVEL_DEG + tiltOff, NECK_DWELL_MS);
  }
}

// 集計を出して、首を正面・水平へ戻す
static void NeckFinish(const char *label) {
  PrintStats(label);
  mode = MODE_IDLE;
  NeckMove(SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG);
  ResetStats();
}

static void NeckUpdate(uint32_t nowMs) {
  if ((mode != MODE_N && mode != MODE_A && mode != MODE_V) || nowMs - neckStepStartMs < neckWaitMs) {
    return;
  }
  if (neckRound >= NECK_ROUNDS) {   // 最後の向きの待ちが終わってから集計する
    NeckFinish(mode == MODE_N ? "n" : mode == MODE_A ? "a" : "v");
    return;
  }
  int pan, tilt;
  NeckTarget(neckStep, &pan, &tilt);
  neckStepStartMs = nowMs;
  neckWaitMs = NeckMove(pan, tilt) + NECK_DWELL_MS;
  if (++neckStep >= NeckStepsPerRound()) {
    neckStep = 0;
    ++neckRound;
    if (neckRound % 10 == 0 && neckRound < NECK_ROUNDS) {
      Serial.printf("[neck] %d/%d frames=%lu 失敗=%lu\n", neckRound, NECK_ROUNDS, (unsigned long)st.frames,
                    (unsigned long)(st.readyFail + st.dataFail));
    }
  }
}

// ---- 手で動かす試験（h）----
static uint32_t handStartMs = 0;
static uint32_t handStatusMs = 0;

static void HandStart(void) {
  PrintBattery("hand");
  mode = MODE_H;
  ResetStats();
  {
    BusScope bus(0);
    pca.setChannelPulseWidth(PCA9685_CH_SERVO1, 0);   // 0 ＝ 常に OFF（パルスを出さない）
    pca.setChannelPulseWidth(PCA9685_CH_SERVO2, 0);
  }
  handStartMs = millis();
  handStatusMs = handStartMs;
  handLastPrintMs = 0;
  handSuppressed = 0;
  Serial.println("[hand] 開始：サーボの PWM を止めた。首を手で動かす・線を揺らす。Enter で終わる（首は正面へ戻る）");
}

static void HandUpdate(uint32_t nowMs) {
  if (mode != MODE_H || nowMs - handStatusMs < HAND_STATUS_MS) {
    return;
  }
  handStatusMs = nowMs;
  Serial.printf("[hand] t=%.0fs frames=%lu 失敗=%lu\n", (nowMs - st.startMs) / 1000.0f, (unsigned long)st.frames,
                (unsigned long)(st.readyFail + st.dataFail));
}

// ---- キー ----
static char pendingCmd = 0;            // 数字を待っているキー（a・v）
static char numBuf[6];
static uint8_t numLen = 0;
static uint32_t pendingMs = 0;

static bool TestRunning(void) {
  return mode != MODE_IDLE;
}

static void RunNumberCmd(char cmd, bool hasNum, int num) {
  if (TestRunning()) {
    Serial.println("[neck] 試験中は受け付けない（x で中断）");
    return;
  }
  int off = hasNum ? num : (cmd == 'a' ? 0 : TILT_TEST_DEFAULT);
  int tilt = SERVO2_LEVEL_DEG + off;
  if (tilt < SERVO2_MIN_DEG || tilt > SERVO2_MAX_DEG || (cmd == 'v' && off == 0)) {
    Serial.printf("[neck] %c%+d は実行しない：上下は水平から %+d〜%+d°（servo2 %d〜%d°、水平 %d°、上が +）%s\n", cmd, off,
                  SERVO2_MIN_DEG - SERVO2_LEVEL_DEG, SERVO2_MAX_DEG - SERVO2_LEVEL_DEG, SERVO2_MIN_DEG, SERVO2_MAX_DEG,
                  SERVO2_LEVEL_DEG, (cmd == 'v' && off == 0) ? "。v0 は動かない" : "");
    return;
  }
  if (!PrintBattery("neck")) {
    Serial.printf("[neck] 開始しない：電池が %.1fV 未満\n", BATTERY_SERVO_MIN_V);
    return;
  }
  NeckStart(cmd == 'a' ? MODE_A : MODE_V, off);
}

static void FinishPending(void) {
  numBuf[numLen] = '\0';
  bool hasNum = (numLen > 0) && !(numLen == 1 && numBuf[0] == '-');
  char cmd = pendingCmd;
  pendingCmd = 0;
  RunNumberCmd(cmd, hasNum, hasNum ? atoi(numBuf) : 0);
}

static void HandleKey(char k) {
  switch (k) {
    case 'g':
      if (tofOk) {
        printTableNext = true;
      } else {
        Serial.println("[tof] 測距していない");
      }
      break;
    case 'r':
      if (TestRunning()) {
        Serial.println("[tof] 試験中は切り替えない");
      } else if (tofOk) {
        ToggleResolution();
      } else {
        Serial.println("[tof] 測距していない");
      }
      break;
    case 'c':
      if (TestRunning()) {
        Serial.println("[tof] 試験中は切り替えない");
      } else {
        ToggleI2cSpeed();
      }
      break;
    case 's':
      PrintStats(TestRunning() ? "途中" : "stats");
      PrintBattery("stats");
      if (!TestRunning()) {
        ResetStats();
      }
      break;
    case 'n':
      if (TestRunning()) {
        Serial.println("[neck] 試験中は受け付けない（x で中断）");
        break;
      }
      if (PrintBattery("neck")) {
        NeckStart(MODE_N, 0);
      } else {
        Serial.printf("[neck] 開始しない：電池が %.1fV 未満\n", BATTERY_SERVO_MIN_V);
      }
      break;
    case 'a':
    case 'v':
      pendingCmd = k;
      numLen = 0;
      pendingMs = millis();
      break;
    case 'h':
      if (TestRunning()) {
        Serial.println("[neck] 試験中は受け付けない（x で中断）");
      } else {
        HandStart();
      }
      break;
    case 'x':
      if (mode == MODE_N || mode == MODE_A || mode == MODE_V) {
        NeckFinish("中断");
      }
      break;
    case 'b':
      PrintBattery("now");
      break;
    default:
      break;
  }
}

static void HandleChar(char c, uint32_t nowMs) {
  // 手で動かす試験の間は Enter だけを見る
  if (mode == MODE_H) {
    if ((c == '\r' || c == '\n') && nowMs - handStartMs >= HAND_ENTER_IGNORE_MS) {
      NeckFinish("hand");
    }
    return;
  }
  if (pendingCmd != 0) {
    if ((c >= '0' && c <= '9') || (c == '-' && numLen == 0) || (c == '+' && numLen == 0)) {
      if (c != '+' && numLen < sizeof(numBuf) - 1) {
        numBuf[numLen++] = c;
      }
      pendingMs = nowMs;
      return;
    }
    FinishPending();
    if (c == '\r' || c == '\n') {
      return;
    }
  }
  HandleKey(c);
}

void setup() {
  Serial.begin(115200);
  ets_install_putc1(LogPutc);   // Serial.begin のあとで（begin が出口を入れ直すため）
  esp_log_set_vprintf(LogVprintf);
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    // シリアルモニタがつながるのを少し待つ
  }
  Serial.println("[tof] l8cx_ranging 開始（キー g r s n a v h c x b）");
  BatterySetup();
  PrintBattery("boot");
  PcaSetup();
  TofSetup();
  Serial.printf("[tof] Wire1 %lukHz\n", (unsigned long)(tofHz / 1000));
  ResetStats();
}

void loop() {
  uint32_t now = millis();
  while (Serial.available() > 0) {
    HandleChar((char)Serial.read(), now);
  }
  if (pendingCmd != 0 && now - pendingMs >= KEY_NUMBER_TIMEOUT_MS) {
    FinishPending();
  }
  NeckUpdate(now);
  HandUpdate(now);
  TofUpdate(now);
}
