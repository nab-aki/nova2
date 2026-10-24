// VL53L8CX：測距の時間・失敗の回数・区画の表と、首を振りながらの測距を測る（スプリント3.6-1。単体試験）
// ライブラリは Nova の lib/（STM32duino_VL53L8CX は NOVA_PATCH.md の改変版、PCA9685 は失敗を数える改変版）
//
// 起動時：is_alive → init → 測距の頻度を設定 → 解像度 8×8 → start_ranging。どれかが失敗したら、その旨を出して測距しない
// 測距：check_data_ready が真のときだけ get_ranging_data を呼び、かかった時間（µs）と失敗を数える
// キー（シリアルに1文字送る）：
//   g … 距離（mm）と区画の状態（target_status）を1回だけ表で出す
//   r … 4×4 と 8×8 を切り替える（stop_ranging → set_resolution → 頻度の設定 → start_ranging）
//   s … ここまでの集計を出して、数え直す
//   n … 首を本体のキー u と同じ動き（正面 → 左＋上 → 右 → 正面）で 50 往復。終わったら集計を出す
// モーターは動かさない：PCA9685 のモーターのチャンネルには何も書かない
//   （起動時の setupSingleDevice のソフトウェアリセットで、全チャンネルが止まった状態になる。本体と同じ）
// delay() は使わない（ライブラリの中の待ちは除く）
#include <Arduino.h>
#include <Wire.h>
#include <PCA9685.h>
#include <vl53l8cx.h>

// ---- ToF（第2バス）----
#define PIN_TOF_SDA         26        // ESP32 基板のヘッダー P3-10
#define PIN_TOF_SCL         27        // ESP32 基板のヘッダー P3-11
#define TOF_I2C_HZ          400000    // 400kHz（線 13cm。2026-10-24 の起動試験で成功）
#define TOF_FREQ_HZ         10        // 測距の頻度（4×4・8×8 の両方で使える値。8×8 の上限は 15Hz）
#define TOF_POLL_MS         5         // check_data_ready を呼ぶ間隔
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
#define SERVO2_MAX_DEG      140
#define SCAN_PAN_DEG        50        // 左右（正面 ±50°。WANDER_SCAN_PAN_DEG）
#define SCAN_TILT_DEG       113       // キー u の上下（DEBUG_SCAN_TILT_DEG）
#define NECK_SETTLE_BASE_MS 60        // 首を動かしたら待つ時間（本体と同じ式）
#define NECK_SETTLE_PER_DEG_MS 2
#define NECK_DWELL_MS       300       // 各向きで止まっている時間（本体の超音波3回の測距の代わり。試験用）
#define NECK_ROUNDS         50        // 往復の回数

static VL53L8CX sensor(&Wire1, -1);
static PCA9685 pca;
static VL53L8CX_ResultsData results;   // 約1.4KB。スタックに置かない

static bool tofOk = false;             // 起動・設定がすべて成功し、測距中
static uint8_t resolution = VL53L8CX_RESOLUTION_8X8;
static uint32_t lastPollMs = 0;

// ---- 集計 ----
struct Stats {
  uint32_t frames;        // get_ranging_data が成功した回数
  uint32_t readyFail;     // check_data_ready の失敗
  uint32_t dataFail;      // get_ranging_data の失敗
  uint32_t minUs, maxUs;
  uint64_t sumUs;
  uint32_t startMs;
  uint32_t pcaTotal0, pcaFail0;
};
static Stats st;

static void ResetStats(void) {
  st.frames = 0;
  st.readyFail = 0;
  st.dataFail = 0;
  st.minUs = UINT32_MAX;
  st.maxUs = 0;
  st.sumUs = 0;
  st.startMs = millis();
  st.pcaTotal0 = pca.nova_i2c_total;
  st.pcaFail0 = pca.nova_i2c_fail;
}

static void PrintStats(const char *label) {
  uint32_t avg = st.frames ? (uint32_t)(st.sumUs / st.frames) : 0;
  uint32_t mn = st.frames ? st.minUs : 0;
  Serial.printf("[tof] %s res=%s frames=%lu sec=%.1f us min=%lu avg=%lu max=%lu fail ready=%lu data=%lu pca=%lu/%lu\n",
                label, resolution == VL53L8CX_RESOLUTION_8X8 ? "8x8" : "4x4",
                (unsigned long)st.frames, (millis() - st.startMs) / 1000.0f,
                (unsigned long)mn, (unsigned long)avg, (unsigned long)st.maxUs,
                (unsigned long)st.readyFail, (unsigned long)st.dataFail,
                (unsigned long)(pca.nova_i2c_fail - st.pcaFail0), (unsigned long)(pca.nova_i2c_total - st.pcaTotal0));
}

// ---- 首 ----
static uint16_t AngleToPulseWidth(int deg) {
  return (uint16_t)map(constrain(deg, 0, 180), 0, 180, 102, 512);   // 本体の hal_servo と同じ
}

static int panDeg = SERVO1_FRONT_DEG;
static int tiltDeg = SERVO2_LEVEL_DEG;

// 首を動かし、動かした角度の大きいほうから待ち時間（ms）を返す
static uint32_t NeckMove(int pan, int tilt) {
  pan = constrain(pan, SERVO1_MIN_DEG, SERVO1_MAX_DEG);
  tilt = constrain(tilt, SERVO2_MIN_DEG, SERVO2_MAX_DEG);
  int moved = max(abs(pan - panDeg), abs(tilt - tiltDeg));
  pca.setChannelPulseWidth(PCA9685_CH_SERVO1, AngleToPulseWidth(pan));
  pca.setChannelPulseWidth(PCA9685_CH_SERVO2, AngleToPulseWidth(tilt));
  panDeg = pan;
  tiltDeg = tilt;
  return NECK_SETTLE_BASE_MS + (uint32_t)moved * NECK_SETTLE_PER_DEG_MS;
}

// 本体と同じ PCA9685 の設定（ソフトウェアリセット → ALLCALL を切る → 50Hz）
static void PcaSetup(void) {
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

// 首を振る試験（キー n）。手順：正面 → 左＋上 → 右（上のまま）→ 正面＋水平。各向きで待ち時間＋NECK_DWELL_MS 止まる
static bool neckRunning = false;
static int neckRound = 0;
static int neckStep = 0;
static uint32_t neckStepStartMs = 0;
static uint32_t neckWaitMs = 0;

static void NeckStart(void) {
  neckRunning = true;
  neckRound = 0;
  neckStep = 0;
  neckWaitMs = 0;
  neckStepStartMs = millis();
  ResetStats();
  Serial.printf("[neck] 開始 %d往復 左右±%d 上下%d 止まる%dms\n", NECK_ROUNDS, SCAN_PAN_DEG, SCAN_TILT_DEG, NECK_DWELL_MS);
}

static void NeckUpdate(uint32_t nowMs) {
  if (!neckRunning || nowMs - neckStepStartMs < neckWaitMs) {
    return;
  }
  uint32_t settle = 0;
  switch (neckStep) {
    case 0: settle = NeckMove(SERVO1_FRONT_DEG + SERVO1_LEFT_SIGN * SCAN_PAN_DEG, SCAN_TILT_DEG); break;   // 左＋上
    case 1: settle = NeckMove(SERVO1_FRONT_DEG - SERVO1_LEFT_SIGN * SCAN_PAN_DEG, SCAN_TILT_DEG); break;   // 右
    default: settle = NeckMove(SERVO1_FRONT_DEG, SERVO2_LEVEL_DEG); break;                                  // 正面
  }
  neckStepStartMs = nowMs;
  neckWaitMs = settle + NECK_DWELL_MS;
  if (++neckStep >= 3) {
    neckStep = 0;
    if (++neckRound >= NECK_ROUNDS) {
      neckRunning = false;
      neckWaitMs = 0;
      PrintStats("neck");
      ResetStats();
      return;
    }
    if (neckRound % 10 == 0) {
      Serial.printf("[neck] %d/%d\n", neckRound, NECK_ROUNDS);
    }
  }
}

// ---- ToF ----
// 測距の頻度・解像度を設定して始める。失敗したら tofOk を false にする
static bool TofConfigureAndStart(uint8_t res) {
  uint8_t s = sensor.set_resolution(res);
  Serial.printf("[tof] set_resolution(%s)=%u\n", res == VL53L8CX_RESOLUTION_8X8 ? "8x8" : "4x4", s);
  if (s != 0) return false;
  s = sensor.set_ranging_frequency_hz(TOF_FREQ_HZ);
  Serial.printf("[tof] set_ranging_frequency_hz(%d)=%u\n", TOF_FREQ_HZ, s);
  if (s != 0) return false;
  s = sensor.start_ranging();
  Serial.printf("[tof] start_ranging=%u\n", s);
  if (s != 0) return false;
  resolution = res;
  return true;
}

static void TofSetup(void) {
  Wire1.begin(PIN_TOF_SDA, PIN_TOF_SCL, TOF_I2C_HZ);
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
  tofOk = TofConfigureAndStart(VL53L8CX_RESOLUTION_8X8);
  if (!tofOk) {
    Serial.println("[tof] 設定 失敗：測距しない");
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
  if (!tofOk || nowMs - lastPollMs < TOF_POLL_MS) {
    return;
  }
  lastPollMs = nowMs;
  uint8_t ready = 0;
  if (sensor.check_data_ready(&ready) != 0) {
    st.readyFail++;
    return;
  }
  if (!ready) {
    return;
  }
  uint32_t t0 = micros();
  uint8_t s = sensor.get_ranging_data(&results);
  uint32_t us = micros() - t0;
  if (s != 0) {
    st.dataFail++;
    return;
  }
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
  uint8_t next = (resolution == VL53L8CX_RESOLUTION_8X8) ? VL53L8CX_RESOLUTION_4X4 : VL53L8CX_RESOLUTION_8X8;
  uint8_t s = sensor.stop_ranging();
  Serial.printf("[tof] stop_ranging=%u\n", s);
  tofOk = (s == 0) && TofConfigureAndStart(next);
  if (!tofOk) {
    Serial.println("[tof] 切り替え 失敗：測距を止めた");
  }
  ResetStats();
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
      if (neckRunning) {
        Serial.println("[tof] 首の試験中は切り替えない");
      } else if (tofOk) {
        ToggleResolution();
      } else {
        Serial.println("[tof] 測距していない");
      }
      break;
    case 's':
      PrintStats("stats");
      ResetStats();
      break;
    case 'n':
      if (!neckRunning) NeckStart();
      break;
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    // シリアルモニタがつながるのを少し待つ
  }
  Serial.println("[tof] l8cx_ranging 開始（キー g r s n）");
  PcaSetup();
  TofSetup();
  ResetStats();
}

void loop() {
  uint32_t now = millis();
  while (Serial.available() > 0) {
    HandleKey((char)Serial.read());
  }
  NeckUpdate(now);
  TofUpdate(now);
}
