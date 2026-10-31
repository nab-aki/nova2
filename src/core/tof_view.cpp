#include "tof_view.h"

#include <math.h>

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_servo.h"
#include "../hal/hal_tof.h"
#include "motion.h"
#include "neck.h"
#include "obstacle.h"

#define FLOOR_ROW_FIRST 5   // 床が写る行（首が水平のとき。r5〜r7）
#define FLOOR_ROWS      3
#define COLS            8
#define NO_COLUMN       (-100)

// マスの下の縁の角度で計算した床の想定距離（mm）。計算は measurements.md「床の写り方の気づき」
static float floorExpectedMm[FLOOR_ROWS];

static TofFrame frame;                 // 最新の1枚（loop の中だけで使う）
static bool haveFrame = false;
static unsigned long lastUnstableMs = 0;   // 車体か首が動いていた最後の時刻
static bool wasTofStarted = false;
static bool startReported = false;

// 影の判定
static bool shadowBlocked = false;
static int nearestMm = -1;
static int nearRun = 0;                // 通路の中の障害物が続いた枚数
static int nearRunColumn = NO_COLUMN;  // 前の枚で最も近かったマスの列
static int clearRun = 0;               // 通路が「なし」の距離まで空いている枚が続いた数

// 比較の記録（起動からの累計）
static uint32_t framesJudged = 0;      // 判定に使った枚数
static uint32_t framesSkipped = 0;     // 車体か首の動きをまたいだので捨てた枚数
static uint32_t shadowOnsets = 0;      // 影の「あり」になった回数（前進中）
static uint32_t shadowOnsetsMatched = 0;  // うち、超音波も前後 TOF_US_MATCH_MS で「あり」になった回数
static uint32_t usOnsets = 0;          // 超音波の「あり」になった回数（前進中）
static uint32_t usOnsetsMatched = 0;   // うち、影も前後 TOF_US_MATCH_MS で「あり」だった回数
static long leadSumMs = 0;             // 影が超音波より先に「あり」になった時間の合計（正＝影が先）
static uint32_t leadCount = 0;
static uint32_t emergencyFrames = 0;   // 通路の中で非常停止の距離より近かった枚数
static uint32_t tooNearZones = 0;      // 状態によらず「近すぎ」としたマスの数
static uint32_t staleEvents = 0;       // データの途切れの回数
static uint32_t staleMaxMs = 0;        // 最も長い途切れ
static bool inStale = false;

static bool usBlockedBefore = false;
static unsigned long lastUsOnsetMs = 0;
static unsigned long lastShadowOnsetMs = 0;
static bool shadowOnsetWaiting = false;   // 影が先に「あり」になり、超音波を待っている
static bool usOnsetWaiting = false;       // 超音波が先に「あり」になり、影を待っている

// 床のマスの分布（走行中と静止中を分ける）。比＝値 ÷ 床の想定距離
static uint16_t floorHist[2][FLOOR_ROWS][TOF_FLOOR_RATIO_BINS];
static uint32_t floorInvalid[2][FLOOR_ROWS];   // 測れずのマスの数
// 列ごとの比の平均（静止中。列によって床の値に差があるかを見る）
static float floorColumnSum[FLOOR_ROWS][COLS];
static uint32_t floorColumnCount[FLOOR_ROWS][COLS];

static inline bool IsValidStatus(uint8_t status) {
  return status == TOF_STATUS_VALID_A || status == TOF_STATUS_VALID_B;
}

// 状態によらず近いとみなす距離か。0 は「値がない」かもしれないので入れない（厚紙で 20〜30mm が状態 255 で出た）
static inline bool IsTooNear(int distanceMm) {
  return distanceMm > 0 && distanceMm < TOF_TOO_NEAR_MM;
}

// 首を実際に向けている角度から、右へのずらし α を求める（段0 は 0°、段1 から走行姿勢で 8°）
static float PanTrimRightDeg(void) {
  return (float)((SERVO1_FRONT_DEG - Servo_GetAngle(SERVO_PAN)) * SERVO1_LEFT_SIGN);
}

// マスの点をセンサーからの前後 x・中心線からの左右 y（左が正）・床からの高さ z に直す（common_tof_scan.md 3.1）
static void ZonePoint(int row, int col, int distanceMm, float trimRightDeg, float *x, float *y, float *z) {
  float az = ((3.5f - col) * TOF_ZONE_DEG - trimRightDeg) * DEG_TO_RAD;
  float el = (3.5f - row) * TOF_ZONE_DEG * DEG_TO_RAD;
  float flat = distanceMm * cosf(el);
  *x = flat * cosf(az);
  *y = flat * sinf(az) + TOF_SENSOR_Y_MM;
  *z = TOF_SENSOR_HEIGHT_MM + distanceMm * sinf(el);
}

// 通路の中にあるか。上の段（r0〜r4）だけを見る（床のマスは段2 まで止める判断に使わない。3.4）
static bool InCorridor(float y, float z) {
  return fabsf(y) < (TOF_BODY_HALF_WIDTH_MM + TOF_CORRIDOR_MARGIN_MM) && z < TOF_HEAD_CLEAR_MM;
}

static void RecordFloor(bool moving) {
  int kind = moving ? 1 : 0;
  for (int i = 0; i < FLOOR_ROWS; i++) {
    int row = FLOOR_ROW_FIRST + i;
    for (int col = 0; col < COLS; col++) {
      int z = row * COLS + col;
      if (!IsValidStatus(frame.status[z])) {
        floorInvalid[kind][i]++;
        continue;
      }
      float ratio = frame.distanceMm[z] / floorExpectedMm[i];
      int bin = (int)((ratio - TOF_FLOOR_RATIO_MIN) / TOF_FLOOR_RATIO_STEP);
      bin = constrain(bin, 0, TOF_FLOOR_RATIO_BINS - 1);
      floorHist[kind][i][bin]++;
      if (!moving) {
        floorColumnSum[i][col] += ratio;
        floorColumnCount[i][col]++;
      }
    }
  }
}

// 影の「あり」「超音波のあり」の対応を数える。先に立ったほうが相手を待つ
static void RecordShadowOnset(unsigned long nowMs) {
  shadowOnsets++;
  lastShadowOnsetMs = nowMs;
  if (usOnsetWaiting && nowMs - lastUsOnsetMs <= TOF_US_MATCH_MS) {
    usOnsetWaiting = false;
    usOnsetsMatched++;
    shadowOnsetsMatched++;
    leadSumMs -= (long)(nowMs - lastUsOnsetMs);
    leadCount++;
    return;
  }
  shadowOnsetWaiting = true;
}

static void RecordUsOnset(unsigned long nowMs) {
  usOnsets++;
  lastUsOnsetMs = nowMs;
  if (shadowOnsetWaiting && nowMs - lastShadowOnsetMs <= TOF_US_MATCH_MS) {
    shadowOnsetWaiting = false;
    shadowOnsetsMatched++;
    usOnsetsMatched++;
    leadSumMs += (long)(nowMs - lastShadowOnsetMs);
    leadCount++;
    return;
  }
  if (shadowBlocked) {
    usOnsetsMatched++;   // 影はもっと前から「あり」のまま（先の時間は数えない）
    return;
  }
  usOnsetWaiting = true;
}

// 1枚を判定する。首が水平で、車体と首が1枚の間ずっと動いていないときだけ呼ぶ
// （走行中の直進は「動いていない」に入れる。回転と首の動きだけを捨てる）
static void JudgeFrame(bool movingForward, unsigned long nowMs) {
  framesJudged++;
  if (movingForward || Motion_IsStill()) {
    RecordFloor(movingForward);   // 後退中は傾きが違うので、どちらにも入れない
  }

  float trim = PanTrimRightDeg();
  float bestX = 1e9f;
  int bestCol = NO_COLUMN;
  bool anyEmergency = false;
  for (int row = 0; row < FLOOR_ROW_FIRST; row++) {
    for (int col = 0; col < COLS; col++) {
      int z = row * COLS + col;
      int d = frame.distanceMm[z];
      bool valid = IsValidStatus(frame.status[z]);
      bool tooNear = IsTooNear(d);
      if (tooNear && !valid) {
        tooNearZones++;
      }
      if (d <= 0 || (!valid && !tooNear)) {
        continue;   // 測れず：空きにも近いにも数えない
      }
      float x, y, h;
      ZonePoint(row, col, d, trim, &x, &y, &h);
      if (!InCorridor(y, h)) {
        continue;
      }
      if (x < bestX) {
        bestX = x;
        bestCol = col;
      }
      if (x < TOF_EMERGENCY_MM) {
        anyEmergency = true;
      }
    }
  }
  if (anyEmergency) {
    emergencyFrames++;
  }
  nearestMm = (bestCol == NO_COLUMN) ? -1 : (int)bestX;

  // 「あり」：通路の中の障害物が2枚続く（2枚の間で列のずれは ±1 まで）
  bool near = bestCol != NO_COLUMN && bestX < TOF_STOP_MM;
  if (near && (nearRun == 0 || abs(bestCol - nearRunColumn) <= 1)) {
    nearRun++;
  } else {
    nearRun = near ? 1 : 0;
  }
  nearRunColumn = near ? bestCol : NO_COLUMN;

  // 「なし」：通路に「停止距離＋余裕」より近いものがない枚が続く
  bool clear = bestCol == NO_COLUMN || bestX >= TOF_STOP_MM + TOF_CLEAR_MARGIN_MM;
  clearRun = clear ? clearRun + 1 : 0;

  bool before = shadowBlocked;
  if (!shadowBlocked && nearRun >= TOF_NEAR_FRAMES) {
    shadowBlocked = true;
  } else if (shadowBlocked && clearRun >= TOF_NEAR_FRAMES) {
    shadowBlocked = false;
  }
  if (shadowBlocked != before) {
    Log_Printf("ToF", "影の判定：%s（通路の中の最も近い距離 %dmm。超音波は %s）",
               shadowBlocked ? "あり" : "なし", nearestMm, Obstacle_IsBlocked() ? "あり" : "なし");
    // 超音波が先に止めた回は、速度が 0 でも対応を取る（止めたあとの滑走中に影が「あり」になる）
    if (shadowBlocked && (movingForward || usOnsetWaiting)) {
      RecordShadowOnset(nowMs);
    }
  }
}

void TofView_Setup(void) {
  for (int i = 0; i < FLOOR_ROWS; i++) {
    float edgeDeg = ((FLOOR_ROW_FIRST + i) + 1 - 4) * TOF_ZONE_DEG;   // r5 → 11.25°、r6 → 16.875°、r7 → 22.5°
    floorExpectedMm[i] = TOF_SENSOR_HEIGHT_MM / sinf(edgeDeg * DEG_TO_RAD);
  }
  frame = TofFrame{};
  Tof_Setup();
  Log_Printf("ToF", "段0：判定は記録するだけで、動きには使いません（超音波で走ります）");
}

// 起動の結果を1回だけ出す（タスクからはログを出さないので、loop で出す）
static void ReportStart(void) {
  if (startReported) {
    return;
  }
  TofStats s;
  Tof_GetStats(&s);
  switch (s.state) {
    case TOF_STATE_RUNNING:
      Log_Printf("ToF", "起動：is_alive %d回目で成功、init %lums、8×8 %dHz", s.aliveTries, (unsigned long)s.initMs, TOF_FREQ_HZ);
      wasTofStarted = true;
      break;
    case TOF_STATE_FAILED:
      Log_Printf("ToF", "起動に失敗：%s=%u（is_alive %d回 alive=%u SDA=%d SCL=%d）。この起動中は使いません（段0 は超音波で走り続けます）",
                 s.failName != NULL ? s.failName : "?", s.failStep, s.aliveTries, s.alive, s.sdaLevel, s.sclLevel);
      break;
    case TOF_STATE_DISABLED:
      Log_Printf("ToF", "このビルドでは使いません（TOF_ENABLE=0）");
      break;
    default:
      return;   // まだ起動中
  }
  startReported = true;
}

// 前進中の超音波の「あり」を数える（影と比べるため）
static void WatchUltrasonic(unsigned long nowMs) {
  bool usBlocked = Obstacle_IsBlocked();
  if (usBlocked && !usBlockedBefore && Motion_GetSpeed() >= MOTOR_SPEED_EPSILON) {
    RecordUsOnset(nowMs);
  }
  usBlockedBefore = usBlocked;
}

// 相手が時間内に来なかった「待ち」を片付ける
static void ExpireWaiting(unsigned long nowMs) {
  if (shadowOnsetWaiting && nowMs - lastShadowOnsetMs > TOF_US_MATCH_MS) {
    shadowOnsetWaiting = false;
  }
  if (usOnsetWaiting && nowMs - lastUsOnsetMs > TOF_US_MATCH_MS) {
    usOnsetWaiting = false;
  }
}

void TofView_Update(unsigned long nowMs) {
  ReportStart();
  WatchUltrasonic(nowMs);
  ExpireWaiting(nowMs);

  // 回転中と、首が動いている（または水平でない）間は「動いていた」とする
  bool neckOk = Neck_IsSteady(nowMs) && Servo_GetAngle(SERVO_TILT) == SERVO2_LEVEL_DEG &&
                abs(Servo_GetAngle(SERVO_PAN) - SERVO1_FRONT_DEG) <= NECK_DRIVE_TRIM_RIGHT_DEG;
  if (Motion_IsTurning() || !neckOk) {
    lastUnstableMs = nowMs;
  }

  if (!wasTofStarted) {
    return;
  }

  // 途切れの見張り（段0 は数えるだけ）
  if (haveFrame) {
    uint32_t age = (uint32_t)max((long)(nowMs - frame.readyMs), 0L);
    if (age > TOF_STALE_MS) {
      if (!inStale) {
        inStale = true;
        staleEvents++;
        Log_Printf("ToF", "データが途切れています（最新の1枚 %lums 前。段0 は止まりません）", (unsigned long)age);
      }
      if (age > staleMaxMs) staleMaxMs = age;
    } else {
      inStale = false;
    }
  }

  if (!Tof_GetLatest(frame.seq, &frame)) {
    return;
  }
  haveFrame = true;

  // 1枚の間に動きが終わっていない枚は捨てる（読み終えた時刻 − 1枚の時間 が、動きの終わりより後の枚だけを使う）
  if ((long)(frame.readyMs - TOF_FRAME_MS - lastUnstableMs) <= 0) {
    framesSkipped++;
    return;
  }
  JudgeFrame(Motion_GetSpeed() >= MOTOR_SPEED_EPSILON, nowMs);
}

bool TofView_ShadowBlocked(void) {
  return shadowBlocked;
}

int TofView_NearestMm(void) {
  return nearestMm;
}

void TofView_FormatStatus(char *buffer, size_t size, unsigned long nowMs) {
  if (!wasTofStarted) {
    snprintf(buffer, size, "なし");
    return;
  }
  if (!haveFrame) {
    snprintf(buffer, size, "待ち");
    return;
  }
  // 1枚は loop の先頭で取った nowMs より新しいことがある（引き算の桁あふれを防ぐ）
  long ageMs = max((long)(nowMs - frame.readyMs), 0L);
  snprintf(buffer, size, "%s(%dmm,%ldms前)", shadowBlocked ? "あり" : "なし", nearestMm, ageMs);
}

static void PrintFloor(void) {
  static const char *const KIND_NAMES[2] = {"静止", "走行"};
  Log_Printf("ToF", "床のマス：値 ÷ 想定距離 の分布（%.2f から %.2f 刻み。外は端に入れる。マスの数）",
             TOF_FLOOR_RATIO_MIN, TOF_FLOOR_RATIO_STEP);
  for (int kind = 0; kind < 2; kind++) {
    for (int i = 0; i < FLOOR_ROWS; i++) {
      char line[160];
      int used = 0;
      for (int b = 0; b < TOF_FLOOR_RATIO_BINS && used < (int)sizeof(line); b++) {
        used += snprintf(line + used, sizeof(line) - used, " %u", floorHist[kind][i][b]);
      }
      Log_Printf("ToF", "  %s r%d（想定 %.0fmm）:%s 測れず %lu", KIND_NAMES[kind], FLOOR_ROW_FIRST + i,
                 floorExpectedMm[i], line, (unsigned long)floorInvalid[kind][i]);
    }
  }
  for (int i = 0; i < FLOOR_ROWS; i++) {
    char line[96];
    int used = 0;
    for (int col = 0; col < COLS && used < (int)sizeof(line); col++) {
      float mean = floorColumnCount[i][col] > 0 ? floorColumnSum[i][col] / floorColumnCount[i][col] : 0.0f;
      used += snprintf(line + used, sizeof(line) - used, " %.2f", mean);
    }
    Log_Printf("ToF", "  静止 r%d の列ごとの比の平均（c0〜c7）:%s", FLOOR_ROW_FIRST + i, line);
  }
}

void TofView_Print(void) {
  TofStats s;
  Tof_GetStats(&s);
  if (s.state == TOF_STATE_DISABLED) {
    Log_Printf("ToF", "このビルドでは使いません（TOF_ENABLE=0）");
    return;
  }
  if (s.state != TOF_STATE_RUNNING) {
    Log_Printf("ToF", "測距していません（%s）。%s=%u is_alive %d回 alive=%u SDA=%d SCL=%d",
               s.state == TOF_STATE_STARTING ? "起動中" : "起動に失敗",
               s.failName != NULL ? s.failName : "-", s.failStep, s.aliveTries, s.alive, s.sdaLevel, s.sclLevel);
    return;
  }
  Log_Printf("ToF", "枚数 %lu 失敗 ready %lu／data %lu 周期 平均 %.1fms／最大 %lums 読み出し 平均 %.1fms／最大 %.1fms 途切れ %lu回（最長 %lums） スタックの残り %lu",
             (unsigned long)s.frames, (unsigned long)s.readyFail, (unsigned long)s.dataFail,
             s.periodCount > 0 ? (float)s.periodSumMs / s.periodCount : 0.0f, (unsigned long)s.periodMaxMs,
             s.frames > 0 ? s.readSumUs / 1000.0f / s.frames : 0.0f, s.readMaxUs / 1000.0f,
             (unsigned long)staleEvents, (unsigned long)staleMaxMs, (unsigned long)s.stackFree);
  Log_Printf("ToF", "影の比較（起動から・前進中）：影の「あり」%lu回（超音波も ±%dms で「あり」%lu回）／超音波の「あり」%lu回（影も %lu回）／両方の回で影が先の平均 %ldms",
             (unsigned long)shadowOnsets, TOF_US_MATCH_MS, (unsigned long)shadowOnsetsMatched,
             (unsigned long)usOnsets, (unsigned long)usOnsetsMatched,
             leadCount > 0 ? leadSumMs / (long)leadCount : 0L);
  Log_Printf("ToF", "判定に使った %lu 枚／動きをまたいで捨てた %lu 枚 非常停止の距離より近い枚 %lu 状態によらず「近すぎ」のマス %lu いま：%s",
             (unsigned long)framesJudged, (unsigned long)framesSkipped, (unsigned long)emergencyFrames,
             (unsigned long)tooNearZones, shadowBlocked ? "あり" : "なし");
  PrintFloor();
}

void TofView_PrintFrame(void) {
  if (!haveFrame) {
    Log_Printf("ToF", "まだ1枚も届いていません");
    return;
  }
  Log_Printf("ToF", "最新の1枚 #%lu（行＝上下 r0 上、列＝左右 c0 左。距離mm:状態。*＝通路の中で %dmm より近い）",
             (unsigned long)frame.seq, TOF_STOP_MM);
  float trim = PanTrimRightDeg();
  for (int row = 0; row < 8; row++) {
    char line[128];
    int used = 0;
    for (int col = 0; col < COLS && used < (int)sizeof(line); col++) {
      int z = row * COLS + col;
      int d = frame.distanceMm[z];
      float x, y, h;
      ZonePoint(row, col, d, trim, &x, &y, &h);
      bool usable = d > 0 && (IsValidStatus(frame.status[z]) || IsTooNear(d));
      bool mark = row < FLOOR_ROW_FIRST && usable && InCorridor(y, h) && x < TOF_STOP_MM;
      used += snprintf(line + used, sizeof(line) - used, " %4d:%-3u%s", d, (unsigned)frame.status[z], mark ? "*" : " ");
    }
    Log_Raw("r%d%s", row, line);
  }
}
