#include "tof_view.h"

#include <math.h>

#include "../config.h"
#include "../hal/hal_log.h"
#include "../hal/hal_servo.h"
#include "../hal/hal_tof.h"
#include "gyro.h"
#include "motion.h"
#include "neck.h"
#include "obstacle.h"
#include "safety.h"
#include "sensors.h"
#include "test_stats.h"
#include "tof_look.h"

#define FLOOR_ROW_FIRST 5   // 床が写る行（首が水平のとき。r5〜r7）
#define FLOOR_ROWS      3
#define COLS            8
#define NO_COLUMN       (-100)

// マスの下の縁の角度で計算した床の想定距離（mm）。計算は measurements.md「床の写り方の気づき」
static float floorExpectedMm[FLOOR_ROWS];

static TofFrame frame;                 // 最新の1枚（loop の中だけで使う）
static bool haveFrame = false;
static unsigned long lastUnstableMs = 0;   // 車体か首が動いていた最後の時刻
static unsigned long lastMoveMs = 0;       // 車体か首が動いていた最後の時刻（首の向きは問わない。前進も入れる）

// 前端の隙間（直近3枚の、車体の幅の中で孤立していないマスの x − 前端までの距離。なければ負）
#define GAP_FRAMES 3
#define NO_GAP (-1)
struct GapCell {
  int gapMm;
  int row;
  int col;
  int distanceMm;
  uint8_t status;
  float x, y, z;
};
static GapCell gapHistory[GAP_FRAMES];
static int sideHistory[GAP_FRAMES];
static int gapHistoryCount = 0;
static bool stopMinValid = false;
static GapCell stopMin;              // 前端の隙間が最小だった停止
static TofFrame stopMinFrame;        // そのときの1枚
static int stopSideMinMm = NO_GAP;   // 横の余裕の最小
static bool wasTofStarted = false;
static bool startReported = false;

// 判定
static bool judgeKnown = false;        // false＝不明（動いたあと、まだ判定に足りる枚を見ていない）
static int unknownFrames = 0;          // 不明になってから判定に使った枚数
static bool judgedBlocked = false;
static bool emergencyNow = false;
static int nearestMm = -1;
static int lastMm = -1;                // 通路の中の最も近い距離の最新の値（見えなくなっても残す）
static bool frameLost = false;         // データが途切れて止めている
static int recoverRun = 0;             // 途切れのあと続けて届いた枚数
static unsigned long holdUntilMs = 0;  // キー b：この時刻まで1枚を受け取らない
static bool holding = false;
static int nearRun = 0;                // 通路の中の障害物が続いた枚数
static int nearRunColumn = NO_COLUMN;  // 前の枚で最も近かったマスの列
static int clearRun = 0;               // 通路が「なし」の距離まで空いている枚が続いた数
static int blindRun = 0;               // 前進中に見えていない枚が続いた数

// 比較の記録（起動からの累計）
static uint32_t framesJudged = 0;      // 判定に使った枚数
static uint32_t framesSkipped = 0;     // 車体か首の動きをまたいだので捨てた枚数
static uint32_t shadowOnsets = 0;      // 影の「あり」になった回数（前進中）
static uint32_t shadowOnsetsMatched = 0;  // うち、超音波も前後 TOF_US_MATCH_MS で「あり」になった回数
static uint32_t usOnsets = 0;          // 超音波の「あり」になった回数（前進中）
static uint32_t usOnsetsMatched = 0;   // うち、影も前後 TOF_US_MATCH_MS で「あり」だった回数
static long leadSumMs = 0;             // 影が超音波より先に「あり」になった時間の合計（正＝影が先）
static uint32_t leadCount = 0;
static uint32_t emergencyFrames = 0;   // 通路の中で非常停止の距離より近かった枚数（前進中のほか）
static uint32_t emergencyFramesForward = 0;   // 同じく、前進中
static uint32_t emergencyTestFrames = 0;          // 上の2つのうち、試験中で床に置いている間（前進中のほか）
static uint32_t emergencyTestFramesForward = 0;   // 同じく、前進中
// 前進中に判定した最後の1枚（気づくの記録に使う）
static bool lastForwardValid = false;
static int lastForwardMm = -1;            // その枚の、通路の中の最も近い距離。なければ負
static unsigned long lastForwardReadyMs = 0;
static uint16_t skippedSinceForward = 0;  // その枚のあとに捨てた枚
static uint16_t stillSinceForward = 0;    // その枚のあとに判定した、前進中でない枚
static uint32_t tooNearZones = 0;      // 状態によらず「近すぎ」としたマスの数
static uint32_t staleEvents = 0;       // データの途切れの回数
static uint32_t staleMaxMs = 0;        // 最も長い途切れ
static bool inStale = false;

static bool usBlockedBefore = false;
static unsigned long lastUsOnsetMs = 0;
static unsigned long lastShadowOnsetMs = 0;
static bool shadowOnsetWaiting = false;   // 影が先に「あり」になり、超音波を待っている
static bool usOnsetWaiting = false;       // 超音波が先に「あり」になり、影を待っている

// 超音波と ToF が食い違った回の記録。「あり」になった時点で作り始め、相手が時間内に来なければ残す
enum MismatchKind {
  MISMATCH_TOF_ONLY,     // ToF だけ「あり」
  MISMATCH_US_ONLY,      // 超音波だけ「あり」
  MISMATCH_US_OLD,       // 超音波が「あり」になったとき、ToF は前から「あり」のまま
};

// 端の列（左 c0〜c1・右 c6〜c7）の上の段（r0〜r4）の、最も近い点（前後の距離）。斜めの壁がいつから見えていたかを見る
struct EdgeCell {
  int16_t mm;            // センサーから前へ測った距離。なければ負
  int8_t row;
  int8_t col;
  bool inside;           // その点が通路の中か
};

struct PreFrame {
  uint32_t readyMs;      // その枚を読み終えた時刻
  EdgeCell left;
  EdgeCell right;
  int16_t centerMm;      // 中央の列（c3〜c4）の、高さ TOF_HEAD_CLEAR_MM 未満の最も近い前後距離。なければ負
  bool moving;           // 前進の速度が出ていた枚か
};

struct PreCell {
  int16_t agoMs;         // 「あり」になった時刻より何ms前の枚か
  EdgeCell left;
  EdgeCell right;
  int16_t centerMm;
  bool moving;
};

struct MismatchFollow {
  int16_t mm;            // 通路の中の最も近い距離。なければ負
  int8_t row;
  int8_t col;
  uint16_t dtMs;         // 記録を始めてからの時間
  float speed;
};

struct Mismatch {
  uint8_t kind;
  uint32_t timeMs;
  float speed;
  float usStartCm;       // 始めたときの超音波の生値。反応なしは負
  float usMinCm;         // 待っている間の超音波の生値の最小。なければ負
  int16_t nearestMm;     // 始めたときの1枚の、通路の中の最も近い距離。なければ負
  int8_t row;
  int8_t col;
  uint8_t upperInvalid;  // r0〜r4 の測れずのマスの数
  uint16_t frameAgeMs;   // 始めたときの1枚の古さ
  uint16_t judged;       // 待っている間に判定に使った枚数
  uint16_t skipped;      // 待っている間に、動きをまたいで捨てた枚数
  uint8_t followCount;
  MismatchFollow follow[TOF_MISMATCH_FOLLOW];
  uint8_t preCount;
  PreCell pre[TOF_MISMATCH_PRE_FRAMES];   // 「あり」になる前の TOF_MISMATCH_PRE_MS の枚（古い順）
  TofFrame snapshot;     // 始めたときの1枚（r5〜r7 も含む）
};

static PreFrame preRing[TOF_MISMATCH_PRE_FRAMES];   // 判定に使った直近の枚の端の列
static int preHead = 0;                              // 次に書く場所
static int preCount = 0;
static TofEdgeView edgeView;                         // ID27 に渡す、判定に使った最新の枚
static uint32_t edgeViewSeq = 0;

static Mismatch mismatchRing[TOF_MISMATCH_RECORDS];
static uint32_t mismatchTotal = 0;     // 起動からの件数（リングより多いと古いものは消えている）
static Mismatch pendingShadow;         // shadowOnsetWaiting の間、作りかけ
static Mismatch pendingUs;             // usRecordActive の間、作りかけ
static bool usRecordActive = false;

// 床のマスの分布（走行中と静止中を分ける）。比＝値 ÷ 床の想定距離
static uint16_t floorHist[2][FLOOR_ROWS][TOF_FLOOR_RATIO_BINS];
static uint32_t floorInvalid[2][FLOOR_ROWS];   // 測れずのマスの数
// 列ごとの比の平均（静止中。列によって床の値に差があるかを見る）
static float floorColumnSum[FLOOR_ROWS][COLS];
static uint32_t floorColumnCount[FLOOR_ROWS][COLS];

static inline bool IsValidStatus(uint8_t status) {
  return status == TOF_STATUS_VALID_A || status == TOF_STATUS_VALID_B;
}

bool TofView_IsValidStatus(uint8_t status) {
  return IsValidStatus(status);
}

// 状態によらず近いとみなす距離か。0 は「値がない」かもしれないので入れない（厚紙で 20〜30mm が状態 255 で出た）
static inline bool IsTooNear(int distanceMm) {
  return distanceMm > 0 && distanceMm < TOF_TOO_NEAR_MM;
}

bool TofView_IsUsable(uint8_t status, int distanceMm) {
  return distanceMm > 0 && (IsValidStatus(status) || IsTooNear(distanceMm));
}

void TofView_PointForPan(int row, int col, int distanceMm, int panDeg, float *x, float *y, float *z) {
  float pan = (panDeg - SERVO1_FRONT_DEG) * SERVO1_LEFT_SIGN * DEG_TO_RAD;   // 左が正
  float az = (3.5f - col) * TOF_ZONE_DEG * DEG_TO_RAD + pan;
  float el = (3.5f - row) * TOF_ZONE_DEG * DEG_TO_RAD;
  float flat = distanceMm * cosf(el);
  float cosP = cosf(pan);
  float sinP = sinf(pan);
  *x = TOF_NECK_TO_SENSOR_MM * cosP - TOF_SENSOR_Y_MM * sinP - TOF_NECK_TO_SENSOR_MM + flat * cosf(az);
  *y = TOF_NECK_TO_SENSOR_MM * sinP + TOF_SENSOR_Y_MM * cosP + flat * sinf(az);
  *z = TOF_SENSOR_HEIGHT_MM + distanceMm * sinf(el);
}

float TofView_FloorExpectedMm(int row) {
  return floorExpectedMm[row - FLOOR_ROW_FIRST];
}

bool TofView_TakeEdgeView(uint32_t *lastSeq, TofEdgeView *out) {
  if (edgeViewSeq == *lastSeq) {
    return false;
  }
  *lastSeq = edgeViewSeq;
  *out = edgeView;
  return true;
}

bool TofView_TakeFrameCenter(uint32_t *lastSeq, uint32_t *readyMs, int *centerMm) {
  if (!haveFrame || frame.seq == *lastSeq) {
    return false;
  }
  *lastSeq = frame.seq;
  *readyMs = frame.readyMs;
  int pan = Servo_GetAngle(SERVO_PAN);
  float sum = 0.0f;
  int n = 0;
  for (int row = 0; row < FLOOR_ROW_FIRST; row++) {
    for (int col = COLS / 2 - 1; col <= COLS / 2; col++) {
      int z = row * COLS + col;
      int d = frame.distanceMm[z];
      if (!TofView_IsUsable(frame.status[z], d)) {
        continue;
      }
      float x, y, h;
      TofView_PointForPan(row, col, d, pan, &x, &y, &h);
      sum += x;
      n++;
    }
  }
  *centerMm = n >= TOF_VIEW_MIN_VALID + 1 ? (int)(sum / n) : -1;
  return true;
}

bool TofView_TakeStillFrame(uint32_t *lastSeq, const TofFrame **out) {
  if (!haveFrame || frame.seq == *lastSeq) {
    return false;
  }
  *lastSeq = frame.seq;
  if ((long)(frame.readyMs - TOF_FRAME_MS - lastMoveMs) <= 0) {
    return false;
  }
  *out = &frame;
  return true;
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

// 上の段（r0〜r4）のマスを見て、通路の中の最も近い点を探す
struct CorridorScan {
  float bestX;
  int bestRow;
  int bestCol;
  bool emergency;
  int tooNear;        // 状態によらず「近すぎ」としたマスの数
  int upperInvalid;   // 測れずのマスの数
};

static void ScanCorridor(CorridorScan *out) {
  int pan = Servo_GetAngle(SERVO_PAN);
  out->bestX = 1e9f;
  out->bestRow = NO_COLUMN;
  out->bestCol = NO_COLUMN;
  out->emergency = false;
  out->tooNear = 0;
  out->upperInvalid = 0;
  for (int row = 0; row < FLOOR_ROW_FIRST; row++) {
    for (int col = 0; col < COLS; col++) {
      int z = row * COLS + col;
      int d = frame.distanceMm[z];
      bool valid = IsValidStatus(frame.status[z]);
      bool tooNear = IsTooNear(d);
      if (tooNear && !valid) {
        out->tooNear++;
      }
      if (d <= 0 || (!valid && !tooNear)) {
        out->upperInvalid++;
        continue;   // 測れず：空きにも近いにも数えない
      }
      float x, y, h;
      TofView_PointForPan(row, col, d, pan, &x, &y, &h);
      if (!InCorridor(y, h)) {
        continue;
      }
      if (x < out->bestX) {
        out->bestX = x;
        out->bestRow = row;
        out->bestCol = col;
      }
      if (x < TOF_EMERGENCY_MM) {
        out->emergency = true;
      }
    }
  }
}

// 列（firstCol から cols 列）の上の段で、通路の中かどうかによらず最も近い点。maxH・maxSide より外のマスは見ない
static EdgeCell NearestCell(int firstCol, int cols, float maxH, float maxSide) {
  int pan = Servo_GetAngle(SERVO_PAN);
  EdgeCell best = {-1, -1, -1, false};
  float bestX = 1e9f;
  for (int row = 0; row < FLOOR_ROW_FIRST; row++) {
    for (int col = firstCol; col < firstCol + cols; col++) {
      int z = row * COLS + col;
      int d = frame.distanceMm[z];
      if (!TofView_IsUsable(frame.status[z], d)) {
        continue;
      }
      float x, y, h;
      TofView_PointForPan(row, col, d, pan, &x, &y, &h);
      if (h >= maxH || fabsf(y) > maxSide) {
        continue;
      }
      if (x < bestX) {
        bestX = x;
        best = EdgeCell{(int16_t)x, (int8_t)row, (int8_t)col, InCorridor(y, h)};
      }
    }
  }
  return best;
}

// この1枚の端の列を、直近の枚の記録へ足す
static void RecordEdges(bool movingForward) {
  const float NO_LIMIT = 1e9f;
  PreFrame &p = preRing[preHead];
  p.readyMs = frame.readyMs;
  p.left = NearestCell(0, TOF_EDGE_COLS, NO_LIMIT, NO_LIMIT);
  p.right = NearestCell(COLS - TOF_EDGE_COLS, TOF_EDGE_COLS, NO_LIMIT, NO_LIMIT);
  p.centerMm = NearestCell(3, 2, TOF_HEAD_CLEAR_MM, NO_LIMIT).mm;
  p.moving = movingForward;
  // ID27 に渡す分
  edgeView.readyMs = frame.readyMs;
  edgeView.anyMm[TOF_EDGE_LEFT] = p.left.mm;
  edgeView.anyMm[TOF_EDGE_RIGHT] = p.right.mm;
  EdgeCell nearLeft = NearestCell(0, TOF_EDGE_COLS, TOF_HEAD_CLEAR_MM, STEER_SIDE_MAX_MM);
  EdgeCell nearRight = NearestCell(COLS - TOF_EDGE_COLS, TOF_EDGE_COLS, TOF_HEAD_CLEAR_MM, STEER_SIDE_MAX_MM);
  edgeView.nearMm[TOF_EDGE_LEFT] = nearLeft.mm;
  edgeView.nearMm[TOF_EDGE_RIGHT] = nearRight.mm;
  edgeView.nearCol[TOF_EDGE_LEFT] = nearLeft.col;
  edgeView.nearCol[TOF_EDGE_RIGHT] = nearRight.col;
  edgeView.centerMm = p.centerMm;
  edgeViewSeq++;
  preHead = (preHead + 1) % TOF_MISMATCH_PRE_FRAMES;
  if (preCount < TOF_MISMATCH_PRE_FRAMES) {
    preCount++;
  }
}

// 食い違いの記録を作り始める
static void MismatchBegin(Mismatch *m, MismatchKind kind, unsigned long nowMs) {
  const SensorData &s = Sensors_Get();
  CorridorScan scan;
  ScanCorridor(&scan);
  memset(m, 0, sizeof(*m));   // 大きいのでスタックに一時領域を作らない
  m->kind = (uint8_t)kind;
  m->timeMs = nowMs;
  m->speed = Motion_GetSpeed();
  m->usStartCm = s.distanceRawCm;
  m->usMinCm = s.distanceRawCm;
  bool found = scan.bestCol != NO_COLUMN;
  m->nearestMm = found ? (int16_t)scan.bestX : -1;
  m->row = found ? (int8_t)scan.bestRow : -1;
  m->col = found ? (int8_t)scan.bestCol : -1;
  m->upperInvalid = (uint8_t)scan.upperInvalid;
  m->frameAgeMs = (uint16_t)max((long)(nowMs - frame.readyMs), 0L);
  m->snapshot = frame;
  // 「あり」になる前の TOF_MISMATCH_PRE_MS の枚を、古い順にコピーする
  for (int i = 0; i < preCount; i++) {
    const PreFrame &p = preRing[(preHead - preCount + i + TOF_MISMATCH_PRE_FRAMES) % TOF_MISMATCH_PRE_FRAMES];
    long ago = (long)(nowMs - p.readyMs);
    if (ago > TOF_MISMATCH_PRE_MS) {
      continue;
    }
    m->pre[m->preCount++] = PreCell{(int16_t)max(ago, 0L), p.left, p.right, p.centerMm, p.moving};
  }
}

static void MismatchCommit(const Mismatch &m) {
  mismatchRing[mismatchTotal % TOF_MISMATCH_RECORDS] = m;
  mismatchTotal++;
}

// 待っている間の超音波の最小を更新する
static void MismatchWatchUs(Mismatch *m) {
  float raw = Sensors_Get().distanceRawCm;
  if (raw >= 0.0f && (m->usMinCm < 0.0f || raw < m->usMinCm)) {
    m->usMinCm = raw;
  }
}

// 判定に使った1枚を、作りかけの記録に足す（歩いた分だけ近づくかを見るため）
static void MismatchAddFollow(Mismatch *m, const CorridorScan &scan, unsigned long nowMs) {
  m->judged++;
  if (m->followCount >= TOF_MISMATCH_FOLLOW) {
    return;
  }
  bool found = scan.bestCol != NO_COLUMN;
  MismatchFollow &f = m->follow[m->followCount++];
  f.mm = found ? (int16_t)scan.bestX : -1;
  f.row = found ? (int8_t)scan.bestRow : -1;
  f.col = found ? (int8_t)scan.bestCol : -1;
  f.dtMs = (uint16_t)(nowMs - m->timeMs);
  f.speed = Motion_GetSpeed();
}

// この1枚の前端の隙間と横の余裕を、直近3枚の記録へ足す
static void RecordGap(void) {
  int pan = Servo_GetAngle(SERVO_PAN);
  GapCell best;
  best.gapMm = NO_GAP;
  int side = NO_GAP;
  for (int row = 0; row < FLOOR_ROW_FIRST; row++) {
    for (int col = 0; col < COLS; col++) {
      int z = row * COLS + col;
      int d = frame.distanceMm[z];
      if (!TofView_IsUsable(frame.status[z], d) || TofLook_IsIsolated(frame, row, col)) {
        continue;
      }
      float x, y, h;
      TofView_PointForPan(row, col, d, pan, &x, &y, &h);
      if (h >= TOF_HEAD_CLEAR_MM || fabsf(y) >= TOF_BODY_HALF_WIDTH_MM + TOF_CORRIDOR_MARGIN_MM) {
        continue;
      }
      int gap = (int)x - TOF_FRONT_OFFSET_MM;
      if (fabsf(y) > TOF_BODY_HALF_WIDTH_MM) {
        if (side == NO_GAP || gap < side) side = max(gap, 0);
        continue;
      }
      if (best.gapMm == NO_GAP || gap < best.gapMm) {
        best = GapCell{max(gap, 0), row, col, d, frame.status[z], x, y, h};
      }
    }
  }
  memmove(&gapHistory[1], &gapHistory[0], sizeof(GapCell) * (GAP_FRAMES - 1));
  memmove(&sideHistory[1], &sideHistory[0], sizeof(int) * (GAP_FRAMES - 1));
  gapHistory[0] = best;
  sideHistory[0] = side;
  if (gapHistoryCount < GAP_FRAMES) {
    gapHistoryCount++;
  }
}

void TofView_EvaluateFront(int nearMm, TofViewResult *out) {
  TofLook_Evaluate(frame, Servo_GetAngle(SERVO_PAN), 0.0f, nearMm, out);
}

int TofView_ReportStop(void) {
  if (gapHistoryCount < GAP_FRAMES) {
    return -1;
  }
  // 3枚とも出ている近さだけを採る（1枚だけの値で小さくしない）。1枚でもなければ「隙間なし」とする
  int gap = gapHistory[0].gapMm;
  int side = sideHistory[0];
  for (int i = 1; i < GAP_FRAMES; i++) {
    gap = (gap == NO_GAP || gapHistory[i].gapMm == NO_GAP) ? NO_GAP : max(gap, gapHistory[i].gapMm);
    side = (side == NO_GAP || sideHistory[i] == NO_GAP) ? NO_GAP : max(side, sideHistory[i]);
  }
  Log_Printf("ToF", "停止：前端の隙間 %dmm（負＝通路の中になし）／横の余裕 %dmm", gap, side);
  TestStats_RecordStopGap(gap, side);
  if (gap != NO_GAP && (!stopMinValid || gap < stopMin.gapMm)) {
    stopMinValid = true;
    stopMin = gapHistory[0];
    stopMin.gapMm = gap;
    stopMinFrame = frame;
  }
  if (side != NO_GAP && (stopSideMinMm == NO_GAP || side < stopSideMinMm)) {
    stopSideMinMm = side;
  }
  return gap == NO_GAP ? -1 : gap;
}

bool TofView_LastForwardFrame(int *mm, unsigned long *readyMs, uint16_t *skipped, uint16_t *still) {
  *mm = lastForwardMm;
  *readyMs = lastForwardReadyMs;
  *skipped = skippedSinceForward;
  *still = stillSinceForward;
  return lastForwardValid;
}

// 影の「あり」「超音波のあり」の対応を数える。先に立ったほうが相手を待つ
static void RecordShadowOnset(unsigned long nowMs) {
  shadowOnsets++;
  lastShadowOnsetMs = nowMs;
  if (usOnsetWaiting && nowMs - lastUsOnsetMs <= TOF_US_MATCH_MS) {
    usOnsetWaiting = false;
    usRecordActive = false;   // 一致したので残さない
    usOnsetsMatched++;
    shadowOnsetsMatched++;
    leadSumMs -= (long)(nowMs - lastUsOnsetMs);
    leadCount++;
    return;
  }
  if (shadowOnsetWaiting) {
    MismatchCommit(pendingShadow);   // 前の回は相手が来ないまま次が来た
  }
  MismatchBegin(&pendingShadow, MISMATCH_TOF_ONLY, nowMs);
  shadowOnsetWaiting = true;
}

static void RecordUsOnset(unsigned long nowMs) {
  usOnsets++;
  lastUsOnsetMs = nowMs;
  if (usRecordActive) {
    MismatchCommit(pendingUs);   // 前の回は相手が来ないまま次が来た
    usRecordActive = false;
  }
  usOnsetWaiting = false;
  if (shadowOnsetWaiting && nowMs - lastShadowOnsetMs <= TOF_US_MATCH_MS) {
    shadowOnsetWaiting = false;
    shadowOnsetsMatched++;
    usOnsetsMatched++;
    leadSumMs += (long)(nowMs - lastShadowOnsetMs);
    leadCount++;
    return;
  }
  if (judgedBlocked) {
    usOnsetsMatched++;   // 影はもっと前から「あり」のまま（先の時間は数えない）
    if (wasTofStarted) {
      MismatchBegin(&pendingUs, MISMATCH_US_OLD, nowMs);
      usRecordActive = true;
    }
    return;
  }
  if (wasTofStarted) {
    MismatchBegin(&pendingUs, MISMATCH_US_ONLY, nowMs);
    usRecordActive = true;
  }
  usOnsetWaiting = true;
}

// 1枚を判定する。首が水平で、車体と首が1枚の間ずっと動いていないときだけ呼ぶ
// （走行中の直進は「動いていない」に入れる。回転と首の動きだけを捨てる）
static void JudgeFrame(bool movingForward, unsigned long nowMs) {
  framesJudged++;
  // 床の分布は、試験中で床に置いている間だけ（台の上や手で持った時間を入れない）。後退中は傾きが違うので入れない
  if ((movingForward || Motion_IsStill()) && TestStats_IsRecording() && !Safety_IsLifted()) {
    RecordFloor(movingForward);
  }

  RecordGap();
  RecordEdges(movingForward);

  CorridorScan scan;
  ScanCorridor(&scan);
  float bestX = scan.bestX;
  int bestCol = scan.bestCol;
  emergencyNow = scan.emergency;
  tooNearZones += scan.tooNear;
  if (scan.emergency) {
    bool inTest = TestStats_IsRecording() && !Safety_IsLifted();
    if (movingForward) {
      emergencyFramesForward++;
      if (inTest) emergencyTestFramesForward++;
    } else {
      emergencyFrames++;
      if (inTest) emergencyTestFrames++;
    }
  }
  if (shadowOnsetWaiting) {
    MismatchAddFollow(&pendingShadow, scan, nowMs);
  }
  if (usRecordActive) {
    MismatchAddFollow(&pendingUs, scan, nowMs);
  }
  nearestMm = (bestCol == NO_COLUMN) ? -1 : (int)bestX;
  if (nearestMm >= 0) {
    lastMm = nearestMm;
  }
  if (movingForward) {
    lastForwardValid = true;
    lastForwardMm = nearestMm;
    lastForwardReadyMs = frame.readyMs;
    skippedSinceForward = 0;
    stillSinceForward = 0;
  } else if (stillSinceForward < 0xFFFF) {
    stillSinceForward++;
  }

  // 「あり」：通路の中の障害物が2枚続く（2枚の間で列のずれは ±1 まで）
  bool near = bestCol != NO_COLUMN && bestX < TOF_STOP_MM;
  if (near && (nearRun == 0 || abs(bestCol - nearRunColumn) <= 1)) {
    nearRun++;
  } else {
    nearRun = near ? 1 : 0;
  }
  nearRunColumn = near ? bestCol : NO_COLUMN;

  // 前進中に、上の段が測れず床も写らない枚が続いたら止まる（止まっているときの扱いは ID25・ID15 が決める）
  TofViewResult view;
  TofLook_Evaluate(frame, Servo_GetAngle(SERVO_PAN), 0.0f, TOF_STOP_MM, &view);
  blindRun = (movingForward && view.state == TOF_VIEW_BLIND) ? blindRun + 1 : 0;

  // 「なし」：通路に「停止距離＋余裕」より近いものがない枚が続く
  bool clear = bestCol == NO_COLUMN || bestX >= TOF_STOP_MM + TOF_CLEAR_MARGIN_MM;
  clearRun = clear ? clearRun + 1 : 0;

  bool before = judgedBlocked;
  if (!judgeKnown) {
    if (++unknownFrames < TOF_NEAR_FRAMES) {
      return;
    }
    // 不明から決めるときは、「なし」と言い切れなければ「あり」にする（38〜48cm の間で歩き出して、すぐ止まるのを防ぐ）
    judgeKnown = true;
    judgedBlocked = clearRun < TOF_NEAR_FRAMES;
  } else if (!judgedBlocked && (nearRun >= TOF_NEAR_FRAMES || blindRun >= TOF_BLIND_STOP_FRAMES)) {
    judgedBlocked = true;
    if (blindRun >= TOF_BLIND_STOP_FRAMES) {
      TestStats_RecordBlindStop();
      Log_Printf("ToF", "前が見えていません（上の段が測れず、床も写りません）。止まります");
    }
  } else if (judgedBlocked && clearRun >= TOF_NEAR_FRAMES) {
    judgedBlocked = false;
  }
  if (judgedBlocked != before) {
    Log_Printf("ToF", "判定：%s（通路の中の最も近い距離 %dmm。超音波は %s）",
               judgedBlocked ? "あり" : "なし", nearestMm, Obstacle_UsIsBlocked() ? "あり" : "なし");
    // 超音波が先に止めた回は、速度が 0 でも対応を取る（止めたあとの滑走中に影が「あり」になる）
    if (judgedBlocked && (movingForward || usOnsetWaiting)) {
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
  Log_Printf("ToF", "段1：止まる判断に使います（見回しは超音波のまま。床のマスでは止めません）");
}

void TofView_GetCounts(TofCounts *out) {
  TofStats s;
  Tof_GetStats(&s);
  out->state = s.state;
  out->frames = s.frames;
  out->readyFail = s.readyFail;
  out->dataFail = s.dataFail;
  out->staleEvents = staleEvents;
  out->staleMaxMs = staleMaxMs;
  out->periodMaxMs = s.periodMaxMs;
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
      Log_Printf("ToF", "起動：is_alive %d回目で成功、init %lums、8×8 %dHz、スタックの残り %lu バイト%s",
                 s.aliveTries, (unsigned long)s.initMs, TOF_FREQ_HZ, (unsigned long)s.stackFreeAtStart,
                 s.stackFreeAtStart < TOF_STACK_MIN_FREE ? "【少ない】" : "");
      wasTofStarted = true;
      break;
    case TOF_STATE_FAILED:
      Log_Printf("ToF", "起動に失敗：%s=%u（is_alive %d回 alive=%u SDA=%d SCL=%d）。この起動中は走りません",
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
  bool usBlocked = Obstacle_UsIsBlocked();
  // ToF が先に止めた回は、超音波の「あり」が速度 0 で立つので、ToF を待たせている間は速度によらず数える
  if (usBlocked && !usBlockedBefore && (Motion_GetSpeed() >= MOTOR_SPEED_EPSILON || shadowOnsetWaiting)) {
    RecordUsOnset(nowMs);
  }
  usBlockedBefore = usBlocked;
}

// 相手が時間内に来なかった「待ち」を片付ける
static void ExpireWaiting(unsigned long nowMs) {
  if (shadowOnsetWaiting && nowMs - lastShadowOnsetMs > TOF_US_MATCH_MS) {
    shadowOnsetWaiting = false;
    MismatchCommit(pendingShadow);
  }
  if (usOnsetWaiting && nowMs - lastUsOnsetMs > TOF_US_MATCH_MS) {
    usOnsetWaiting = false;
  }
  if (usRecordActive && nowMs - lastUsOnsetMs > TOF_US_MATCH_MS) {
    usRecordActive = false;
    MismatchCommit(pendingUs);
  }
  if (shadowOnsetWaiting) {
    MismatchWatchUs(&pendingShadow);
  }
  if (usRecordActive) {
    MismatchWatchUs(&pendingUs);
  }
}

void TofView_Update(unsigned long nowMs) {
  ReportStart();
  WatchUltrasonic(nowMs);
  ExpireWaiting(nowMs);

  // 回転中と、首が動いている（または水平でない）間は「動いていた」とする
  bool neckOk = Neck_IsSteady(nowMs) && Servo_GetAngle(SERVO_TILT) == SERVO2_LEVEL_DEG &&
                abs(Servo_GetAngle(SERVO_PAN) - SERVO1_FRONT_DEG) <= NECK_DRIVE_TRIM_RIGHT_DEG;
  // 回転を止めたあとの惰性も「動いている」に入れる（前進中の揺れでは捨てない）
  bool coasting = Motion_GetSpeed() < MOTOR_SPEED_EPSILON && fabsf(Gyro_YawRateDps()) > TOF_SETTLE_YAW_DPS;
  if (Motion_IsTurning() || !neckOk || coasting) {
    lastUnstableMs = nowMs;
  }
  bool neckStill = Neck_IsSteady(nowMs) && Servo_GetAngle(SERVO_TILT) == SERVO2_LEVEL_DEG;
  if (Motion_IsTurning() || !neckStill || coasting || !Motion_IsStill()) {
    lastMoveMs = nowMs;
  }

  if (!wasTofStarted) {
    return;
  }

  // 途切れの見張り（止めるのは安全層。TofView_CanDrive を見る）
  if (haveFrame) {
    uint32_t age = (uint32_t)max((long)(nowMs - frame.readyMs), 0L);
    if (age > TOF_STALE_MS) {
      if (!inStale) {
        inStale = true;
        staleEvents++;
        frameLost = true;
        recoverRun = 0;
        Log_Printf("ToF", "データが途切れました（最新の1枚 %lums 前）。すべての動きを止めます", (unsigned long)age);
      }
      if (age > staleMaxMs) staleMaxMs = age;
    } else {
      inStale = false;
    }
  }

  if (holding) {
    if ((long)(nowMs - holdUntilMs) < 0) {
      return;   // キー b：1枚を届けない
    }
    holding = false;
  }
  if (!Tof_GetLatest(frame.seq, &frame)) {
    return;
  }
  haveFrame = true;

  // 途切れから戻るのは、続けて届いてから。init はやり直さない
  if (frameLost) {
    if (++recoverRun < TOF_RECOVER_FRAMES) {
      return;
    }
    frameLost = false;
    TofView_Reset();
    Log_Printf("ToF", "データが戻りました（%d枚続けて届いた）", TOF_RECOVER_FRAMES);
    return;
  }

  // 1枚の間に動きが終わっていない枚は捨てる（読み終えた時刻 − 1枚の時間 が、動きの終わりより後の枚だけを使う）
  if ((long)(frame.readyMs - TOF_FRAME_MS - lastUnstableMs) <= 0) {
    framesSkipped++;
    if (skippedSinceForward < 0xFFFF) {
      skippedSinceForward++;
    }
    if (shadowOnsetWaiting) {
      pendingShadow.skipped++;
    }
    if (usRecordActive) {
      pendingUs.skipped++;
    }
    return;
  }
  JudgeFrame(Motion_GetSpeed() >= MOTOR_SPEED_EPSILON, nowMs);
}

void TofView_Reset(void) {
  lastUnstableMs = millis();
  judgeKnown = false;
  judgedBlocked = false;
  emergencyNow = false;
  unknownFrames = 0;
  blindRun = 0;
  nearRun = 0;
  nearRunColumn = NO_COLUMN;
  clearRun = 0;
}

bool TofView_IsKnown(void) {
  return judgeKnown;
}

bool TofView_IsBlocked(void) {
  return judgeKnown && judgedBlocked;
}

bool TofView_IsEmergency(void) {
  return emergencyNow;
}

float TofView_LastCm(void) {
  return lastMm >= 0 ? lastMm / 10.0f : -1.0f;
}

bool TofView_CanDrive(void) {
#if TOF_ENABLE
  return wasTofStarted && haveFrame && !frameLost;
#else
  return true;
#endif
}

const char *TofView_StopReason(void) {
  TofStats s;
  Tof_GetStats(&s);
  if (s.state == TOF_STATE_FAILED) {
    return "ToF の起動に失敗。この起動中は走りません（電池と USB を入れ直してください）";
  }
  if (!wasTofStarted || !haveFrame) {
    return "ToF の起動を待っています";
  }
  return "ToF のデータが途切れています";
}

void TofView_HoldFrames(unsigned long nowMs) {
#if TOF_ENABLE
  holding = true;
  holdUntilMs = nowMs + TOF_DEBUG_HOLD_MS;
  Log_Printf("ToF", "%dms の間、1枚を届けません（途切れで止まる試験）", TOF_DEBUG_HOLD_MS);
#else
  (void)nowMs;
  Log_Printf("ToF", "このビルドでは使いません（TOF_ENABLE=0）");
#endif
}

uint32_t TofView_JudgedCount(void) {
  return framesJudged;
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
  if (!judgeKnown) {
    snprintf(buffer, size, "不明(%ldms前)", ageMs);
    return;
  }
  snprintf(buffer, size, "%s(%dmm,%ldms前)", judgedBlocked ? "あり" : "なし", nearestMm, ageMs);
}

static void PrintFloor(void) {
  static const char *const KIND_NAMES[2] = {"静止", "走行"};
  Log_Printf("ToF", "床のマス（試験中で床に置いている間）：値 ÷ 想定距離 の分布（%.2f から %.2f 刻み。外は端に入れる。マスの数）",
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

static const char *MismatchKindName(uint8_t kind) {
  switch (kind) {
    case MISMATCH_TOF_ONLY: return "ToFだけ";
    case MISMATCH_US_ONLY:  return "超音波だけ";
    default:                return "超音波（ToF は前から「あり」）";
  }
}

// 記録を新しい順に数えたときの i 番目。なければ NULL
static const Mismatch *MismatchAt(uint32_t i) {
  uint32_t kept = min(mismatchTotal, (uint32_t)TOF_MISMATCH_RECORDS);
  if (i >= kept) {
    return NULL;
  }
  return &mismatchRing[(mismatchTotal - 1 - i) % TOF_MISMATCH_RECORDS];
}

static void FormatEdge(char *buffer, size_t size, const EdgeCell &e) {
  if (e.mm < 0) {
    snprintf(buffer, size, "写らず");
  } else {
    snprintf(buffer, size, "%dmm(r%d c%d %s)", e.mm, e.row, e.col, e.inside ? "内" : "外");
  }
}

// 「あり」になる前の枚。端の列の上の段（r0〜r4）の最も近い前後距離と、通路の中(内)か外か
static void PrintPre(const Mismatch &m) {
  Log_Printf("ToF", "     あり になる前 %dms の枚（古い順。秒＝「あり」の何秒前。動＝前進中の枚・止＝止まっていた枚。左＝c0〜c1・右＝c6〜c7 の r0〜r4 の最も近い前後距離。内＝通路の中・外＝通路の外。中＝c3〜c4 の高さ %dmm 未満の最も近い前後距離）:",
             TOF_MISMATCH_PRE_MS, TOF_HEAD_CLEAR_MM);
  for (int k = 0; k < m.preCount; k++) {
    char left[32];
    char right[32];
    FormatEdge(left, sizeof(left), m.pre[k].left);
    FormatEdge(right, sizeof(right), m.pre[k].right);
    char center[16];
    if (m.pre[k].centerMm < 0) snprintf(center, sizeof(center), "写らず");
    else snprintf(center, sizeof(center), "%dmm", m.pre[k].centerMm);
    Log_Printf("ToF", "       -%.1fs %s 左 %s 右 %s 中 %s", m.pre[k].agoMs / 1000.0f, m.pre[k].moving ? "動" : "止", left, right, center);
  }
}

static void PrintMismatches(void) {
  Log_Printf("ToF", "食い違いの記録（起動から %lu件。直近 %d件まで。新しい順。c で 8×8 の表）",
             (unsigned long)mismatchTotal, TOF_MISMATCH_RECORDS);
  for (uint32_t i = 0;; i++) {
    const Mismatch *m = MismatchAt(i);
    if (m == NULL) {
      break;
    }
    char us[20];
    char usMin[20];
    if (m->usStartCm < 0.0f) snprintf(us, sizeof(us), "反応なし");
    else snprintf(us, sizeof(us), "%.1fcm", m->usStartCm);
    if (m->usMinCm < 0.0f) snprintf(usMin, sizeof(usMin), "なし");
    else snprintf(usMin, sizeof(usMin), "%.1fcm", m->usMinCm);
    Log_Printf("ToF", "  %lu) %lums %s 速度%.2f 超音波 %s（最小 %s） 通路の最も近い %dmm(r%d c%d) r0-4測れず %u 1枚の古さ %ums 判定 %u枚 捨てた %u枚",
               (unsigned long)(i + 1), (unsigned long)m->timeMs, MismatchKindName(m->kind),
               m->speed, us, usMin, m->nearestMm, m->row, m->col,
               m->upperInvalid, m->frameAgeMs, m->judged, m->skipped);
    char line[192];
    int used = 0;
    for (int k = 0; k < m->followCount && used < (int)sizeof(line); k++) {
      const MismatchFollow &f = m->follow[k];
      used += snprintf(line + used, sizeof(line) - used, " %dmm(r%d c%d)+%ums 速度%.2f／", f.mm, f.row, f.col, f.dtMs, f.speed);
    }
    Log_Printf("ToF", "     あとの枚:%s", m->followCount > 0 ? line : " なし");
    if (m->kind == MISMATCH_TOF_ONLY) {
      PrintPre(*m);
    }
  }
}

static void PrintStoredFrame(const TofFrame &f) {
  for (int row = 0; row < 8; row++) {
    char line[128];
    int used = 0;
    for (int col = 0; col < COLS && used < (int)sizeof(line); col++) {
      int z = row * COLS + col;
      used += snprintf(line + used, sizeof(line) - used, " %4d:%-3u", f.distanceMm[z], (unsigned)f.status[z]);
    }
    Log_Raw("r%d%s", row, line);
  }
}

void TofView_PrintMismatchFrames(void) {
  if (stopMinValid) {
    Log_Printf("ToF", "前端の隙間が最小の停止 %dmm の1枚 #%lu（距離mm:状態）", stopMin.gapMm, (unsigned long)stopMinFrame.seq);
    PrintStoredFrame(stopMinFrame);
  }
  if (mismatchTotal == 0) {
    Log_Printf("ToF", "食い違いの記録はまだありません");
    return;
  }
  for (uint32_t i = 0;; i++) {
    const Mismatch *m = MismatchAt(i);
    if (m == NULL) {
      break;
    }
    Log_Printf("ToF", "%lu) %lums %s の1枚 #%lu（距離mm:状態）", (unsigned long)(i + 1), (unsigned long)m->timeMs,
               MismatchKindName(m->kind), (unsigned long)m->snapshot.seq);
    PrintStoredFrame(m->snapshot);
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
  Log_Printf("ToF", "枚数 %lu 失敗 ready %lu／data %lu 周期 平均 %.1fms／最大 %lums 読み出し 平均 %.1fms／最大 %.1fms 途切れ %lu回（最長 %lums） スタックの残り %lu バイト（起動直後 %lu）%s 確認 %lu回（1枚あたり %.1f回）",
             (unsigned long)s.frames, (unsigned long)s.readyFail, (unsigned long)s.dataFail,
             s.periodCount > 0 ? (float)s.periodSumMs / s.periodCount : 0.0f, (unsigned long)s.periodMaxMs,
             s.frames > 0 ? s.readSumUs / 1000.0f / s.frames : 0.0f, s.readMaxUs / 1000.0f,
             (unsigned long)staleEvents, (unsigned long)staleMaxMs, (unsigned long)s.stackFree,
             (unsigned long)s.stackFreeAtStart, s.stackFree < TOF_STACK_MIN_FREE ? "【少ない】" : "",
             (unsigned long)s.polls, s.frames > 0 ? (float)s.polls / s.frames : 0.0f);
  Log_Printf("ToF", "超音波との比較（起動から）：ToF の「あり」%lu回（超音波も ±%dms で「あり」%lu回）／超音波の「あり」%lu回（ToF も %lu回）／両方の回で ToF が先の平均 %ldms",
             (unsigned long)shadowOnsets, TOF_US_MATCH_MS, (unsigned long)shadowOnsetsMatched,
             (unsigned long)usOnsets, (unsigned long)usOnsetsMatched,
             leadCount > 0 ? leadSumMs / (long)leadCount : 0L);
  Log_Printf("ToF", "判定に使った %lu 枚／動きをまたいで捨てた %lu 枚 非常停止の距離より近い枚 前進中 %lu／それ以外 %lu（うち試験中で床に置いている間 前進中 %lu／それ以外 %lu） 状態によらず「近すぎ」のマス %lu いま：%s",
             (unsigned long)framesJudged, (unsigned long)framesSkipped, (unsigned long)emergencyFramesForward,
             (unsigned long)emergencyFrames, (unsigned long)emergencyTestFramesForward, (unsigned long)emergencyTestFrames,
             (unsigned long)tooNearZones, judgedBlocked ? "あり" : "なし");
  PrintFloor();
  if (stopMinValid) {
    Log_Printf("ToF", "前端の隙間の最小 %dmm（r%d c%d %dmm:%u、前後 %.0f 左右 %.0f 高さ %.0f）／横の余裕の最小 %dmm。c で 8×8 の表",
               stopMin.gapMm, stopMin.row, stopMin.col, stopMin.distanceMm, stopMin.status, stopMin.x, stopMin.y, stopMin.z, stopSideMinMm);
  } else {
    Log_Printf("ToF", "前端の隙間：通路の中に3枚とも写った停止はありません／横の余裕の最小 %dmm", stopSideMinMm);
  }
  PrintMismatches();
}

void TofView_PrintFrame(void) {
  if (!haveFrame) {
    Log_Printf("ToF", "まだ1枚も届いていません");
    return;
  }
  Log_Printf("ToF", "最新の1枚 #%lu（行＝上下 r0 上、列＝左右 c0 左。距離mm:状態。*＝通路の中で %dmm より近い）",
             (unsigned long)frame.seq, TOF_STOP_MM);
  int pan = Servo_GetAngle(SERVO_PAN);
  for (int row = 0; row < 8; row++) {
    char line[128];
    int used = 0;
    for (int col = 0; col < COLS && used < (int)sizeof(line); col++) {
      int z = row * COLS + col;
      int d = frame.distanceMm[z];
      float x, y, h;
      TofView_PointForPan(row, col, d, pan, &x, &y, &h);
      bool usable = d > 0 && (IsValidStatus(frame.status[z]) || IsTooNear(d));
      bool mark = row < FLOOR_ROW_FIRST && usable && InCorridor(y, h) && x < TOF_STOP_MM;
      used += snprintf(line + used, sizeof(line) - used, " %4d:%-3u%s", d, (unsigned)frame.status[z], mark ? "*" : " ");
    }
    Log_Raw("r%d%s", row, line);
  }
}
