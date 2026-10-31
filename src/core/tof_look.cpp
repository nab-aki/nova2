#include "tof_look.h"

#include <math.h>

#include "../config.h"
#include "../hal/hal_servo.h"
#include "gyro.h"
#include "neck.h"
#include "test_stats.h"
#include "tof_view.h"

#define UPPER_ROWS 5   // 上の段（r0〜r4）

const char *TofLook_StateName(TofViewState state) {
  switch (state) {
    case TOF_VIEW_NEAR:      return "近い";
    case TOF_VIEW_CLEAR:     return "空き";
    case TOF_VIEW_FAR_CLEAR: return "遠くて空き";
    default:                 return "見えていない";
  }
}

bool TofLook_IsIsolated(const TofFrame &frame, int row, int col) {
  int d = frame.distanceMm[row * TOF_LOOK_COLUMNS + col];
  int allowed = max(TOF_ISOLATED_MM, (int)(d * TOF_ISOLATED_RATIO));
  for (int r = max(row - 1, 0); r <= min(row + 1, UPPER_ROWS - 1); r++) {
    for (int c = max(col - 1, 0); c <= min(col + 1, TOF_LOOK_COLUMNS - 1); c++) {
      if (r == row && c == col) {
        continue;
      }
      int z = r * TOF_LOOK_COLUMNS + c;
      if (TofView_IsUsable(frame.status[z], frame.distanceMm[z]) && abs(frame.distanceMm[z] - d) <= allowed) {
        return false;
      }
    }
  }
  return true;
}

// センサーから見た点 (x, y) を、車体を headingDeg 回した先の前後 x・左右 y に直す（回転の中心のまわりに回す）
static void RotateToHeading(float x, float y, float cosH, float sinH, float *outX, float *outY) {
  float fromCenter = x + TOF_CENTER_TO_SENSOR_MM;
  *outX = fromCenter * cosH + y * sinH - TOF_CENTER_TO_SENSOR_MM;
  *outY = -fromCenter * sinH + y * cosH;
}

static inline bool InBodyWidth(float y) {
  return fabsf(y) < (TOF_BODY_HALF_WIDTH_MM + TOF_CORRIDOR_MARGIN_MM);
}

// 通路に入る床のマス（r6・r7）のうち、想定どおりに写っているものが足りるか
static bool FloorLooksRight(const TofFrame &frame, int panDeg, float cosH, float sinH) {
  int total = 0;
  int ok = 0;
  for (int row = TOF_FLOOR_CHECK_ROW; row < 8; row++) {
    float expected = TofView_FloorExpectedMm(row);
    for (int col = 0; col < TOF_LOOK_COLUMNS; col++) {
      float x, y, h, rx, ry;
      TofView_PointForPan(row, col, (int)expected, panDeg, &x, &y, &h);
      RotateToHeading(x, y, cosH, sinH, &rx, &ry);
      if (!InBodyWidth(ry) || rx <= -TOF_CENTER_TO_SENSOR_MM) {
        continue;
      }
      total++;
      int z = row * TOF_LOOK_COLUMNS + col;
      if (TofView_IsValidStatus(frame.status[z])) {
        float ratio = frame.distanceMm[z] / expected;
        if (ratio >= TOF_FLOOR_RATIO && ratio <= TOF_FLOOR_RATIO_FAR) {
          ok++;
        }
      }
    }
  }
  return total >= 2 && ok >= total * TOF_FLOOR_SEEN_RATIO;
}

void TofLook_Evaluate(const TofFrame &frame, int panDeg, float headingDeg, int nearMm, TofViewResult *out) {
  float cosH = cosf(headingDeg * DEG_TO_RAD);
  float sinH = sinf(headingDeg * DEG_TO_RAD);
  float best = 1e9f;
  int valid = 0;
  for (int row = 0; row < UPPER_ROWS; row++) {
    for (int col = 0; col < TOF_LOOK_COLUMNS; col++) {
      int z = row * TOF_LOOK_COLUMNS + col;
      int d = frame.distanceMm[z];
      if (!TofView_IsUsable(frame.status[z], d) || TofLook_IsIsolated(frame, row, col)) {
        continue;
      }
      float x, y, h, rx, ry;
      TofView_PointForPan(row, col, d, panDeg, &x, &y, &h);
      RotateToHeading(x, y, cosH, sinH, &rx, &ry);
      if (h >= TOF_HEAD_CLEAR_MM || !InBodyWidth(ry) || rx <= -TOF_CENTER_TO_SENSOR_MM) {
        continue;
      }
      valid++;
      if (rx < best) {
        best = rx;
      }
    }
  }
  out->valid = valid;
  out->cm = valid > 0 ? best / 10.0f : -1.0f;
  if (valid > 0 && best < nearMm) {
    out->state = TOF_VIEW_NEAR;
  } else if (valid >= TOF_VIEW_MIN_VALID) {
    out->state = TOF_VIEW_CLEAR;
  } else if (FloorLooksRight(frame, panDeg, cosH, sinH)) {
    out->state = TOF_VIEW_FAR_CLEAR;
  } else {
    out->state = TOF_VIEW_BLIND;
  }
}

void TofLooker::begin(int panDeg, float headingDeg, int nearMm) {
  panDeg_ = panDeg;
  headingDeg_ = headingDeg;
  nearMm_ = nearMm;
  granted_ = false;
  neckMoves_ = false;
  onsetSeen_ = false;
  taken_ = 0;
  anyBlind_ = false;
  anyNear_ = false;
  anyClear_ = false;
  done_ = false;
  result_ = TofViewResult{TOF_VIEW_BLIND, -1.0f, 0};
  for (int col = 0; col < TOF_LOOK_COLUMNS; col++) {
    columnCm_[col] = -1.0f;
  }
}

float TofLooker::columnsCm(int first, int last) const {
  float best = -1.0f;
  for (int col = first; col <= last; col++) {
    if (columnCm_[col] >= 0.0f && (best < 0.0f || columnCm_[col] < best)) {
      best = columnCm_[col];
    }
  }
  return best;
}

bool TofLooker::update(unsigned long nowMs) {
  if (done_) {
    return true;
  }
  if (!granted_) {
    bool moves = Servo_GetAngle(SERVO_PAN) != panDeg_;
    if (!Neck_Request(NECK_OWNER_RANGE, panDeg_, SERVO2_LEVEL_DEG, nowMs)) {
      return false;
    }
    granted_ = true;
    neckMoves_ = moves;
    commandMs_ = nowMs;
    return false;
  }
  if (neckMoves_ && !onsetSeen_ && nowMs - commandMs_ <= TOF_LOOK_ONSET_WINDOW_MS &&
      fabsf(Gyro_YawRateDps()) >= TOF_LOOK_ONSET_DPS) {
    onsetSeen_ = true;
  }

  const TofFrame *frame = NULL;
  if (!TofView_TakeStillFrame(&lastSeq_, &frame)) {
    return false;
  }
  TofViewResult one;
  TofLook_Evaluate(*frame, panDeg_, headingDeg_, nearMm_, &one);
  anyBlind_ = anyBlind_ || one.state == TOF_VIEW_BLIND;
  anyNear_ = anyNear_ || one.state == TOF_VIEW_NEAR;
  anyClear_ = anyClear_ || one.state == TOF_VIEW_CLEAR;
  if (one.cm >= 0.0f && (result_.cm < 0.0f || one.cm < result_.cm)) {
    result_.cm = one.cm;
  }
  result_.valid = max(result_.valid, one.valid);
  for (int col = 0; col < TOF_LOOK_COLUMNS; col++) {
    for (int row = 0; row < UPPER_ROWS; row++) {
      int z = row * TOF_LOOK_COLUMNS + col;
      int d = frame->distanceMm[z];
      if (!TofView_IsUsable(frame->status[z], d) || TofLook_IsIsolated(*frame, row, col)) {
        continue;
      }
      if (columnCm_[col] < 0.0f || d / 10.0f < columnCm_[col]) {
        columnCm_[col] = d / 10.0f;
      }
    }
  }
  if (++taken_ < TOF_LOOK_FRAMES) {
    return false;
  }
  result_.state = anyBlind_ ? TOF_VIEW_BLIND
                  : anyNear_ ? TOF_VIEW_NEAR
                  : anyClear_ ? TOF_VIEW_CLEAR : TOF_VIEW_FAR_CLEAR;
  if (neckMoves_) {
    TestStats_RecordNeckLook(onsetSeen_);
  }
  done_ = true;
  return true;
}
