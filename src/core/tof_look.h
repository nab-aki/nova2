// ToF で向きを見る部品（docs/specs/common_tof_scan.md 3.5・4章「段2 の方針」）。
// 1枚を「その向きへ車体を回したら通路が空いているか」に近い形で見て、次の4つに分ける。
//   近い／空き／遠くて空き（上の段は測れないが、床が想定どおり写る）／見えていない（空きにしない）
// TofLooker は首を向けて新しい3枚を見る。3枚のうち最も近い値を採り、1枚でも見えていなければ空きにしない。
// 首が動いたかどうかで測り直しはしない（動き出しの有無を記録するだけ）。
#ifndef NOVA_CORE_TOF_LOOK_H
#define NOVA_CORE_TOF_LOOK_H

#include <Arduino.h>

#include "../hal/hal_tof.h"

#define TOF_LOOK_FRAMES  3
#define TOF_LOOK_COLUMNS 8

enum TofViewState {
  TOF_VIEW_NEAR,        // 通路の中に基準より近い物がある
  TOF_VIEW_CLEAR,       // 通路の中に有効なマスが足りていて、基準より遠い
  TOF_VIEW_FAR_CLEAR,   // 上の段は足りないが、床が想定どおり写る＝遠くまで空き
  TOF_VIEW_BLIND        // 床も写らないかおかしい＝見えていない
};

struct TofViewResult {
  TofViewState state;
  float cm;    // 通路の中の最も近い距離。なければ負
  int valid;   // 通路の中の有効なマスの数
};

const char *TofLook_StateName(TofViewState state);

// 周りに近い値のマスがない孤立したマスか（上の段だけを見る）
bool TofLook_IsIsolated(const TofFrame &frame, int row, int col);

// 首が panDeg を向いているときの1枚を、車体を headingDeg（左が正）回した先の通路で見る
void TofLook_Evaluate(const TofFrame &frame, int panDeg, float headingDeg, int nearMm, TofViewResult *out);

class TofLooker {
 public:
  // 首を panDeg へ向け、車体を headingDeg 回した先の通路を、新しい3枚で見始める
  void begin(int panDeg, float headingDeg, int nearMm);

  // loop から呼ぶ。見終わったら true
  bool update(unsigned long nowMs);

  const TofViewResult &result() const { return result_; }

  // 列の最も近い距離（cm。上の段の有効なマス。孤立したマスを除く）。なければ負
  float columnCm(int col) const { return columnCm_[col]; }

  // first〜last の列の最も近い距離（cm）。なければ負
  float columnsCm(int first, int last) const;

 private:
  int panDeg_ = 0;
  float headingDeg_ = 0.0f;
  int nearMm_ = 0;
  bool granted_ = false;
  bool neckMoves_ = false;     // 今回の指示で首が動くはずか
  bool onsetSeen_ = false;     // 指示の直後にジャイロで動き出しが見えたか
  unsigned long commandMs_ = 0;
  uint32_t lastSeq_ = 0;
  int taken_ = 0;
  bool anyBlind_ = false;
  bool anyNear_ = false;
  bool anyClear_ = false;
  bool done_ = true;
  TofViewResult result_ = {TOF_VIEW_BLIND, -1.0f, 0};
  float columnCm_[TOF_LOOK_COLUMNS] = {};
};

#endif // NOVA_CORE_TOF_LOOK_H
