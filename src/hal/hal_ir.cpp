#include "hal_ir.h"

#include <IRremoteESP8266.h>
#include <IRrecv.h>

#include "../config.h"
#include "hal_log.h"
#include "hal_pins.h"

// Freenove のリモコンのコード表（公式サンプル 05.2_IR_Receiver_Car・05.3_Multi_Functional_Car）
struct IrButton {
  uint32_t code;
  const char *name;
};

static const IrButton BUTTONS[] = {
    {0xFF02FD, "+"},   {0xFF9867, "-"},   {0xFFE01F, "|<<"}, {0xFF906F, ">>|"},
    {0xFFA857, "▶"},   {0xFF6897, "0"},   {0xFF30CF, "1"},   {0xFF18E7, "2"},
    {0xFF7A85, "3"},   {0xFF10EF, "4"},   {0xFF38C7, "5"},   {0xFF5AA5, "6"},
    {0xFF42BD, "7"},   {0xFF4AB5, "8"},   {0xFF52AD, "9"},   {0xFFB04F, "C"},
    {0xFF22DD, "TEST"}};

static IRrecv *irrecv = NULL;
static uint32_t lastCode = 0;
static unsigned long lastCodeMs = 0;
static unsigned long lastOtherLogMs = 0;

const char *Ir_ButtonName(uint32_t code) {
  for (size_t i = 0; i < sizeof(BUTTONS) / sizeof(BUTTONS[0]); i++) {
    if (BUTTONS[i].code == code) {
      return BUTTONS[i].name;
    }
  }
  return NULL;
}

void Ir_Setup(void) {
  // 受信は GPIO の変化割り込みと、フレームの終わりを見るための単発のハードウェアタイマー（3番）で行う。
  // 超音波（GPIO割り込み）・WS2812（RMT）・ブザー（LEDC）・PCA9685（I2C）とは取り合わない
  irrecv = new IRrecv(PIN_IR_RECV);
  irrecv->enableIRIn();
  Log_Printf("IR", "受信を始めます（GPIO%d、NEC）", PIN_IR_RECV);
}

bool Ir_Poll(uint32_t *code) {
  if (irrecv == NULL) {
    return false;
  }
  decode_results results;
  if (!irrecv->decode(&results)) {
    return false;
  }
  // 結果を取り出したら、すぐ次の受信に移る
  bool isNec = (results.decode_type == NEC && results.bits == 32);
  bool isRepeat = results.repeat || results.value == kRepeat;
  uint32_t value = (uint32_t)results.value;
  int bits = results.bits;
  irrecv->resume();

  unsigned long now = millis();
  if (!isNec) {
    // NEC 以外（別のリモコンや雑音）。数秒に1回だけ知らせる
    if (now - lastOtherLogMs >= 3000) {
      lastOtherLogMs = now;
      Log_Printf("IR", "NEC 以外の信号を受信（%dbit）。無視します", bits);
    }
    return false;
  }
  if (isRepeat) {
    lastCodeMs = now;   // 押しっぱなし。直前のボタンが続いているだけなので、捨てる
    return false;
  }
  if (value == lastCode && now - lastCodeMs < IR_DEBOUNCE_MS) {
    lastCodeMs = now;   // 同じコードが短い間隔で続く間は、押しっぱなしとみなして捨て続ける
    return false;
  }
  lastCode = value;
  lastCodeMs = now;

  const char *name = Ir_ButtonName(value);
  Log_Printf("IR", "0x%lX（%s）", (unsigned long)value, name != NULL ? name : "未登録");
  *code = value;
  return true;
}
