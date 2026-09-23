#include "hal_trace.h"

#include <esp_attr.h>
#include <string.h>

#define TRACE_MAGIC  0x54524331u   // "TRC1"。中身の構造を変えたら値も変える

struct TraceStore {
  uint32_t magic;
  uint8_t head;    // 次に書く位置
  uint8_t count;   // 有効な件数（0〜TRACE_MAX_RECORDS）
  uint16_t pad;
  TraceRecord ring[TRACE_MAX_RECORDS];
};

// RTCメモリ（初期化しない領域）。電源投入直後は中身が不定なので、必ず magic で確かめる
static RTC_NOINIT_ATTR TraceStore store;

// 前回のブートぶんの控え（取り出したあと、RTC側は今回の記録のために空にする）
static TraceRecord prevRing[TRACE_MAX_RECORDS];
static int prevCount = 0;

static void ClearStore(void) {
  store.magic = TRACE_MAGIC;
  store.head = 0;
  store.count = 0;
  store.pad = 0;
}

// UTF-8 の文字の途中で切らずに詰める（切れると文字化けするため）
static void CopyText(char *dst, size_t size, const char *src) {
  if (src == NULL) {
    src = "";
  }
  size_t max = size - 1;
  size_t n = 0;
  size_t boundary = 0;   // 直前に見つかった文字の切れ目
  while (src[n] != '\0' && n < max) {
    n++;
    if (((unsigned char)src[n] & 0xC0) != 0x80) {   // 継続バイトでなければ文字の切れ目
      boundary = n;
    }
  }
  size_t len = (src[n] == '\0') ? n : boundary;
  memcpy(dst, src, len);
  dst[len] = '\0';
}

void RtcTrace_Setup(void) {
  prevCount = 0;
  bool valid = (store.magic == TRACE_MAGIC) &&
               (store.head < TRACE_MAX_RECORDS) &&
               (store.count <= TRACE_MAX_RECORDS);
  if (valid) {
    int count = store.count;
    int start = (store.head + TRACE_MAX_RECORDS - count) % TRACE_MAX_RECORDS;
    for (int i = 0; i < count; i++) {
      prevRing[i] = store.ring[(start + i) % TRACE_MAX_RECORDS];
      // 途中で電源が落ちて壊れていても、表示で暴走しないように終端を強制する
      prevRing[i].behavior[TRACE_TEXT_LEN - 1] = '\0';
      prevRing[i].state[TRACE_TEXT_LEN - 1] = '\0';
    }
    prevCount = count;
  }
  ClearStore();
}

void RtcTrace_Add(const char *behavior, const char *state, unsigned long nowMs) {
  TraceRecord &r = store.ring[store.head];
  CopyText(r.behavior, sizeof(r.behavior), behavior);
  CopyText(r.state, sizeof(r.state), state);
  r.enteredMs = (uint32_t)nowMs;
  r.aliveMs = (uint32_t)nowMs;
  r.speed = 0.0f;
  r.target = 0.0f;
  r.turnPwm = 0;
  r.turnKind = TRACE_TURN_NONE;
  r.reserved = 0;

  store.head = (uint8_t)((store.head + 1) % TRACE_MAX_RECORDS);
  if (store.count < TRACE_MAX_RECORDS) {
    store.count++;
  }
}

void RtcTrace_Motor(float speed, float target, int turnPwm, uint8_t turnKind, unsigned long nowMs) {
  if (store.count == 0) {
    return;
  }
  TraceRecord &r = store.ring[(store.head + TRACE_MAX_RECORDS - 1) % TRACE_MAX_RECORDS];
  r.speed = speed;
  r.target = target;
  r.turnPwm = (int16_t)turnPwm;
  r.turnKind = turnKind;
  r.aliveMs = (uint32_t)nowMs;
}

int RtcTrace_PrevCount(void) {
  return prevCount;
}

const TraceRecord *RtcTrace_Prev(int index) {
  if (index < 0 || index >= prevCount) {
    return NULL;
  }
  return &prevRing[index];
}
