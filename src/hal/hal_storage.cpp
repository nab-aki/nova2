#include "hal_storage.h"

#include <Preferences.h>

// NVS の名前空間とキー。用途を増やすときは名前空間を分ける
#define STORAGE_NAMESPACE  "nova"
#define STORAGE_KEY        "teststats"

static Preferences prefs;

void Storage_Setup(void) {
  // begin/end は呼び出しごとに行う（Load/Save/Clear の中）ので、ここでは何もしない
}

bool Storage_Load(void *buf, size_t len) {
  prefs.begin(STORAGE_NAMESPACE, true);   // 読み取り専用
  size_t got = prefs.getBytesLength(STORAGE_KEY);
  bool ok = (got == len);
  if (ok) {
    prefs.getBytes(STORAGE_KEY, buf, len);
  }
  prefs.end();
  return ok;
}

void Storage_Save(const void *buf, size_t len) {
  prefs.begin(STORAGE_NAMESPACE, false);
  prefs.putBytes(STORAGE_KEY, buf, len);
  prefs.end();
}

void Storage_Clear(void) {
  prefs.begin(STORAGE_NAMESPACE, false);
  prefs.remove(STORAGE_KEY);
  prefs.end();
}
