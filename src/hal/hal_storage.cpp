#include "hal_storage.h"

#include <Preferences.h>

// NVS の名前空間。キーは用途ごとに分ける（hal_storage.h の STORAGE_KEY_*）
#define STORAGE_NAMESPACE  "nova"

static Preferences prefs;

void Storage_Setup(void) {
  // begin/end は呼び出しごとに行う（Load/Save/Clear の中）ので、ここでは何もしない
}

bool Storage_Load(const char *key, void *buf, size_t len) {
  prefs.begin(STORAGE_NAMESPACE, true);   // 読み取り専用
  size_t got = prefs.getBytesLength(key);
  bool ok = (got == len);
  if (ok) {
    prefs.getBytes(key, buf, len);
  }
  prefs.end();
  return ok;
}

void Storage_Save(const char *key, const void *buf, size_t len) {
  prefs.begin(STORAGE_NAMESPACE, false);
  prefs.putBytes(key, buf, len);
  prefs.end();
}

void Storage_Clear(const char *key) {
  prefs.begin(STORAGE_NAMESPACE, false);
  prefs.remove(key);
  prefs.end();
}
