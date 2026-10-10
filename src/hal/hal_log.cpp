#include "hal_log.h"

#include <stdarg.h>

#include "../config.h"

void Log_Setup(void) {
  Serial.begin(SERIAL_BAUD);
}

void Log_Printf(const char *tag, const char *fmt, ...) {
  // 日本語1文字3バイト。状態行（main.cpp）は 270 バイト前後になるので余裕をみる
  char message[384];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);
  Serial.printf("[%lu][%s] %s\n", millis(), tag, message);
}

void Log_Raw(const char *fmt, ...) {
  char message[384];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);
  Serial.println(message);
}
