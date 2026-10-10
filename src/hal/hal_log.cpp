#include "hal_log.h"

#include <stdarg.h>

#include "../config.h"

void Log_Setup(void) {
  Serial.begin(SERIAL_BAUD);
}

static LogStats stats;

const LogStats &Log_GetStats(void) {
  return stats;
}

// 1行を送り、かかった時間を記録する（送信バッファが 0 なので、128 バイトを超えると送り終わるまで待つ）
static void WriteLine(const char *line, size_t length) {
  uint32_t beginUs = micros();
  Serial.write((const uint8_t *)line, length);
  uint32_t elapsedUs = micros() - beginUs;
  stats.count++;
  stats.totalUs += elapsedUs;
  stats.totalBytes += (uint32_t)length;
  if (elapsedUs >= LOG_SLOW_US) {
    stats.slowCount++;
  }
  if (elapsedUs > stats.maxUs) {
    stats.maxUs = elapsedUs;
    stats.maxBytes = (uint32_t)length;
  }
}

void Log_Printf(const char *tag, const char *fmt, ...) {
  // 日本語1文字3バイト。状態行（main.cpp）は 270 バイト前後になるので余裕をみる
  char message[384];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);
  char line[448];
  int length = snprintf(line, sizeof(line), "[%lu][%s] %s\n", millis(), tag, message);
  if (length < 0) {
    return;
  }
  if ((size_t)length >= sizeof(line)) {
    length = sizeof(line) - 1;
    line[length - 1] = '\n';   // 切れても改行は残す
  }
  WriteLine(line, (size_t)length);
}

void Log_Raw(const char *fmt, ...) {
  char message[384];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);
  char line[400];
  int length = snprintf(line, sizeof(line), "%s\r\n", message);   // Serial.println と同じ改行
  if (length < 0) {
    return;
  }
  if ((size_t)length >= sizeof(line)) {
    length = sizeof(line) - 1;
    line[length - 2] = '\r';
    line[length - 1] = '\n';
  }
  WriteLine(line, (size_t)length);
}
