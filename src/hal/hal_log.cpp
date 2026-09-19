#include "hal_log.h"

#include <stdarg.h>

#include "../config.h"

void Log_Setup(void) {
  Serial.begin(SERIAL_BAUD);
}

void Log_Printf(const char *tag, const char *fmt, ...) {
  char message[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(message, sizeof(message), fmt, args);
  va_end(args);
  Serial.printf("[%lu][%s] %s\n", millis(), tag, message);
}
