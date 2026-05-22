#include "logging.h"

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>

namespace {

LogMirrorCallback g_logMirrorCallback = nullptr;

}  // namespace

void setLogMirrorCallback(LogMirrorCallback callback) {
  g_logMirrorCallback = callback;
}

void logLine(const char* line) {
  Serial.println(line);
  if (g_logMirrorCallback != nullptr) {
    g_logMirrorCallback(line);
  }
}

void logf(const char* fmt, ...) {
  char buffer[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  logLine(buffer);
}
