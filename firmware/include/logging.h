#pragma once

using LogMirrorCallback = void (*)(const char* line);

void setLogMirrorCallback(LogMirrorCallback callback);
void logLine(const char* line);
void logf(const char* fmt, ...);
