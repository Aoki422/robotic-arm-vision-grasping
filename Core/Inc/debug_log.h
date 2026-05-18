#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*DebugLogWriter)(const char* text);

void DebugLog_SetWriter(DebugLogWriter writer);
void DebugLog_Info(const char* tag, const char* fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
