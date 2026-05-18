#include "debug_log.h"

#include <stdarg.h>
#include <stdio.h>

static DebugLogWriter g_writer = 0;

void DebugLog_SetWriter(DebugLogWriter writer) {
    g_writer = writer;
}

void DebugLog_Info(const char* tag, const char* fmt, ...) {
    char body[192];
    char line[240];
    va_list args;

    if (tag == 0) tag = "LOG";
    if (fmt == 0) fmt = "";

    va_start(args, fmt);
    (void)vsnprintf(body, sizeof(body), fmt, args);
    va_end(args);

    (void)snprintf(line, sizeof(line), "[%s] %s\n", tag, body);

    if (g_writer) {
        g_writer(line);
        return;
    }

#ifdef PC_SIM
    fputs(line, stdout);
#else
    /*
     * 实机适配时可在上层通过 DebugLog_SetWriter 注入 UART 发送函数。
     * 默认不触碰任何真实串口，确保核心逻辑没有硬件强依赖。
     */
#endif
}
