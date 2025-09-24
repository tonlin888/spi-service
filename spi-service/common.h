#pragma once

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cerrno>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>

#define LOG_TAG "spi-service"

#define LOGE(fmt, ...) log_print(LogLevel::ERROR, LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGW(fmt, ...) log_print(LogLevel::WARN,  LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGI(fmt, ...) log_print(LogLevel::INFO,  LOG_TAG, fmt, ##__VA_ARGS__)
#define LOGD(fmt, ...) log_print(LogLevel::DEBUG, LOG_TAG, fmt, ##__VA_ARGS__)

enum class LogLevel { ERROR, WARN, INFO, DEBUG };

inline void log_print(LogLevel level, const char* tag, const char* fmt, ...) {
    const char* levelStr = nullptr;
    FILE* out = nullptr;

    switch (level) {
        case LogLevel::ERROR: levelStr = "[E]"; out = stderr; break;
        case LogLevel::WARN:  levelStr = "[W]"; out = stderr; break;
        case LogLevel::INFO:  levelStr = "[I]"; out = stdout; break;
        case LogLevel::DEBUG: levelStr = "[D]"; out = stdout; break;
    }

    va_list args;
    va_start(args, fmt);
    fprintf(out, "[%s]%s ", tag, levelStr);
    vfprintf(out, fmt, args);
    fprintf(out, "\n");
    va_end(args);
}