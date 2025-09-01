#pragma once

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cerrno>
#include <string>
#include <vector>

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

inline std::string bytesToHexString(const uint8_t* buf, size_t len) {
    if (len == 0) return {};

    static const char* hex = "0123456789ABCDEF";

    // 每個 byte -> 兩個 hex 字元，加上中間空格 (len-1 個)
    std::string hexStr;
    hexStr.resize(len * 3 - 1);

    char* out = hexStr.data();
    for (size_t i = 0; i < len; i++) {
        *out++ = hex[buf[i] >> 4];      // 高 4 bits
        *out++ = hex[buf[i] & 0x0F];    // 低 4 bits
        if (i != len - 1) {
            *out++ = ' ';
        }
    }

    return hexStr;
}

inline std::string bytesToHexString(const std::vector<uint8_t>& buf) {
    return bytesToHexString(buf.data(), buf.size());
}
