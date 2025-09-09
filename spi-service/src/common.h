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

// 把任意 byte 序列轉成十六進位字串 (固定大寫 + 空白分隔)
inline std::string bytesToHexString(const uint8_t* data, size_t len,  size_t maxLen = SIZE_MAX) {
    if (len == 0) return {};

    len = std::min(len, maxLen);
    static const char* hex = "0123456789ABCDEF";

    // 每個 byte -> 兩個 hex 字元，加上中間空格 (len-1 個)
    std::string hexStr;
    hexStr.resize(len * 3 - 1);

    char* out = hexStr.data();
    for (size_t i = 0; i < len; i++) {
        *out++ = hex[data[i] >> 4];
        *out++ = hex[data[i] & 0x0F];
        if (i != len - 1) {
            *out++ = ' ';
        }
    }

    return hexStr;
}

// 泛型整數 -> Hex 字串 (Little Endian)
template <typename T,
          typename = std::enable_if_t<std::is_integral<T>::value>>
inline std::string intToHexStringLE(T value, size_t maxLen = SIZE_MAX) {
    uint8_t bytes[sizeof(T)];
    for (size_t i = 0; i < sizeof(T); i++) {
        bytes[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);  // 小端序
    }
    return bytesToHexString(bytes, sizeof(T), maxLen);
}

// buffer -> Hex 字串
inline std::string bufferToHexString(const char* buf, size_t len, size_t maxLen = SIZE_MAX) {
    return bytesToHexString(reinterpret_cast<const uint8_t*>(buf), len, maxLen);
}

inline std::string bytesToHexString(const std::vector<uint8_t>& buf, size_t maxLen = SIZE_MAX) {
    return bytesToHexString(buf.data(), buf.size(), maxLen);
}

#if 0
inline std::string bytesToHexString(const uint8_t* buf, size_t len, size_t maxLen = SIZE_MAX) {
    if (len == 0) return {};

    len = std::min(len, maxLen);
    static const char* hex = "0123456789ABCDEF";

    // 每個 byte -> 兩個 hex 字元，加上中間空格 (len-1 個)
    std::string hexStr;
    hexStr.resize(len * 3 - 1);

    char* out = hexStr.data();
    for (size_t i = 0; i < len; i++) {
        *out++ = hex[buf[i] >> 4];
        *out++ = hex[buf[i] & 0x0F];
        if (i != len - 1) {
            *out++ = ' ';
        }
    }

    return hexStr;
}

inline std::string bytesToHexString(const std::vector<uint8_t>& buf, size_t maxLen = SIZE_MAX) {
    return bytesToHexString(buf.data(), buf.size(), maxLen);
}

inline std::string uint16ToHexStringLE(uint16_t value) {
    std::ostringstream oss;
    // Little-endian: LSB first
    oss << std::hex << std::setw(2) << std::setfill('0') << (value & 0xFF)
        << ' '
        << std::setw(2) << std::setfill('0') << ((value >> 8) & 0xFF);
    return oss.str();
}
#endif
inline uint8_t getByteAt(const std::vector<uint8_t>& data, size_t index) {
    if (index >= data.size()) {
        throw std::out_of_range("Index out of range in getByteAt()");
    }
    return data[index];
}
