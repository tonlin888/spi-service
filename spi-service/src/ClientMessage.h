#pragma once
#include "common.h"
#include "SpiCommon.h"
#include <sstream>
#include <iomanip>
#include <string>
#include <vector>
#include <cstdint>
#include <cstddef>

#undef LOG_TAG
#define LOG_TAG "spi-service/ClientMessage"

// Client Message: SEQ_ID(2 bytes) + CMD_ID(1 byte) + ERROR_CODE(1 byte) + DATA
class ClientMessage {
public:
    static constexpr size_t HEADER_SIZE = 4;  // Maximum IPC packet length
    
    uint16_t seq_;              // SEQ_ID (2 bytes, little endian)
    SpiCommon::Command  cmd_;   // CMD_ID (1 byte)
    SpiCommon::ErrorCode  err_;
    std::vector<uint8_t> data_; // DATA (binary-safe) (ex, error code)
    bool valid_{false};

    ClientMessage() : seq_(0), cmd_(SpiCommon::Command::NONE), err_(SpiCommon::ErrorCode::NONE) {}

    // Constructor from buffer
    ClientMessage(const uint8_t* buf, size_t len) {
        if (!parse(buf, len)) {
            valid_ = false;
            LOGE("Invalid ClientMessage buffer");
        } else {
            valid_ = true;
        }
    }
    
    // Constructor for error or manual build
    ClientMessage(uint16_t seq_id, SpiCommon::Command cmd_id, SpiCommon::ErrorCode err, std::vector<uint8_t> data = {})
        : seq_(seq_id), cmd_(cmd_id), err_(err), data_(data), valid_(true) {}
    

    bool parse(const uint8_t* buf, size_t len) {
        if (len < HEADER_SIZE) {
            LOGE("parse, len < HEADER_SIZE %d", HEADER_SIZE);
            return false;
        }
        seq_ = static_cast<uint16_t>(buf[0]) |
              (static_cast<uint16_t>(buf[1]) << 8);
        cmd_ = static_cast<SpiCommon::Command>(buf[2]);
        err_ = static_cast<SpiCommon::ErrorCode>(buf[3]);
        if (len > HEADER_SIZE) {
            data_.assign(buf + HEADER_SIZE, buf + len);
        } else {
            data_.clear();
        }
        return true;
    }

    static std::optional<ClientMessage> fromBytes(const uint8_t* buf, size_t n) {
        if (buf == nullptr || n == 0) {
            LOGE("fromBytes, buf is null or empty");
            return std::nullopt;
        }
    
        ClientMessage msg;
        if (!msg.parse(buf, n)) {
            LOGE("fromBytes, failed to parse buffer");
            return std::nullopt;
        }
    
        return msg;  // optional 自動包裝
    }

    static std::optional<ClientMessage> fromBytes(const std::vector<uint8_t>& buf) {
        return fromBytes(buf.data(), buf.size());
    }

    // --- 新增: toBytes ---
    std::vector<uint8_t> toBytes() const {
        std::vector<uint8_t> out;
        out.reserve(3 + data_.size());

        out.push_back(static_cast<uint8_t>(seq_ & 0xFF));       // low byte
        out.push_back(static_cast<uint8_t>((seq_ >> 8) & 0xFF)); // high byte
        out.push_back(static_cast<uint8_t>(cmd_));
        out.push_back(static_cast<uint8_t>(err_));

        out.insert(out.end(), data_.begin(), data_.end());

        return out;
    }

    bool is_valid() const { return valid_; }
    
    // 輸出 seq_, cmd_, data_ 的 debug string
    std::string toString() const {
        std::ostringstream oss;
        oss << "SEQ=0x" << std::hex << std::setw(4) << std::setfill('0') << seq_
            << ", CMD=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cmd_)
            << ", ERR=0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(err_)
            << ", DATA=" << bytesToHexString(data_.data(), data_.size());
        return oss.str();
    }
};

