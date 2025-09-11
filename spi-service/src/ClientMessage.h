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

// Client Message: SEQ_ID(2 bytes) + MSG_TYPE(1 byte) + ERROR_CODE(1 byte) + LEN(2 bytes) + DATA + Checksum (2 bytes)
class ClientMessage {
public:
    static constexpr size_t HEADER_SIZE = 6;   // SEQ(2) + MSG_TYPE(1) + ERR(1) + LEN(2)
    static constexpr size_t TAIL_SIZE   = 2;   // checksum(2)

    uint16_t seq_;              // sequence id (2 bytes, little endian)
    SpiCommon::MsgType  msg_t_; // message type (1 byte)
    SpiCommon::ErrorCode  err_; // error code
    uint16_t len_{0};           // data length
    std::vector<uint8_t> data_; // DATA (variable length)
    uint16_t checksum_{0};
    bool valid_{false};

    ClientMessage() : seq_(0), msg_t_(SpiCommon::MsgType::NONE), err_(SpiCommon::ErrorCode::NONE) {}

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
    ClientMessage(uint16_t seq_id, SpiCommon::MsgType msg_type, SpiCommon::ErrorCode err, std::vector<uint8_t> data = {})
        : seq_(seq_id), msg_t_(msg_type), err_(err), data_(data), valid_(true) {
        len_ = data.size();
        checksum_ = calcChecksum();
    }


    bool parse(const uint8_t* buf, size_t len) {
        if (len < HEADER_SIZE + TAIL_SIZE) {
            LOGE("parse, len (%d) < HEADER_SIZE + TAIL_SIZE (%d)", len, (HEADER_SIZE + TAIL_SIZE));
            return false;
        }
        
        seq_   = static_cast<uint16_t>(buf[0]) |
                (static_cast<uint16_t>(buf[1]) << 8);
        msg_t_ = static_cast<SpiCommon::MsgType>(buf[2]);
        err_   = static_cast<SpiCommon::ErrorCode>(buf[3]);
        len_   = static_cast<uint16_t>(buf[4]) |
                (static_cast<uint16_t>(buf[5]) << 8);
        
        // Expect total size = HEADER_SIZE + len_ + TAIL_SIZE
        size_t expected = HEADER_SIZE + len_ + TAIL_SIZE;
        if (len < expected) {
            LOGE("parse, invalid length: expect %zu, got %zu", expected, len);
            return false;
        }
        
        // Copy data
        if (len_ > 0) {
            data_.assign(buf + HEADER_SIZE, buf + HEADER_SIZE + len_);
        } else {
            data_.clear();
        }
        
        // Decode checksum (little endian)
        size_t cs_offset = HEADER_SIZE + len_;
        checksum_ = static_cast<uint16_t>(buf[cs_offset]) |
                   (static_cast<uint16_t>(buf[cs_offset + 1]) << 8);

        // Verify checksum
        uint16_t calc = calcChecksum(buf, HEADER_SIZE + len_);
        if (checksum_ != calc) {
            LOGE("parse, checksum mismatch: got=0x%04X, calc=0x%04X",
                 checksum_, calc);
            return false;
        }
        valid_ = true;
        return true;
    }

    // 16-bit sum of all bytes
    static uint16_t calcChecksum(const uint8_t* buf, size_t len) {
        uint32_t sum = 0;
        for (size_t i = 0; i < len; i++) {
            sum += buf[i];
        }
        return static_cast<uint16_t>(sum & 0xFFFF);
    }
    
    uint16_t calcChecksum() const {
        uint32_t sum = 0;
    
        // SEQ (2 bytes, little endian)
        sum += seq_ & 0xFF;
        sum += (seq_ >> 8) & 0xFF;
    
        // MSG_TYPE (1 byte)
        sum += static_cast<uint8_t>(msg_t_);
    
        // ERROR_CODE (1 byte)
        sum += static_cast<uint8_t>(err_);
    
        // LEN (2 bytes, little endian)
        sum += len_ & 0xFF;
        sum += (len_ >> 8) & 0xFF;
    
        // DATA (N bytes)
        for (auto b : data_) {
            sum += b;
        }
    
        return static_cast<uint16_t>(sum & 0xFFFF);
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

        return msg;  // Automatically wrapped into std::optional
    }

    static std::optional<ClientMessage> fromBytes(const std::vector<uint8_t>& buf) {
        return fromBytes(buf.data(), buf.size());
    }

    std::vector<uint8_t> toBytes() const {
        std::vector<uint8_t> out;
        out.reserve(HEADER_SIZE + data_.size() + TAIL_SIZE);
    
        // SEQ_ID (little endian)
        out.push_back(static_cast<uint8_t>(seq_ & 0xFF));
        out.push_back(static_cast<uint8_t>((seq_ >> 8) & 0xFF));
    
        // MSG_TYPE
        out.push_back(static_cast<uint8_t>(msg_t_));
    
        // ERROR_CODE
        out.push_back(static_cast<uint8_t>(err_));
    
        // LEN (little endian)
        uint16_t len_field = static_cast<uint16_t>(data_.size());
        out.push_back(static_cast<uint8_t>(len_field & 0xFF));
        out.push_back(static_cast<uint8_t>((len_field >> 8) & 0xFF));
    
        // DATA
        out.insert(out.end(), data_.begin(), data_.end());
    
        // checksum
        out.push_back(static_cast<uint8_t>(checksum_ & 0xFF));
        out.push_back(static_cast<uint8_t>((checksum_ >> 8) & 0xFF));
        return out;
    }

    bool is_valid() const { return valid_; }

    std::string toString() const {
        std::ostringstream oss;
        oss << std::uppercase << std::hex;
    
        oss << "SEQ=0x" << std::setw(4) << std::setfill('0') << seq_
            << ", TYP=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(msg_t_)
            << ", ERR=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(err_)
            << ", LEN=0x" << std::setw(4) << std::setfill('0') << len_
            << ", DATA=" << bytesToHexString(data_.data(), data_.size())
            << ", CHKSUM=0x" << std::setw(4) << std::setfill('0') << checksum_;
    
        return oss.str();
    }

};

