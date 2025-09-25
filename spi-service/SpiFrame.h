#pragma once
#include "SpiCommon.h"
#include "common.h"
#include <cstdint>
#include <vector>
#include <algorithm>

#undef LOG_TAG
#define LOG_TAG "spi-service/SpiFrame"

// SPI Frame: DIR(1 byte) + SEQ(2 bytes) + CMD(1 byte) + LEN(2 bytes) + PAYLOAD + CHECKSUM(2 bytes)
class SpiFrame {
public:
    static constexpr uint8_t MCU_ACK_SUCCESS = 0x00;
    static constexpr uint8_t MCU_ACK_FAIL = 0x01;

    // defined in SPI_format_V3.pptx
    enum class Direction : uint8_t {
        MCU2SOC = 0x55,
        SOC2MCU = 0xAA
    };

    enum class Command : uint8_t {
        WRITE_UNREL = 0x00,
        READ_UNREL = 0x01,
        RESPONSE = 0x02,
        ACK_STATUS = 0x03,
        WRITE = 0x04,
        READ = 0x05,
        SERVICE_MCU_REQUEST = 0x06,
        UNKNOWN_CMD = UINT8_MAX,
    };

    Direction direction_;
    uint16_t seq_id_;
    Command cmd_id_;
    uint16_t payload_len_;
    std::vector<uint8_t> payload_;
    uint16_t checksum_;

    size_t next_;
    bool valid_{false};

    SpiFrame() {}

    SpiFrame(Direction direction,
             uint16_t seq_id,
             Command cmd_id,
             const std::vector<uint8_t>& full_payload,
             size_t offset = 0)
        : direction_(direction), seq_id_(seq_id), cmd_id_(cmd_id), offset_(offset) {
        if (offset_ >= full_payload.size()) {
            payload_len_ = 0;
        } else {
            payload_len_ = std::min<uint16_t>(
                SpiCommon::MAX_SPI_PAYLOAD_SIZE,
                static_cast<uint16_t>(full_payload.size() - offset_)
            );
            payload_.assign(full_payload.begin() + offset_,
                            full_payload.begin() + offset_ + payload_len_);
            if (payload_.size() < SpiCommon::MAX_SPI_PAYLOAD_SIZE) {
                payload_.resize(SpiCommon::MAX_SPI_PAYLOAD_SIZE, 0);
            }
            valid_ = (payload_len_ > 0);
        }

        next_ = offset_ + payload_len_;
        checksum_ = calcChecksum();
    }

    bool parse(const std::vector<uint8_t>& buf) {
        if (buf.size() < SpiCommon::MAX_SPI_FRAME_SIZE) { // buffer size must be 128 bytes
            LOGW("parse, buf.size()=%u < %u", buf.size(), SpiCommon::MAX_SPI_FRAME_SIZE);
            return false;
        }

        direction_ = static_cast<Direction>(buf[0]);
        seq_id_ = buf[1] | (buf[2] << 8);
        cmd_id_ = static_cast<Command>(buf[3]);
        payload_len_ = buf[4] | (buf[5] << 8);

        if (payload_len_ == 0 || payload_len_ > SpiCommon::MAX_SPI_PAYLOAD_SIZE) {
            LOGW("parse, invalid payload_len=%u", payload_len_);
            return false;
        }

        if (buf.size() < 6 + payload_len_ + 2) {
            LOGW("parse, buf.size() < 6 + payload_len + 2");
            return false;
        }

        payload_.assign(buf.begin() + 6, buf.begin() + 6 + SpiCommon::MAX_SPI_PAYLOAD_SIZE);

        uint16_t recv_checksum = buf[SpiCommon::MAX_SPI_FRAME_SIZE - 2] | (buf[SpiCommon::MAX_SPI_FRAME_SIZE - 1] << 8);
        checksum_ = recv_checksum;
        valid_ = (recv_checksum == calcChecksum());

        if (!valid_) {
            LOGW("parse, checksum mismatch");
        }

        return valid_;
    }

    // parse bytes to SpiFrame
    static std::optional<SpiFrame> fromBytes(const std::vector<uint8_t>& buf) {
        SpiFrame frame;
        if (!frame.parse(buf)) {
            LOGW("fromBytes, parse failed");
            return std::nullopt;
        }
        return frame;
    }

    static std::optional<SpiFrame> fromBytes(const uint8_t* buf, size_t size) {
        if (!buf || size == 0) {
            LOGW("fromBytes, invalid buffer");
            return std::nullopt;
        }

        std::vector<uint8_t> vec(buf, buf + size);
        return fromBytes(vec);
    }

    // convert SpiFrame to bytes
    std::vector<uint8_t> toBytes() const {
        std::vector<uint8_t> out;

        out.push_back(static_cast<uint8_t>(direction_));
        out.push_back(seq_id_ & 0xFF);
        out.push_back((seq_id_ >> 8) & 0xFF);
        out.push_back(static_cast<uint8_t>(cmd_id_));
        out.push_back(payload_len_ & 0xFF);
        out.push_back((payload_len_ >> 8) & 0xFF);

        out.insert(out.end(), payload_.begin(), payload_.end());

        out.push_back(checksum_ & 0xFF);
        out.push_back((checksum_ >> 8) & 0xFF);

        return out;
    }

    bool is_valid() const { return valid_; }

    // output debug string
    std::string toString() const {
        return SpiCommon::bytesToHexString(toBytes());
    }

    std::vector<uint8_t> get_effective_payload() const {
        return std::vector<uint8_t>(payload_.begin(), payload_.begin() + payload_len_);
    }

    bool is_ack_success() const {
        return !payload_.empty() && payload_[0] == MCU_ACK_SUCCESS;
    }

    static bool is_ack_success(const uint8_t* buf, size_t len) {
        if (!buf || len < 7) { // dir(1)+seq(2)+cmd(1)+len(2)+payload(1)
            return false;
        }
        return buf[6] == MCU_ACK_SUCCESS;
    }

    static uint16_t get_seq_id(const uint8_t* buf, size_t len) {
        if (!buf || len < 3) { 
            LOGE("get_seq_id, invalid buffer!");
            return SpiCommon::INVALID_SEQ_ID;
        }
        return static_cast<uint16_t>((buf[1]) | (static_cast<uint16_t>(buf[2]) << 8));
    }
    
    static Command get_cmd_id(const uint8_t* buf, size_t len) {
        if (!buf || len < 4) {
            LOGE("get_cmd_id, invalid buffer!");
            return Command::UNKNOWN_CMD;
        }
        return static_cast<Command>(buf[3]);
    }

private:
    size_t offset_;

    // sum of all word in 16-bit Little Endian words
    uint16_t calcChecksum() const {
        std::vector<uint8_t> toBytes() const {
        std::vector<uint8_t> buf;
    
        // direction_ (1 byte)
        buf.push_back(static_cast<uint8_t>(direction_));
    
        // seq_id_ (2 bytes, little endian)
        buf.push_back(static_cast<uint8_t>(seq_id_ & 0xFF));
        buf.push_back(static_cast<uint8_t>((seq_id_ >> 8) & 0xFF));
    
        // cmd_id_ (2 bytes, little endian)
        buf.push_back(static_cast<uint8_t>(cmd_id_ & 0xFF));
        buf.push_back(static_cast<uint8_t>((cmd_id_ >> 8) & 0xFF));
    
        // payload_len_ (2 bytes, little endian)
        buf.push_back(static_cast<uint8_t>(payload_len_ & 0xFF));
        buf.push_back(static_cast<uint8_t>((payload_len_ >> 8) & 0xFF));
    
        // payload_
        buf.insert(buf.end(), payload_.begin(), payload_.end());
        
        return SpiCommon::calcChecksum(buf.data(), buf.size());
    }
};
