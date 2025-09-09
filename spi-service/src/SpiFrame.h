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
    //static constexpr size_t MAX_FRAME_SIZE = 128; // MAX_FRAME_SIZE = 6(header)+MAX_PAYLOAD+2(checksum)
    //static constexpr size_t MAX_PAYLOAD = 120;
    
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
        SERVICE_MCU_REQUEST = 0x06
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
            valid_ = (payload_len_ > 0);
        }
    
        next_ = offset_ + payload_len_;
        checksum_ = computeChecksum();
    }
    
    bool parse(const std::vector<uint8_t>& buf) {
        if (buf.size() < 8) { // header(6) + checksum(2)
            LOGW("parse, buf.size() < 8");
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
    
        payload_.assign(buf.begin() + 6, buf.begin() + 6 + payload_len_);
        uint16_t recv_checksum = buf[6 + payload_len_] | (buf[7 + payload_len_] << 8);
    
        checksum_ = recv_checksum;
        valid_ = (recv_checksum == computeChecksum());
    
        if (!valid_) {
            LOGW("parse, checksum mismatch");
        }
    
        return valid_;
    }

    // 從 bytes 解析成 SpiFrame
    static std::optional<SpiFrame> fromBytes(const std::vector<uint8_t>& buf) {
        SpiFrame frame;
        if (!frame.parse(buf)) {
            LOGW("fromBytes, parse failed");
            return std::nullopt;
        }
        return frame;
    }

    // 把 frame 打包成 bytes
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
    
    // 輸出 seq, cmd, data 的 debug string
    std::string toString() const {
        return bytesToHexString(toBytes());
    }
 
private:
    // Constructor: private, 直接用 payload 初始化（避免 fromBytes 多餘 copy）
    SpiFrame(Direction direction,
             uint16_t seq_id,
             Command cmd_id,
             const std::vector<uint8_t>& payload,
             uint16_t checksum,
             size_t offset,
             bool valid)
        : direction_(direction), seq_id_(seq_id), cmd_id_(cmd_id),
          payload_len_(static_cast<uint16_t>(payload.size())), payload_(payload),
          checksum_(checksum), offset_(offset), valid_(valid)
    {
        next_ = offset_ + payload_len_;
    }
    
    uint16_t computeChecksum() const {
        uint32_t sum = 0;

        sum += static_cast<uint8_t>(direction_);
        sum += seq_id_ & 0xFF;
        sum += (seq_id_ >> 8) & 0xFF;
        sum += static_cast<uint8_t>(cmd_id_);
        sum += payload_len_ & 0xFF;
        sum += (payload_len_ >> 8) & 0xFF;
    
        for (auto b : payload_) {
            sum += b;
        }

        return static_cast<uint16_t>(sum & 0xFFFF);
    }
    size_t offset_;
    
};
