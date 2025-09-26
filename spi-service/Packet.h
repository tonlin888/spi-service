#pragma once
#include "SpiCommon.h"
#include <variant>
#include <vector>
#include <sstream>
#include <iomanip>

enum class PacketSource : uint8_t {
    IPC,
    GPIO,
    READ
};

class IPCData {
public:
    int client_fd;
    SpiCommon::MsgType typ;    // Message type
    uint16_t seq;               // Sequence ID (used for matching requests/responses)
    std::vector<uint8_t> data;

    IPCData() = default;

    IPCData(int fd, SpiCommon::MsgType typ, uint16_t s, std::vector<uint8_t> d)
        : client_fd(fd), typ(typ), seq(s), data(std::move(d)) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "IPCData{"
            << "client_fd=" << client_fd
            << ", " << SpiCommon::MsgTypeToStr(typ)
            << ", seq=" << seq
            << ", data=[";

        for (size_t i = 0; i < data.size(); ++i) {
            if (i > 0) oss << " ";
            oss << "0x" << std::uppercase << std::setw(2)
                << std::setfill('0') << std::hex
                << static_cast<int>(data[i]);
        }
        oss << "]}";
        return oss.str();
    }
};

struct GPIOData {
    uint8_t level;
};

struct Packet {
    PacketSource source;       // source
    std::variant<std::monostate, IPCData, GPIOData> payload;
};