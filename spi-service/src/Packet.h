#pragma once
#include <variant>
#include <vector>
#include <sstream>
#include <iomanip>

enum class PacketSource : uint8_t {
    IPC,
    GPIO,
    READ
};

enum class MessageFlow : uint8_t {
    NONE,

    // IPC --> SPI
    EXECUTE = 0x10,  // expects a response from MCU
    SET = 0x11,      // no response required

    // IPC <-- SPI
    NOTIFY = 0x20,   // deliver to all registered clients
    RESPONSE  = 0x21 // deliver only to the client that issued EXECUTE_REQ
};

class IPCData {
public:
    int client_fd;
    MessageFlow flow;        // Message flow
    uint16_t seq;            // Sequence ID (used for matching requests/responses)
    std::vector<uint8_t> data;

    IPCData() = default;

    IPCData(int fd, MessageFlow f, uint16_t s, std::vector<uint8_t> d)
        : client_fd(fd), flow(f), seq(s), data(std::move(d)) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "IPCData{"
            << "client_fd=" << client_fd
            << ", flow=" << static_cast<int>(flow)
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