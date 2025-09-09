#pragma once
#include <variant>
#include <vector>

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

struct IPCData {
    int client_fd;
    MessageFlow flow; // Message flow
    uint16_t seq; // Sequence ID (used for matching requests/responses)
    std::vector<uint8_t> data;
};

struct GPIOData {
    uint8_t level;
};

struct Packet {
    PacketSource source;       // source
    std::variant<std::monostate, IPCData, GPIOData> payload;
};
