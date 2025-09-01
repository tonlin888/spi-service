#pragma once
#include <variant>
#include <vector>

enum class PacketSource {
    IPC,
    GPIO
};

struct IPCData {
    int client_fd;
    std::vector<uint8_t> data;
};

struct GPIOData {
    bool level_low;
};

struct Packet {
    PacketSource source;       // source
    std::variant<IPCData, GPIOData> payload;
};
