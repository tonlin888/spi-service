// SpiCommon.h
#pragma once
#include <cstdint>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <type_traits>

//
// SPI IPC Protocol Definitions
// Shared by Client and Server (SPI service)
//
// Each message has the following structure:
//     SEQ_ID: 2 bytes, the message sequence number
//     MSG_TYPE: 1 byte, the message type
//     ERROR_CODE: 1 byte, only valid in spi-service's RESPONSE
//     LEN: 2 bytes, payload length
//     DATA: variable-length payload
//     CHECKSUM: 2 bytes, 16-bit sum of all fields except CHECKSUM
//
// Command modes:
// 1) Execute / Response
//    Direction: SoC --> MCU
//    Purpose: Operations that require execution and a reply
//    Flow: Client sends an Execute Message; after completing the action,
//          the MCU replies with a Response Message
//    Rule: The Execute Message and its corresponding Response Message
//          share the same Sequence ID
//
// 2) Set (no Response)
//    Direction: SoC --> MCU
//    Purpose: One-way configuration, no acknowledgment required
//
// 3) Notify (from MCU)
//    Direction: MCU --> SoC
//    Purpose: Event notifications, e.g., sensor data updates
//
// 4) Notify / Set
//    Direction: MCU --> SoC
//    Purpose: Operations that require execution and a reply
//    Flow: Client receives a Notify Message from the MCU, performs
//          the requested action, and replies with a Set Message
//    Rule: The Notify Message and its corresponding Set Message
//          share the same Sequence ID

namespace SpiCommon {

// ========== Constants ==========
static constexpr size_t MAX_IPC_PACKET_SIZE = 512;      // Maximum IPC packet length
static constexpr size_t MAX_IPC_DATA_SIZE = 120;        // Maximum IPC payload size
static constexpr size_t MAX_SPI_FRAME_SIZE = 128;       // SPI frame length = header(6 bytes) + MAX_PAYLOAD + checksum(2 bytes)
static constexpr size_t MAX_SPI_PAYLOAD_SIZE = 120;     // Maximum SPI frame payload size
static constexpr size_t HEADER_SIZE = 6;                // SEQ(2) + MSG_TYPE(1) + ERR(1) + LEN(2)
static constexpr size_t TAIL_SIZE   = 2;                // checksum(2)

static constexpr uint16_t INVALID_SEQ_ID = 0;           // invalid sequence id

static constexpr const char* IPC_SOCKET_PATH = "/tmp/spi-service/ipc.sock";

// Message Type reserved ranges:
//   0x00 ~ 0x0F: Reserved
//   0x10 ~ 0x1F: Client/Server commands
//   0x20 ~ 0xEF: General
//   0xF0 ~ 0xFF: Internal/Debug
enum class MsgType : uint8_t {
    NONE           = 0x00,  // No action

    // Client --> SPI Service
    REGISTER_REQ   = 0x10,  // Re-register client for notifications (expects a response from SPI Service with the same SEQ_ID)
                            // Clients register the MCU commands they can handle, including both notifications
                            // (unsolicited MCU messages) and requests (commands requiring a response). The system uses this
                            // registration to route incoming MCU messages to the appropriate client handler.
    UNREGISTER_REQ = 0x11,  // Remove client from notification list (expects a response from SPI Service with the same SEQ_ID)

    // Client --> SPI Service ---> MCU
    EXECUTE_REQ    = 0x20,  // Request the other side to execute an action (expects a response from MCU with the same SEQ_ID)
    SET_REQ        = 0x21,  // Request the other side to execute an action (no response required)

    // Client <--- SPI Service <--- MCU
    // Client <--- SPI Service
    RESPONSE       = 0x22,  // Response to a request (success / error / data)

    // Client <--- SPI Service <-- MCU
    NOTIFY         = 0x23,  // One-way notification, no response required
};

enum class ErrorCode : uint8_t {
    NONE            = 0x00, // No error
    INVALID_FORMAT  = 0x01, // Buffer length insufficient or format error
    DATA_TOO_LONG   = 0x02, // Payload exceeds maximum allowed length
    UNKNOWN_CMD     = 0x03, // Unknown command
    INTERNAL_ERROR  = 0x04, // Internal service error
};

struct Message {
    uint16_t seq;
    MsgType msg_type;
    ErrorCode err;
    std::vector<uint8_t> data;
    uint16_t checksum;
};

// ========== MCU(Sub) Command Enum ==========, sync from NORDIC/app/inc/hal/command_dispatcher.h
enum class McuCommand : uint16_t {
    SUB_CMD_GET_LED_STATUS,
    SUB_CMD_SET_LED,
    // NOTE: Add other sub-command IDs here in the future
    SUB_CMD_INVALID = UINT16_MAX,
};

// ========== Utility Functions ==========

// Core function: convert any byte sequence to a hex string (uppercase + space-separated)
inline std::string bytesToHexString(const uint8_t* data, size_t len) {
    std::ostringstream oss;
    oss.setf(std::ios::uppercase);   // Uppercase fixed
    oss << std::hex;
    for (size_t i = 0; i < len; i++) {
        oss << std::setw(2) << std::setfill('0')
            << static_cast<unsigned int>(data[i]);
        if (i != len - 1) oss << " ";  // Space between bytes
    }
    return oss.str();
}

inline std::string bytesToHexString(const std::vector<uint8_t>& buf) {
    return bytesToHexString(buf.data(), buf.size());
}

// Generic integral -> hex string (Little Endian)
template <typename T,
        typename = std::enable_if_t<std::is_integral<T>::value>>
inline std::string intToHexString(T value) {
    uint8_t bytes[sizeof(T)];
    for (size_t i = 0; i < sizeof(T); i++) {
        bytes[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);  // Little-endian
    }
    return bytesToHexString(bytes, sizeof(T));
}

// Convert hex string to binary
inline std::vector<uint8_t> hexStringToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    std::istringstream iss(hex);
    std::string byteStr;
    while (iss >> byteStr) {
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteStr, nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

// Format message buffer into hex string for logging
inline std::string msgToHexString(const uint8_t* buf, size_t len) {
    std::ostringstream oss;
    oss << std::uppercase << std::hex;

    if (len < 8) { // minimal size: SEQ(2) + TYPE(1) + ERR(1) + LEN(2) + CHECKSUM(2)
        oss << "Buffer too short (" << len << " bytes)";
        return oss.str();
    }

    size_t offset = 0;

    // SEQ_ID (2 bytes, little endian)
    uint16_t seq = buf[offset] | (buf[offset+1] << 8);
    oss << "SEQ_ID=0x" << std::setw(4) << std::setfill('0') << seq << " ";
    offset += 2;

    // MSG_TYPE (1 byte)
    oss << "MSG_TYPE=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(buf[offset]) << " ";
    offset += 1;

    // ERROR_CODE (1 byte)
    oss << "ERR=0x" << std::setw(2) << std::setfill('0') << static_cast<int>(buf[offset]) << " ";
    offset += 1;

    // LEN (2 bytes, little endian)
    uint16_t payload_len = buf[offset] | (buf[offset+1] << 8);
    oss << "LEN=0x" << std::setw(4) << std::setfill('0') << payload_len << " ";
    offset += 2;

    // DATA (payload_len bytes)
    oss << "DATA=";
    if (payload_len > 0 && offset + payload_len <= len - 2) { // minus 2 for checksum
        for (size_t i = 0; i < payload_len; ++i) {
            if (i != 0) oss << " ";
            oss << std::setw(2) << std::setfill('0') << static_cast<int>(buf[offset + i]);
        }
    } else {
        // oss << "(invalid length)";
    }
    offset += payload_len;

    // CHECKSUM (2 bytes at the end)
    if (len >= offset + 2) {
        uint16_t checksum = buf[offset] | (buf[offset+1] << 8);
        oss << " CHECKSUM=0x" << std::setw(4) << std::setfill('0') << checksum;
    } else {
        oss << " CHECKSUM=(missing)";
    }

    return oss.str();
}

// sum of all word in 16-bit Little Endian words
inline uint16_t calcChecksum(const uint8_t* buf, size_t len) {
    uint32_t sum = 0;
    size_t i = 0;

    for (; i + 1 < len; i += 2) {
        uint16_t word = buf[i] | (buf[i + 1] << 8);
        sum += word;
    }

    if (i < len) {
        sum += buf[i];
    }
    return static_cast<uint16_t>(sum & 0xFFFF);
}

// Pack message into binary buffer
inline std::vector<uint8_t> packMessage(uint16_t seq,
                                        MsgType msg_type,
                                        ErrorCode err,
                                        const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> buf;

    // SEQ_ID: 2 bytes, little endian
    buf.push_back(static_cast<uint8_t>(seq & 0xFF));
    buf.push_back(static_cast<uint8_t>((seq >> 8) & 0xFF));

    // MSG_TYPE: 1 byte
    buf.push_back(static_cast<uint8_t>(msg_type));

    // ERROR_CODE: 1 byte
    buf.push_back(static_cast<uint8_t>(err));

    // LEN: 2 bytes, little endian
    uint16_t payload_len = static_cast<uint16_t>(data.size());
    buf.push_back(static_cast<uint8_t>(payload_len & 0xFF));
    buf.push_back(static_cast<uint8_t>((payload_len >> 8) & 0xFF));

    // DATA: variable length
    buf.insert(buf.end(), data.begin(), data.end());

    // CHECKSUM: 2 bytes, sum of all previous bytes
    uint16_t checksum = calcChecksum(buf.data(), buf.size());
    buf.push_back(static_cast<uint8_t>(checksum & 0xFF));
    buf.push_back(static_cast<uint8_t>((checksum >> 8) & 0xFF));

    return buf;
}

inline bool unpackMessage(const std::vector<uint8_t>& buf, Message& msg_out) {
    if (buf.size() < (HEADER_SIZE + TAIL_SIZE)) return false;

    size_t offset = 0;

    // SEQ_ID
    msg_out.seq = buf[offset] | (buf[offset + 1] << 8);
    offset += 2;

    // MSG_TYPE
    msg_out.msg_type = static_cast<MsgType>(buf[offset]);
    offset += 1;

    // ERROR_CODE
    msg_out.err = static_cast<ErrorCode>(buf[offset]);
    offset += 1;

    // LEN
    uint16_t payload_len = buf[offset] | (buf[offset + 1] << 8);
    offset += 2;

    // DATA
    if (offset + payload_len > buf.size() - 2) {
        return false; // payload length is incorrect
    }
    msg_out.data.assign(buf.begin() + offset, buf.begin() + offset + payload_len);
    offset += payload_len;

    // CHECKSUM
    msg_out.checksum = buf[offset] | (buf[offset + 1] << 8);

    return true;
}

} // namespace SpiCommon
// ========== Sample Code ==========
//
//  #include <sys/socket.h>
//  #include <sys/un.h>
//  #include <unistd.h>
//  #include <cstring>
//
//  #include "SpiCommon.h"
//
//  // Create a UNIX domain socket
//  int sock = socket(AF_UNIX, SOCK_STREAM, 0);
//  if (sock < 0) {
//      perror("socket"); // Print error if socket creation fails
//      return 1;
//  }
//
//  // Prepare the socket address structure
//  sockaddr_un addr{};
//  addr.sun_family = AF_UNIX; // Specify UNIX domain
//  std::strncpy(addr.sun_path, SpiCommon::IPC_SOCKET_PATH, sizeof(addr.sun_path) - 1);
//  // Copy the socket path, leaving room for null terminator
//
//  // Connect to the server socket
//  if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
//      perror("connect"); // Print error if connection fails
//      close(sock);       // Close socket before exiting
//      return 1;
//  }
//
//  // Read data from the socket
//  ssize_t bytes_read = read(sock, buf, sizeof(buf) - 1);
//  // read() returns the number of bytes actually read, or -1 on error
//  // buf is a buffer to store the received data
//  // sizeof(buf) - 1 leaves space for a null terminator if needed (for text)
//
//  // Write data to the socket
//  ssize_t bytes_written = write(sock, data.data(), data.size());
//  // write() returns the number of bytes actually written to the socket
//  // On success: returns a non-negative number ≤ data.size()
//  //   - This number may be less than data.size() if the socket buffer is full
//  // On error: returns -1 and sets errno
//  // Note: For blocking sockets, write usually writes all bytes unless an error occurs
//
//  // Close the socket when done
//  close(sock);
