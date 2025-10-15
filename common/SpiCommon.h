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
//     SEQ_ID: 2 bytes, the message sequence number (little endian)
//     MSG_TYPE: 1 byte, the message type
//     ERROR_CODE: 1 byte, only valid in spi-service's RESPONSE
//     LEN: 2 bytes, payload length (little endian)
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
//
//  Client must actively reconnect to the SPI service if the socket is unexpectedly closed (e.g. service restart).

namespace SpiCommon {

// ========== Constants ==========
static constexpr size_t MAX_IPC_PACKET_SIZE = 2048;      // Maximum IPC packet length
static constexpr size_t MAX_IPC_DATA_SIZE = 1016;        // Maximum IPC payload size
static constexpr size_t MAX_SPI_FRAME_SIZE = 1024;       // SPI frame length = header(6 bytes) + MAX_PAYLOAD + checksum(2 bytes)
static constexpr size_t MAX_SPI_PAYLOAD_SIZE = 1016;     // Maximum SPI frame payload size
static constexpr size_t HEADER_SIZE = 6;                // SEQ(2) + MSG_TYPE(1) + ERR(1) + LEN(2)
static constexpr size_t TAIL_SIZE   = 2;                // Checksum(2)
static constexpr uint32_t SPI_SPEED = 10000000;         // 10 MHz

static constexpr uint16_t INVALID_SEQ_ID = 0;           // invalid sequence id
static constexpr size_t MAX_HEX_STRING_LOG_LEN = 20;    // Maximum hex string log length at toString()

static constexpr const char* IPC_SOCKET_PATH = "/tmp/spi-service/ipc.sock";

// Message Type reserved ranges:
//   0x00 ~ 0x0F: Reserved
//   0x10 ~ 0x1F: Client/Server commands
//   0x20 ~ 0xEF: General
//   0xF0 ~ 0xFF: Internal/Debug
enum class MsgType : uint8_t {
    NONE           = 0x00,  // No action

    // Client --> SPI Service
    REGISTER_REQ   = 0x10,  // Re-register client for MCU command list. (expects a response from SPI Service with the same SEQ_ID)
                            //     Data is a little-endian 16-bit array.
                            //     Values ≥ CMD_GROUP_ID_BASE are mapped to predefined command groups(CMD_GROUP).
                            // Clients register the MCU commands they can handle, including:
                            //     Notifications: unsolicited MCU messages
                            //          Requests: commands requiring a response
                            // The system uses this registration to route incoming MCU messages to the appropriate client handler.
    UNREGISTER_REQ = 0x11,  // Remove client from MCU command list. (expects a response from SPI Service with the same SEQ_ID)

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
    uint16_t len;
    std::vector<uint8_t> data; // mcu code (2 bytes, littlen endian) + parameters(len - 2)
    uint16_t checksum;
};

// ========== MCU(Sub) Command Enum ==========, sync from NORDIC/app/inc/hal/command_dispatcher.h
enum class McuCommand : uint16_t {
    SUB_CMD_GET_LED_STATUS,
    SUB_CMD_SET_LED,
    // NOTE: Add other sub-command IDs here in the future
    SUB_CMD_OTA_UPDATE_REQ, // Initiate OTA session
    SUB_CMD_OTA_UPDATE_STATUS, // Get OTA status
    SUB_CMD_OTA_DATA_CHUNK, // Send firmware chunk
    SUB_CMD_OTA_COMPLETE,   // Finalize update
    SUB_CMD_OTA_ABORT,      // Cancel update
    SUB_CMD_GET_HAL_STATUS,
    SUB_CMD_SET_HAL,
    SUB_CMD_HAL_CHANGE, // notification # step 1, add MCU command enum

    // Dev Info Commands
    SUB_CMD_GET_IMEI = 0x0304, // get_imei # step 1, add MCU command enum, get_imei@dev_info_hal.h
    SUB_CMD_GET_FW_INFO = 0x0308,
    SUB_CMD_GET_FW_INFO_FROM_SOC = 0x030A, // get_fw_info # step 1, add MCU command enum, get_fw_info@dev_info_hal.h

    SUB_CMD_INVALID = UINT16_MAX,
};

// ========== Predefined Command Groups ==========
static constexpr size_t CMD_GROUP_COUNT = 1;
static constexpr uint16_t CMD_GROUP_ID_BASE = 0xFF00;
const std::array<std::vector<uint16_t>, CMD_GROUP_COUNT> CMD_GROUP = {{
    {0x1001, 0x1002, 0x1003, 0xFFFF},        // group 0, spi-service internal test
}};

// ========== Utility Functions ==========

// Core function: convert any byte sequence to a hex string (uppercase + space-separated)
inline std::string bytesToHexString(const uint8_t* data, size_t len, size_t maxBytes = 0) {
    std::ostringstream oss;
    oss.setf(std::ios::uppercase);   // Uppercase fixed
    oss << std::hex;

    size_t limit = (maxBytes == 0) ? len : std::min(len, maxBytes);

    for (size_t i = 0; i < limit; i++) {
        oss << std::setw(2) << std::setfill('0')
            << static_cast<unsigned int>(data[i])
            << " ";
    }
    return oss.str();
}

inline std::string bytesToHexString(const std::vector<uint8_t>& buf, size_t maxBytes = 0) {
    return bytesToHexString(buf.data(), buf.size(), maxBytes);
}

inline std::string bytesToShortHexString(const uint8_t* rx_buf, size_t len) {
    if (!rx_buf || len == 0) {
        return "<empty>";
    }

    if (len > (MAX_HEX_STRING_LOG_LEN + TAIL_SIZE)) {
        std::string head = bytesToHexString(rx_buf, std::min(len, MAX_HEX_STRING_LOG_LEN));
        std::string tail = bytesToHexString(rx_buf + len - TAIL_SIZE, TAIL_SIZE);
        return head + " ... " + tail;
    } else {
        return bytesToHexString(rx_buf, len);
    }
}

inline std::string bytesToShortHexString(const std::vector<uint8_t>& buf) {
    return bytesToShortHexString(buf.data(), buf.size());
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

inline std::string MsgTypeToStr(MsgType msg) {
    switch (msg) {
        case MsgType::NONE:           return "NONE";
        case MsgType::REGISTER_REQ:   return "REGISTER_REQ";
        case MsgType::UNREGISTER_REQ: return "UNREGISTER_REQ";
        case MsgType::EXECUTE_REQ:    return "EXECUTE_REQ";
        case MsgType::SET_REQ:        return "SET_REQ";
        case MsgType::RESPONSE:       return "RESPONSE";
        case MsgType::NOTIFY:         return "NOTIFY";
        default:                      return "UNKNOWN";
    }
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

inline uint16_t get_mcu_code(std::vector<uint8_t>& data) {
    if (data.size() < 2) {
        return static_cast<uint16_t>(McuCommand::SUB_CMD_INVALID);
    }
    return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
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
    msg_out.len = buf[offset] | (buf[offset + 1] << 8);
    offset += 2;

    // DATA
    if (offset + msg_out.len > buf.size() - 2) {
        return false; // payload length is incorrect
    }
    msg_out.data.assign(buf.begin() + offset, buf.begin() + offset + msg_out.len);
    offset += msg_out.len;

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
