// SpiCommon.h
#pragma once
#include <cstdint>
#include <vector>
#include <cstddef>

//
// SPI IPC Protocol Definitions
// Shared by Client and Server (SPI service)
//
// Each message has the following structure:
//     SEQ_ID: 2 bytes, the message sequence number
//     MSG_TYPE: 1 byte, the message type
//     ERROR_CODE: 1 byte, only valid in spi-service's RESPONSE
//     DATA: variable-length payload
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
    static constexpr size_t MAX_IPC_PACKET_SIZE = 512;  // Maximum IPC packet length
    static constexpr size_t MAX_IPC_DATA_SIZE = 120;    // Maximum IPC payload size
    static constexpr size_t MAX_SPI_FRAME_SIZE = 128;   // SPI frame length = header(6 bytes) + MAX_PAYLOAD + checksum(2 bytes)
    static constexpr size_t MAX_SPI_PAYLOAD_SIZE = 120; // Maximum SPI frame payload size

    static constexpr const char* IPC_SOCKET_PATH = "/tmp/spi-service/ipc.sock";

    // Message Type reserved ranges:
    //   0x00 ~ 0x0F: Reserved
    //   0x10 ~ 0x1F: Client/Server commands
    //   0x20 ~ 0xEF: General
    //   0xF0 ~ 0xFF: Internal/Debug

    // ========== Commands ==========
    enum class MsgType : uint8_t {
        NONE           = 0x00,  // No action

        // Client --> SPI Service
        REGISTER_REQ   = 0x10,  // Re-register client for notifications (expects a response from SPI Service with the same SEQ_ID)
                                // Clients register the MCU commands they can handle, including both notifications
                                // (unsolicited MCU messages) and requests (commands requiring a response). The system uses this
                                // registration to route incoming MCU messages to the appropriate client handler.
        UNREGISTER_REQ = 0x11,  // Remove client from notification list (expects a response from SPI Service with the same SEQ_ID)

        // SOC <--> MCU
        EXECUTE_REQ    = 0x20,  // Request the other side to execute an action (expects a response from MCU with the same SEQ_ID)
        SET_REQ        = 0x21,  // Request the other side to execute an action (no response required)

        // MCU <--> SOC
        // SPI Service --> SOC
        RESPONSE       = 0x22,  // Response to a request (success / error / data)
                                // DATA field format: Error Code + Data

        // MCU <-- SOC
        NOTIFY         = 0x23,  // One-way notification, no response required
    };

    // ========== Error Codes ==========
    enum class ErrorCode : uint8_t {
        NONE            = 0x00, // No error
        INVALID_FORMAT  = 0x01, // Buffer length insufficient or format error
        DATA_TOO_LONG   = 0x02, // Payload exceeds maximum allowed length
        UNKNOWN_CMD     = 0x03, // Unknown command
        INTERNAL_ERROR  = 0x04, // Internal service error
    };

    // ========== MCU(Sub) Command Enum ==========
    // sync from NORDIC/app/inc/hal/command_dispatcher.h
    enum class McuCommand : uint16_t {
        SUB_CMD_GET_LED_STATUS,
        SUB_CMD_SET_LED,
        // NOTE: Add other sub-command IDs here in the future
        SUB_CMD_INVALID = UINT16_MAX,
    };
}
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
