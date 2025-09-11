#include "SpiCommon.h"

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <atomic>
#include <thread>

using MsgType = SpiCommon::MsgType;
using McuCommand = SpiCommon::McuCommand;

uint16_t global_seq{0};
std::atomic<bool> running{true};

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

// Generic integral -> hex string (Little Endian)
template <typename T,
          typename = std::enable_if_t<std::is_integral<T>::value>>
inline std::string intToHexStringLE(T value) {
    uint8_t bytes[sizeof(T)];
    for (size_t i = 0; i < sizeof(T); i++) {
        bytes[i] = static_cast<uint8_t>((value >> (8 * i)) & 0xFF);  // Little-endian
    }
    return bytesToHexString(bytes, sizeof(T));
}

// Buffer -> hex string
inline std::string bufferToHexString(const uint8_t* buf, size_t len) {
    return bytesToHexString(buf, len);
}

// Convert hex string to binary
std::vector<uint8_t> hexStringToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    std::istringstream iss(hex);
    std::string byteStr;
    while (iss >> byteStr) {
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteStr, nullptr, 16));
        bytes.push_back(byte);
    }
    return bytes;
}

// Return default command title based on command number
std::string getCommandTitle(int cmd) {
    switch (cmd) {
        case 1: return "Register: 02, 03, 04, 05, 07";
        case 2: return "Unregister: 04 05";
        case 3: return "Execute MCU Command";
        case 4: return "Set MCU Red LED On";
        case 5: return "Set MCU Red LED Off";
        default: return "Unknown Command";
    }
}

// Return default hex string based on command number
std::string getCommandHexString(int cmd) {
    switch(cmd) {
        // Register the commands 02 03 04 05 07; the spi-service responds with a message carrying the same SEQ_ID indicating success or failure
        // and notifications for each of the four commands (02, 03, 04, 05).
        case 1: return intToHexStringLE(global_seq) + " "                             // --> SEQ_ID
                + intToHexStringLE(static_cast<uint8_t>(MsgType::REGISTER_REQ)) + " " // --> COMMAND
                + "00" + " "                                                          // --> ERROR
                + "02 03 04 05 07";                                                      // --> DATA

        // Unregister the commands 02 and 03; the spi-service responds with a message carrying the same SEQ_ID indicating success or failure
        // and no further notifications will be received for these two commands.
        case 2: return intToHexStringLE(global_seq) + " "                               // --> SEQ_ID
                + intToHexStringLE(static_cast<uint8_t>(MsgType::UNREGISTER_REQ)) + " " // --> COMMAND
                + "00" + " "                                                            // --> ERROR (only valid in spi-service's RESPONSE)
                + "04 05";                                                              // --> DATA

        // Execute command 06; the MCU responds with a message carrying the same SEQ_ID and the execution result.
        case 3: return intToHexStringLE(global_seq) + " "                               // --> SEQ_ID
                + intToHexStringLE(static_cast<uint8_t>(MsgType::EXECUTE_REQ)) + " "    // --> COMMAND
                + "00" + " "                                                            // --> ERROR (only valid in spi-service's RESPONSE)
                + "06 00 01";                                                           // --> DATA := {MCU COMMAND + DATA} (for MCU)

        // Set the red LED to ON; no response message is returned.
        case 4: return intToHexStringLE(global_seq) + " "                                               // --> SEQ_ID
                + intToHexStringLE(static_cast<uint8_t>(MsgType::SET_REQ)) + " "                        // --> COMMAND
                + "00" + " "                                                                            // --> ERROR (only valid in spi-service's RESPONSE)
                + intToHexStringLE(static_cast<uint16_t>(McuCommand::SUB_CMD_SET_LED)) + " " + "00 01"; // --> DATA := {MCU COMMAND + DATA} (for MCU)

        // Set the red LED to OFF; no response message is returned.
        case 5: return intToHexStringLE(global_seq) + " "                                               // --> SEQ_ID
                + intToHexStringLE(static_cast<uint8_t>(MsgType::SET_REQ)) + " "                        // --> COMMAND
                + "00" + " "                                                                            // --> ERROR (only valid in spi-service's RESPONSE)
                + intToHexStringLE(static_cast<uint16_t>(McuCommand::SUB_CMD_SET_LED)) + " " + "00 00"; // --> DATA := {MCU COMMAND + DATA} (for MCU)

        default: return "00"; // default
    }
}

// Get byte at specific index, throws if out-of-range
uint8_t getByteAt(const std::vector<uint8_t>& data, size_t index) {
    if (index >= data.size()) {
        throw std::out_of_range("Index out of range in getByteAt()");
    }
    return data[index];
}

// Format message buffer into hex string for logging
std::string msgToHexString(const uint8_t* buf, size_t len) {
    uint16_t seq_id = (static_cast<unsigned char>(buf[1]) << 8) | static_cast<unsigned char>(buf[0]);
    uint8_t cmd = static_cast<unsigned char>(buf[2]);
    uint8_t error = static_cast<unsigned char>(buf[3]);

    std::string data_hex;
    if (len > 4) data_hex = bufferToHexString(buf + 4, len - 4);

    std::ostringstream oss;
    oss << "SEQ:"
        << std::setw(2) << std::setfill('0') << std::hex << std::uppercase
        << (int)(unsigned char)buf[0] << " "
        << std::setw(2) << (int)(unsigned char)buf[1]
        << ", CMD:" << std::setw(2) << (int)cmd
        << " ERROR:" << std::setw(2) << (int)error;

    if (!data_hex.empty()) {
        oss << " DATA:" << data_hex;
    }
    return oss.str();
}

// Receiver thread: read from socket and print messages
void socketReceiver(int sock) {
    char buf[1024];
    while (running) {
        ssize_t n = read(sock, buf, sizeof(buf) - 1);
        if (n > 0) {
            std::cout << "Received --> " << msgToHexString(reinterpret_cast<uint8_t *>(buf), n) << std::endl;
            if (sizeof(buf) > 5) {
                uint16_t mcu_cmd = static_cast<uint16_t>(buf[4]) | (static_cast<uint16_t>(buf[5]) << 8);
                // result is required for mcu command 07
                if (mcu_cmd == 0x07) {                    
                    std::vector<uint8_t> data = hexStringToBytes("01 00 21 00 07 00 D1 D2 D3");
                    if (write(sock, data.data(), data.size()) < 0) {
                        perror("write");
                    }
                }
            }
            std::cout << "Command> ";
        } else if (n == 0) {
            std::cout << "[Receiver] Connection closed by server.\n";
            running = false;
            break;
        } else {
            perror("[Receiver] read");
            running = false;
            break;
        }
    }
}

int main() {
    // Create UNIX Domain Socket
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, SpiCommon::IPC_SOCKET_PATH, sizeof(addr.sun_path) - 1);

    // Connect to SPI service
    if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("connect");
        close(sock);
        return 1;
    }

    std::cout << "Connected to SPI service. Enter command number (0 to exit).\n";

    // Start receiver thread
    std::thread recvThread(socketReceiver, sock);

    while (true) {
        std::cout << "Command> ";
        int cmd;
        std::cin >> cmd;
        if (!std::cin) break; // Invalid input
        if (cmd == 0) break;

        std::cout << getCommandTitle(cmd) << std::endl;
        global_seq++;
        std::string hexStr = getCommandHexString(cmd);
        std::vector<uint8_t> data = hexStringToBytes(hexStr);
        std::cout << "Send <-- " << msgToHexString(data.data(), data.size()) << std::endl;

        // Send binary data
        if (write(sock, data.data(), data.size()) < 0) {
            perror("write");
            break;
        }

        std::cout << "Sent command [" << cmd << "]: " << hexStr << std::endl;

        if (getByteAt(data, 3) == 0x04 || getByteAt(data, 3) == 0x05) {
            // Reliable read/write: wait for response
            char buf[1024];
            ssize_t n = read(sock, buf, sizeof(buf) - 1);
            if (n > 0) {
                buf[n] = '\0';
                std::cout << "Received: " << buf << std::endl;
            } else {
                std::cout << "No response or connection closed." << std::endl;
            }
        }
    }

    std::cout << "Exiting cmd_test.\n";
    running = false;
    if (recvThread.joinable()) recvThread.join();
    close(sock);
    return 0;
}
