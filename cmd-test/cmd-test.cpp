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
using ErrorCode = SpiCommon::ErrorCode;
using SpiCommon::bytesToHexString;
using SpiCommon::intToHexString;
using SpiCommon::hexStringToBytes;
using SpiCommon::msgToHexString;

uint16_t global_seq{0};
std::atomic<bool> running{true};

// Return default command title based on command number
std::string getMessageTitle(int cmd) {
    switch (cmd) {
        case 1: return "register 2, 3, 4, 5, 7 commands";
        case 2: return "register command group#0";
        case 3: return "unregister 4 5 commands";
        case 4: return "set MCU red LED on";
        case 5: return "execute MCU command";
        // special test for spi-service and MCU communication test only
        case 6: return "trigger MCU reliable write -> red LED twice";
        case 7: return "trigger MCU reliable read -> green LED twice";
        case 8: return "trigger MCU unreliable write -> blue LED twice";
        case 9: return "trigger MCU unreliable read -> white LED twice";
        default: return "Unknown Command";
    }
}

// Return default hex string based on command number
std::vector<uint8_t> getMessage(int cmd) {
    switch(cmd) {
        // Register the commands 02 03 04 05 07; the spi-service responds with a message carrying the same SEQ_ID indicating success or failure
        // and notifications for each of the four commands (02, 03, 04, 05).
        case 1: return packMessage(global_seq,
                    MsgType::REGISTER_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("02 00 03 00 04 00 05 00 07 00"));

        // Register the command group 0 defined at cmd_groups@SpiCommon.h
        case 2: return packMessage(global_seq,
                    MsgType::REGISTER_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("00 FF"));

        // Unregister the commands 02 and 03; the spi-service responds with a message carrying the same SEQ_ID indicating success or failure
        // and no further notifications will be received for these two commands.
        case 3: return packMessage(global_seq,
                    MsgType::UNREGISTER_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("04 00 05 00"));

        // Set the red LED to ON; no response message is returned.
        case 4: return packMessage(global_seq,
                    MsgType::SET_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("01 00 01")); // SUB_CMD_SET_LED (little endian)

        // Execute command; the MCU responds with a message carrying the same SEQ_ID and the execution result.
        case 5: return packMessage(global_seq,
                    MsgType::EXECUTE_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("07 00")); // SUB_CMD_ID_READ (little endian)

        // special test for spi-service and MCU communication test only
        case 6: return packMessage(global_seq,
                    MsgType::SET_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("08 00 00")); // SUB_CMD_ID_WRITE (little endian)
        case 7: return packMessage(global_seq,
                    MsgType::SET_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("08 00 01")); // SUB_CMD_ID_WRITE (little endian)
        case 8: return packMessage(global_seq,
                    MsgType::SET_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("08 00 02")); // SUB_CMD_ID_WRITE (little endian)
        case 9: return packMessage(global_seq,
                    MsgType::SET_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("08 00 03")); // SUB_CMD_ID_WRITE (little endian)
        default: return std::vector<uint8_t>{}; // default
    }
}

// Receiver thread: read from socket and print messages
void socketReceiver(int sock) {
    std::vector<uint8_t> buf(SpiCommon::MAX_IPC_PACKET_SIZE);
    while (running) {
        ssize_t n = read(sock, buf.data(), buf.size() - 1);
        if (n > 0) {
            std::cout << "Received --> " << msgToHexString(buf.data(), n) << std::endl;
            SpiCommon::Message msg;
            SpiCommon::unpackMessage(buf, msg);

            // MCU communication internal test
            if (msg.msg_type == MsgType::NOTIFY && msg.data.size() == 1 && (msg.data[0] == 0x01 || msg.data[0] == 0x03)) {
                std::cout << "MCU read test" << std::endl;
                std::vector<uint8_t> data = packMessage(msg.seq,
                    MsgType::SET_REQ,
                    ErrorCode::NONE,
                    ((msg.data[0] == 0x01) ? hexStringToBytes("01 00 C1 C2 C3") : hexStringToBytes("03 00 D1 D2 D3")));
                if (write(sock, data.data(), data.size()) < 0) {
                    perror("write");
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

        std::cout << getMessageTitle(cmd) << std::endl;
        global_seq++;
        std::vector<uint8_t> data = getMessage(cmd);
        std::cout << "Send <-- " << msgToHexString(data.data(), data.size()) << std::endl;

        // Send binary data
        if (write(sock, data.data(), data.size()) < 0) {
            perror("write");
            break;
        }
    }

    std::cout << "Exiting cmd_test.\n";
    running = false;
    if (recvThread.joinable()) recvThread.join();
    close(sock);
    return 0;
}
