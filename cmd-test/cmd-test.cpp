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

// get_fw_info # step 2, include header or define structure, dev_info_hal.h
typedef struct {
    char version[64];  /* Semantic version: "1.2.3" or "v7.1.0-rc2" */
    char variant[16];  /* Build variant: "release", "debug", "commercial" */
    char build_id[64]; /* Unique build identifier with timestamp/commit */
}fw_ver_info_t;

typedef struct {
    fw_ver_info_t nordic_fw;   /* Snap SDK, Zephyr and FIH components version */
    fw_ver_info_t modem_fw;    /* cellular and wlan modem version */
    fw_ver_info_t sailfish_fw; /* Combined FW version */
} fw_info_t;

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
        
        case 10: return "register SUB_CMD_GET_IMEI";
        case 11: return "call get_fw_info_handler()";
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
                    
        // get_imei # step 2, Register SUB_CMD_GET_IMEI
        // notification # step 2, Register SUB_CMD_HAL_CHANGE
        // get_fw_info # step 3, Register SUB_CMD_GET_FW_INFO_FROM_SOC
        case 10: return packMessage(global_seq,
                    MsgType::REGISTER_REQ,
                    ErrorCode::NONE,
                    hexStringToBytes("09 00 04 03 08 03 0A 03"));
        default: return std::vector<uint8_t>{}; // default
    }
}

// get_fw_info # step 4, create get_fw_info_handler
// 將任意記憶體區塊轉為 hex string
std::string to_hex_string(const void* data, size_t len) {
    const unsigned char* bytes = reinterpret_cast<const unsigned char*>(data);
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (size_t i = 0; i < len; ++i) {
        oss << std::setw(2) << static_cast<int>(bytes[i]);
        if (i != len - 1) oss << " ";
    }
    return oss.str();
}

void get_fw_info_handler(int sock, SpiCommon::Message& msg) {
    std::cout << "get_fw_info_handler()" << std::endl;

    fw_info_t fw_info{};  // 全部自動初始化為 0

    // 用安全拷貝填入字串
    auto set_fw = [](fw_ver_info_t& fw, const char* ver, const char* var, const char* build) {
        std::strncpy(fw.version, ver, sizeof(fw.version) - 1);
        std::strncpy(fw.variant, var, sizeof(fw.variant) - 1);
        std::strncpy(fw.build_id, build, sizeof(fw.build_id) - 1);
    };

    set_fw(fw_info.nordic_fw,   "1.0.1", "debug", "20251015094712");
    set_fw(fw_info.modem_fw,    "1.0.2", "debug", "20251015094713");
    set_fw(fw_info.sailfish_fw, "1.0.3", "debug", "20251015094714");

    // mcu code: SUB_CMD_GET_FW_INFO_FROM_SOC
    std::string data = SpiCommon::intToHexString(static_cast<uint16_t>(McuCommand::SUB_CMD_GET_FW_INFO_FROM_SOC));
    // fill fw_info
    data += to_hex_string(&fw_info, sizeof(fw_info));
    
    // Respond to the MCU's request using the same sequence ID from the NOTIFY
    std::vector<uint8_t> raw_msg = packMessage(msg.seq,
        MsgType::SET_REQ,
        ErrorCode::NONE,
        hexStringToBytes(data));
    if (write(sock, raw_msg.data(), raw_msg.size()) < 0) {
        perror("socke write error!");
    }
}

// get_imei # step 3, create get_imei_handler
void get_imei_handler(int sock, SpiCommon::Message& msg) {
    std::cout << "get_imei_handler()" << std::endl;

    // mcu code: SUB_CMD_GET_IMEI
    std::string data = SpiCommon::intToHexString(static_cast<uint16_t>(McuCommand::SUB_CMD_GET_IMEI));
    // imei 356938035643809
    data += "33 35 36 39 33 38 30 33 35 36 34 33 38 30 39";

    // Respond to the MCU's request using the same sequence ID from the NOTIFY
    std::vector<uint8_t> raw_msg = packMessage(msg.seq,
        MsgType::SET_REQ,
        ErrorCode::NONE,
        hexStringToBytes(data));
    if (write(sock, raw_msg.data(), raw_msg.size()) < 0) {
        perror("socke write error!");
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
            // get_imei # step 4, call get_imei_handler while SUB_CMD_GET_IMEI received
            } else if (msg.msg_type == MsgType::NOTIFY
                    && SpiCommon::get_mcu_code(msg.data) == static_cast<uint16_t>(McuCommand::SUB_CMD_GET_IMEI)) {
                get_imei_handler(sock, msg);

            // notification # step 3, print hall sensor status
            } else if (msg.msg_type == MsgType::NOTIFY
                    && SpiCommon::get_mcu_code(msg.data) == static_cast<uint16_t>(McuCommand::SUB_CMD_HAL_CHANGE)) {
                if (msg.data.size() > 3) {
                    std::cout << "SUB_CMD_HAL_CHANGE "
                              << ((msg.data[3] == 0) ? "open" : "close")
                              << std::endl;
                } else {
                    std::cout << "SUB_CMD_HAL_CHANGE invalid data length" << std::endl;
                }
            
            // get_fw_info # step 5, call get_fw_info_handler while SUB_CMD_GET_FW_INFO_FROM_SOC received
            } else if (msg.msg_type == MsgType::NOTIFY
                    && SpiCommon::get_mcu_code(msg.data) == static_cast<uint16_t>(McuCommand::SUB_CMD_GET_FW_INFO_FROM_SOC)) {
                get_fw_info_handler(sock, msg);
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
