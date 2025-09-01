#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>
#include <sstream>

// IPC Socket 路徑
static constexpr const char* IPC_SOCKET_PATH = "/tmp/spi-service/ipc.sock";

// 將 HEX 字串轉成 binary
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

// 根據 command 返回預設 HEX string
std::string getCommandHexString(int cmd) {
    switch(cmd) {
        case 1: return "AA 01 00 00 03 00 01 00 01 00";
        case 2: return "AA BB CC DD EE";
        case 3: return "10 20 30";
        default: return "00"; // default
    }
}

int main() {
    // 建立 UNIX Domain Socket
    int sock = socket(AF_UNIX, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, IPC_SOCKET_PATH, sizeof(addr.sun_path) - 1);

    if (connect(sock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        perror("connect");
        close(sock);
        return 1;
    }

    std::cout << "Connected to SPI service. Enter command number (0 to exit).\n";

    while (true) {
        std::cout << "Command> ";
        int cmd;
        std::cin >> cmd;
        if (!std::cin) break; // invalid input
        if (cmd == 0) break;

        std::string hexStr = getCommandHexString(cmd);
        std::vector<uint8_t> data = hexStringToBytes(hexStr);

        // 傳送 binary data
        if (write(sock, data.data(), data.size()) < 0) {
            perror("write");
            break;
        }

        std::cout << "Sent command " << cmd << ": " << hexStr << std::endl;

        // 接收回覆
        char buf[1024];
        ssize_t n = read(sock, buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            std::cout << "Received: " << buf << std::endl;
        } else {
            std::cout << "No response or connection closed." << std::endl;
        }
    }
    
    std::cout << "Exiting cmd_test.\n";
    close(sock);
    return 0;
}
