#include "common.h"
#include "MessageQueue.h"
#include "Packet.h"
#include "IPCManager.h"
#include "SPIManager.h"
#include <thread>

int main() {
    MessageQueue<Packet> msg_queue;

    IPCManager ipc(msg_queue);
    SPIManager spi(msg_queue);

    try {
        ipc.init_socket();
    } catch (const std::runtime_error& e) {
        LOGE("[Main] IPCManager initialization failed: %s", e.what());
        return 1;
    } catch (...) {
        LOGE("[Main] Unknown error occurred");
        return 1;
    }
    
    ipc.start();
    spi.start();

    // 主執行緒
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 通知 IPCManager / SPIManager 停止
    ipc.stop(); 
    spi.stop();

    return 0;
}
