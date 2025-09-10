#include "common.h"
#include "MessageQueue.h"
#include "Packet.h"
#include "IpcManager.h"
#include "SpiManager.h"
#include "SeqMapper.h"
#include <thread>

int main() {
    MessageQueue<Packet> tx_queue;
    MessageQueue<Packet> rx_queue;
    SeqMapper seq_mapper;

    IpcManager ipc(tx_queue, rx_queue, seq_mapper);
    SpiManager spi(tx_queue, rx_queue, seq_mapper);

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

    // main thread
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Stop IpcManager / SpiManager
    ipc.stop();
    spi.stop();

    return 0;
}
