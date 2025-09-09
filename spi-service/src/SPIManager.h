#pragma once
#include <atomic>
#include <thread>
#include "MessageQueue.h"
#include "Packet.h"
#include "SpiFrame.h"
#include "SeqMapper.h"

#define SIMULATE_GPIO_BEHAVIOR 1
// DIRECTION(1): MCU to SOC: 0x55, SOC to MCU: 0xAA
// SEQ_ID(2)
// CMD_ID(1)
// PAYLOAD_LEN(2)
// PAYLOAD: SUB_CMD_ID(2) + Data(118)
// CHECKSUM(2)
class SpiManager {
public:
    static constexpr const char* GPIO_PATH = "/sys/class/gpio/gpio502/value";
    static constexpr const char* DEFAULT_SPI_DEV = "/dev/spidev2.0";
    static constexpr size_t MAX_PAYLOAD = 120;

    SpiManager(MessageQueue<Packet>& tx_mq, MessageQueue<Packet>& rx_mq, SeqMapper& seq_mapper);
    ~SpiManager();
    void run();
    void start();
    void stop();

private:
    static constexpr int SPI_SPEED = 1000000; // 1 MHz
    std::atomic<bool> running_;
    std::thread spi_thread_;
    int spi_fd_;
    std::string device_;
    MessageQueue<Packet>& tx_queue_;
    MessageQueue<Packet>& rx_queue_;
    SeqMapper &seq_mapper_;
    std::mutex spi_mutex_; // 保護 spi_fd_ 的 mutex
    int seq_id_{0};
    
    std::thread gpio_thread_; // GPIO 監聽執行緒
    int epoll_fd_; // epoll 檔案描述符
    int gpio_fd_;  // GPIO 檔案描述符
    
    int spi_transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    int wait_gpio_interrupt();
    
    uint8_t check_gpio_level_and_notify(uint8_t &last_level);
    void gpioMonitorThread(); // GPIO 監聽執行緒函數
    bool init_gpio(); // 初始化 GPIO 和 epoll
    bool isSpiReadRequired(uint8_t gpio_level);
    std::optional<Packet> gen_ipc_packet(const SpiFrame& spi_frame);
    
#ifdef SIMULATE_GPIO_BEHAVIOR
    // simulate gpio
    static constexpr const char* MOCK_GPIO_PATH = "/tmp/gpio502";
    int inotify_fd_;
    int watch_fd_;
    int gpio_fd2_;  // GPIO 檔案描述符
    std::thread gpio_thread2_; // GPIO 監聽執行緒
    bool init_mock_gpio();
    uint8_t check_mock_gpio_level_and_notify(uint8_t &last_level);
    void mockGpioThread();
    SpiFrame makeTestSpiFrame(uint16_t seq_id);
#endif
};
