#pragma once
#include <atomic>
#include <thread>
#include "MessageQueue.h"
#include "Packet.h"

class SPIManager {
public:
    static constexpr const char* GPIO_PATH = "/sys/class/gpio/gpio502/value";
    static constexpr const char* DEFAULT_SPI_DEV = "/dev/spidev2.0";
    static constexpr size_t DATA_LENGTH = 128;

    SPIManager(MessageQueue<Packet>& mq);
    ~SPIManager();
    void run();
    void start();
    void stop();

private:
    enum class State { IDLE, WRITE, READ };
    static constexpr int SPI_SPEED = 1000000; // 1 MHz
    std::atomic<bool> running_;
    std::thread spi_thread_;
    int spi_fd_;
    std::string device_;
    MessageQueue<Packet>& msg_queue_;
    std::mutex spi_mutex_; // 保護 spi_fd_ 的 mutex
    
    int spiTransfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    int waitGpioInterrupt();
    
    std::thread gpio_thread_; // GPIO 監聽執行緒
    int epoll_fd_; // epoll 檔案描述符
    int gpio_fd_;  // GPIO 檔案描述符
    
    bool checkGpioLevelAndNotify(bool &last_level_low);
    void gpioMonitorThread(); // GPIO 監聽執行緒函數
    bool initGpio(); // 初始化 GPIO 和 epoll
};
