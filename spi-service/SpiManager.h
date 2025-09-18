#pragma once
#include <atomic>
#include <thread>
#include "MessageQueue.h"
#include "Packet.h"
#include "SpiFrame.h"
#include "SeqMapper.h"

// SPI Frame format:
// DIRECTION (1 byte): MCU->SOC: 0x55, SOC->MCU: 0xAA
// SEQ_ID (2 bytes)
// CMD_ID (1 byte)
// PAYLOAD_LEN (2 bytes)
// PAYLOAD: SUB_CMD_ID (2 bytes) + Data (up to 118 bytes)
// CHECKSUM (2 bytes)

class SpiManager {
public:
    static constexpr const char* GPIO_PATH = "/sys/class/gpio/gpio502/value";
    static constexpr const char* DEFAULT_SPI_DEV = "/dev/spidev2.0";

    SpiManager(MessageQueue<Packet>& tx_mq, MessageQueue<Packet>& rx_mq, SeqMapper& seq_mapper);
    ~SpiManager();

    void run();   // Main SPI processing loop
    void start(); // Start SPI manager threads
    void stop();  // Stop SPI manager threads

private:
    static constexpr int SPI_SPEED = 1000000; // 1 MHz
    std::atomic<bool> running_;
    std::thread spi_thread_;
    int spi_fd_;
    std::string device_;
    MessageQueue<Packet>& tx_queue_;
    MessageQueue<Packet>& rx_queue_;
    SeqMapper &seq_mapper_;
    std::mutex spi_mutex_; // Mutex to protect spi_fd_
    int seq_id_{0};

    std::thread gpio_thread_; // GPIO monitoring thread
    int epoll_fd_; // epoll file descriptor
    int gpio_fd_;  // GPIO file descriptor

    int spi_transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool sync = true);
    int wait_gpio_interrupt();
    uint8_t check_gpio_level_and_notify(uint8_t &last_level);
    void gpioMonitorThread(); // GPIO monitoring thread function
    bool init_gpio();         // Initialize GPIO and epoll
    bool isSpiReadRequired(uint8_t gpio_level);
    std::optional<Packet> gen_ipc_packet(const SpiFrame& spi_frame);
    std::pair<uint16_t, SpiFrame::Command> get_spi_frame_params(const IPCData& ipc);
};
