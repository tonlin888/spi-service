#pragma once
#include <atomic>
#include <thread>
#include "MessageQueue.h"
#include "Packet.h"
#include "SpiFrame.h"
#include "SpiDev.h"
#include "SeqMapper.h"

// SPI Frame format:
// DIRECTION (1 byte): MCU->SOC: 0x55, SOC->MCU: 0xAA
// SEQ_ID (2 bytes)
// CMD_ID (1 byte)
// PAYLOAD_LEN (2 bytes)
// PAYLOAD: SUB_CMD_ID (2 bytes) + Data (zero padding up to MAX_SPI_FRAME_SIZE bytes)
// CHECKSUM (2 bytes)

class SpiManager {
public:
    static constexpr const char* GPIO_PATH = "/sys/class/gpio/gpio502/value";

    SpiManager(MessageQueue<Packet>& tx_mq, MessageQueue<Packet>& rx_mq, SeqMapper& seq_mapper);
    ~SpiManager();

    void run();   // Main SPI processing loop
    void start(); // Start SPI manager threads
    void stop();  // Stop SPI manager threads

private:
    std::atomic<bool> running_;
    std::thread spi_thread_;
    MessageQueue<Packet>& tx_queue_;
    MessageQueue<Packet>& rx_queue_;
    SeqMapper &seq_mapper_;
    int seq_id_{0};

    SpiDev spi_dev_;

    std::thread gpio_thread_; // GPIO monitoring thread
    int epoll_fd_; // epoll file descriptor
    int gpio_fd_;  // GPIO file descriptor

    int wait_gpio_interrupt();
    uint8_t check_gpio_level_and_notify(uint8_t &last_level);
    void gpioMonitorThread(); // GPIO monitoring thread function
    bool init_gpio();         // Initialize GPIO and epoll

    std::optional<Packet> gen_ipc_packet(const SpiFrame& spi_frame);
    std::optional<SpiFrame> gen_spi_frame(const IPCData& ipc);
    bool send_spi_frame(const SpiFrame& frame, std::vector<uint8_t>& recv_buf);
    std::optional<Packet> parse_spi_response(const std::vector<uint8_t>& recv_buf);
};
