#pragma once
#include <cstdint>
#include <mutex>
#include <linux/spi/spidev.h>
#include <optional>
#include "SpiFrame.h"

#define SPI_MODE_STR(mode) ((mode) ? "reliable" : "unreliable")

class SpiDev {
public:
    static constexpr const char* DEFAULT_SPI_DEV = "/dev/spidev2.0";
    static constexpr const bool RELIABLE = true;
    static constexpr const bool UNRELIABLE = false;

    SpiDev() : device_(DEFAULT_SPI_DEV) {}
    ~SpiDev() {
        close_dev();
    };

    bool open_dev();
    void close_dev();

    int write(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable = true);

    int read(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable = true);

    int read_async(uint8_t* rx_buf, size_t len, bool reliable = true);

private:
    static constexpr uint8_t MCU_READY_FLAG = 0x55;
    static constexpr size_t SPI_RETRY_DELAY_MS = 10;
    static constexpr size_t SPI_RETRY_COUNT = 10;

    struct FrameResult {
        SpiFrame::Command cmd;
        bool good;
    };

    int spi_fd_;
    std::string device_;
    std::mutex mtx_;

    int do_transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    int do_transfer_locked(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    int transfer_locked(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    int transfer_reliable_locked(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);

    int write1(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    int write1_reliable(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len);
    int read1(uint8_t* rx_buf, size_t len, uint16_t seq_id, bool reliable);
    int send_ack(uint16_t seq, uint8_t status);
    FrameResult verify_rx_frame(const uint8_t* rx_buf, size_t len, uint16_t expected_seq_id);
};

//Reliable mode:
//  [Read MCU]: If the read data does not begin with the MCU_READY_FLAG, reread. Retry up to SPI_RETRY_COUNT times.
//  [Receive X]: Execute [Read MCU] and check the CHECKSUM; check if it is Frame X. If no specific X is specified, skip this check. Check if the SEQ is correct.
//  [Write X]: Write Frame X. If no specific X is specified, it is normal data. (When X != ACK AND X != NACK [Receive ACK])
//
//  SoC Writing to MCU:
//     1. [Write] (If failure, skip to STEP 1. Retry up to SPI_RETRY_COUNT times)
//  SoC Reading from MCU:
//     1. [Write] (If failure, skip to STEP 1. Retry up to SPI_RETRY_COUNT times)
//     2. [Receive RESPONSE] (If correct, [Write ACK]; if error, [Write NACK], then skip to STEP 1. Retry up to SPI_RETRY_COUNT times)
//  MCU writes to the Soc:
//     1. Detects GPIO going LOW
//     2. [Write CMD_SERVICE_MCU_REQUEST] (If failed, jump to STEP 2, maximum SPI_RETRY_COUNT times)
//     3. [Receive] (If correct, [Write ACK]. If error, [Write NACK], then jump to STEP 3.)
//     4. Detects GPIO going LOW, terminates the process
//  MCU reads from the Soc:
//     1. Detects GPIO going LOW
//     2. [Write CMD_SERVICE_MCU_REQUEST] (If failed, jump to STEP 2, maximum SPI_RETRY_COUNT times)
//     3. [Receive READ] (If correct, [Write ACK]. If error, [Write NACK], then jump to STEP 3)
//     4. [Write RESPONSE], if no ACK is received, jump to STEP 3
//     5. Detects GPIO going LOW, End the process.
//
// Unreliable mode: Do not perform the actions in brackets.