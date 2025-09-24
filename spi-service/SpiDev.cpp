#include "SpiDev.h"
#include "SpiCommon.h"
#include "common.h"
#include "SpiFrame.h"
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

#include <linux/spi/spidev.h>

#undef LOG_TAG
#define LOG_TAG "spi-service/SpiDev"

using Command = SpiFrame::Command;
using Direction = SpiFrame::Direction;

bool SpiDev::open_dev() {
    spi_fd_ = open(device_.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
        LOGE("open %s fail: %s", device_.c_str(), strerror(errno));
        return false;
    }

    uint8_t mode = SPI_MODE_0;
    uint32_t speed = SPI_SPEED;

    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
        LOGW("set mode fail: %s", strerror(errno));
    }
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        LOGW("set speed fail: %s", strerror(errno));
    }

    LOGI("SPI opened: %s (speed=%u)", device_.c_str(), speed);
    return true;
}

void SpiDev::close_dev() {
    if (spi_fd_ >= 0) {
        close(spi_fd_);
        spi_fd_ = -1;
        LOGI("SPI closed: %s", device_.c_str());
    }
}

int SpiDev::do_transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    struct spi_ioc_transfer tr{};
    tr.len = len;
    tr.speed_hz = SPI_SPEED;
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_buf;

    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        LOGE("do_transfer: %s", strerror(errno));
        return -1;
    }
    return 0;
}

int SpiDev::transfer_with_retry(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, int retry_count, bool check_flag) {
    for (int i = 0; i < retry_count; ++i) {
        if (do_transfer(tx_buf, rx_buf, len) == 0) {
            if (check_flag) {
                if (rx_buf && rx_buf[0] == MCU_READY_FLAG) {
                    return 0;
                }
            } else {
                return 0;
            }
        }
        usleep(SPI_RETRY_DELAY_MS * 1000);
    }
    LOGE("transfer_with_retry, return -1!");
    return -1;
}

int SpiDev::send_ack(uint16_t seq, uint8_t status) {
    int result;

    SpiFrame spi_frame = SpiFrame(Direction::SOC2MCU, seq,
                            Command::ACK_STATUS, std::vector<uint8_t>{status});
    std::vector<uint8_t> bytes = spi_frame.toBytes();
    result = write(bytes.data(), bytes.size(), UNRELIABLE);
    LOGI("send_ack, seq=%u, status=%u, write return %d", seq, status, result);
    return result;
}

int SpiDev::write1(const uint8_t* tx_buf, size_t len, bool reliable) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<uint8_t> rx_buf(SpiCommon::MAX_SPI_FRAME_SIZE);

    int retry_count = reliable ? SPI_RETRY_COUNT : 1;

    // Reliable mode: if the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    // Unreliable mode: transfer data once, no need to check MCU_READY_FLAG
    if (transfer_with_retry(tx_buf, rx_buf.data(), len, retry_count, reliable) < 0) {
        LOGE("write, transfer_with_retry failed!");
        return -1;
    }

    // ACK checked only in reliable mode.
    if (reliable) {
        if (!got_ack(rx_buf.data(), len, SpiFrame::get_seq_id(tx_buf, len))) {
            LOGE("write, got_ack failed!");
            return -1;
        }
    }
    return 0;
}

int SpiDev::write(const uint8_t* tx_buf, size_t len, bool reliable) {
    int result;
    int retry_count = reliable ? SPI_RETRY_COUNT : 1;

    int i;
    for (i = 0; i < retry_count; i++) {
        if (write1(tx_buf, len, reliable) == 0) break;
    }

    result = (i < retry_count) ? 0 : -1;
    LOGI("write, return %d", result);
    return result;
}

int SpiDev::read1(uint8_t* rx_buf, size_t len, uint16_t seq_id, bool reliable) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<uint8_t> tx_buf(SpiCommon::MAX_SPI_FRAME_SIZE);

    // If the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    if (transfer_with_retry(tx_buf.data(), rx_buf, len, SPI_RETRY_COUNT, RELIABLE) < 0) {
        LOGE("read, transfer_with_retry failed!");
        return -1;
    }

    // RESPONSE checked and read restarted only in reliable mode.
    if (reliable) {
        if (got_response(rx_buf, len, seq_id)) {
            LOGI("read1, got_reponse");
            send_ack(seq_id, SpiFrame::MCU_ACK_SUCCESS);
        } else {
            LOGE("read1, got_reponse failed");
            send_ack(seq_id, SpiFrame::MCU_ACK_FAIL);
        }
        return -1;
    }
    return 0;
}

int SpiDev::read(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable) {
    int result;
    int retry_count = reliable ? SPI_RETRY_COUNT : 1;
    uint16_t seq_id = SpiFrame::get_seq_id(tx_buf, len);

    int i;
    for (i = 0; i < retry_count; i++) {
        if (write(tx_buf, len, reliable) != 0) {
            LOGE("read, write failed");
            return -1;
        }
        if (read1(rx_buf, len, seq_id, reliable) == 0) break;
    }

    result = (i < retry_count) ? 0 : -1;
    LOGI("read, return %d", result);
    return result;
}

// this function applies to MCU-initiated read and write operations.
int SpiDev::read_async(uint8_t* rx_buf, size_t len, bool reliable) {
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<uint8_t> tx_buf(SpiCommon::MAX_SPI_FRAME_SIZE);

    // If the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    if (transfer_with_retry(tx_buf.data(), rx_buf, len, SPI_RETRY_COUNT, RELIABLE) < 0) {
        LOGE("read_async, transfer_with_retry failed!");
        return -1;
    }

    // checksum
    if (reliable) {
        auto spi_frame_opt  = SpiFrame::fromBytes(rx_buf, len);
        if (spi_frame_opt) {
            send_ack(spi_frame_opt->seq_id_, SpiFrame::MCU_ACK_SUCCESS);
        } else {
            LOGE("read_async, invalid frame!");
            send_ack(0, SpiFrame::MCU_ACK_FAIL);
            return -1;
        }
    }

    return 0;
}

bool SpiDev::got_ack(const uint8_t* rx_buf, size_t len, uint16_t expected_seq_id) {
    auto spi_frame_opt = SpiFrame::fromBytes(rx_buf, len);
    if (!spi_frame_opt) {
        LOGE("got_ack, invalid frame %s", SpiCommon::bytesToHexString(rx_buf, len));
        return false;
    }

    const SpiFrame& spi_frame = *spi_frame_opt;
    if (spi_frame.cmd_id_ != Command::ACK_STATUS) {
        LOGE("got_ack, not ACK frame");
        return false;
    }

    if (spi_frame.seq_id_ != expected_seq_id) {
        LOGE("got_ack, seq_id mismatch exp=%u got=%u", expected_seq_id, spi_frame.seq_id_);
        return false;
    }

    if (!spi_frame.is_ack_success()) {
        LOGE("got_ack, is_ack_success failed!");
        return false;
    }

    return true;
}

bool SpiDev::got_response(const uint8_t* rx_buf, size_t len, uint16_t expected_seq_id) {
    auto spi_frame_opt = SpiFrame::fromBytes(rx_buf, len);
    if (!spi_frame_opt) {
        LOGE("got_response, invalid frame %s", SpiCommon::bytesToHexString(rx_buf, len));
        return false;
    }

    const SpiFrame& spi_frame = *spi_frame_opt;
    if (spi_frame.cmd_id_ != Command::RESPONSE) {
        LOGE("got_response, not response frame");
        return false;
    }

    if (spi_frame.seq_id_ != expected_seq_id) {
        LOGE("got_response, seq_id mismatch exp=%u got=%u", expected_seq_id, spi_frame.seq_id_);
        return false;
    }

    return true;
}