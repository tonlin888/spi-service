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
    uint32_t speed = SpiCommon::SPI_SPEED;

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
    tr.speed_hz = SpiCommon::SPI_SPEED;
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_buf;

    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    
    LOGI("do_transfer, TX: %s", SpiCommon::bytesToShortHexString(tx_buf, len).c_str());
    LOGI("do_transfer, RX: %s", SpiCommon::bytesToShortHexString(rx_buf, len).c_str());
    
    if (ret < 0) {
        LOGE("do_transfer: %s", strerror(errno));
        return -1;
    }

    return 0;
}

int SpiDev::transfer_with_retry(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, int retry_count, bool check_flag) {
    for (int i = 0; i < retry_count; ++i) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (do_transfer(tx_buf, rx_buf, len) == 0) {
                if (check_flag) {
                    if (rx_buf && rx_buf[0] == MCU_READY_FLAG) {
                        return 0;
                    }
                } else {
                    return 0;
                }
            }
        }
        usleep(SPI_RETRY_DELAY_MS * 1000);
    }
    LOGE("transfer_with_retry, return -1!");
    return -1;
}

int SpiDev::send_ack(uint16_t seq, uint8_t status) {
    int result;

    LOGI("send_ack");
    SpiFrame spi_frame = SpiFrame(Direction::SOC2MCU, seq,
                            Command::ACK_STATUS, std::vector<uint8_t>{status});
    std::vector<uint8_t> bytes = spi_frame.toBytes();
    result = write(bytes.data(), nullptr, bytes.size(), UNRELIABLE);
    LOGI("send_ack, seq=%u, status=%u, write return %d", seq, status, result);
    return result;
}

int SpiDev::write1(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable) {
    LOGI("write1");
    std::vector<uint8_t> dummy(SpiCommon::MAX_SPI_FRAME_SIZE);

    int retry_count = reliable ? SPI_RETRY_COUNT : 1;
    uint8_t* rx_ptr = (rx_buf != nullptr) ? rx_buf : dummy.data();

    // Reliable mode: if the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    // Unreliable mode: transfer data once, no need to check MCU_READY_FLAG
    if (transfer_with_retry(tx_buf, rx_ptr, len, retry_count, reliable) < 0) {
        LOGE("write1, transfer_with_retry failed!");
        return -1;
    }

    // ACK checked in reliable mode.
    // ACK/RESPONSE checked in reliable read.
    if (reliable) {
        uint16_t seq_id = SpiFrame::get_seq_id(tx_buf, len);
        FrameResult res = got_frame(rx_ptr, len, seq_id);

        if (rx_buf != nullptr && res.cmd == Command::RESPONSE) {
            if (res.good) {
               LOGI("write1, got_reponse");
               send_ack(seq_id, SpiFrame::MCU_ACK_SUCCESS);
            } else {
               LOGE("write1, got_reponse failed");
               send_ack(seq_id, SpiFrame::MCU_ACK_FAIL);
               return -1;
            }
        } else if (!(res.cmd == Command::ACK_STATUS && res.good)) {
            LOGE("write1, got ACK failed! cmd=%u, good=%d", static_cast<uint8_t>(res.cmd), res.good);
            return -1;
        }
    }
    return 0;
}

int SpiDev::read1(uint8_t* rx_buf, size_t len, uint16_t seq_id, bool reliable) {
    LOGI("read1");
    std::vector<uint8_t> dummy(SpiCommon::MAX_SPI_FRAME_SIZE);

    // If the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    if (transfer_with_retry(dummy.data(), rx_buf, len, SPI_RETRY_COUNT, RELIABLE) < 0) {
        LOGE("read1, transfer_with_retry failed!");
        return -1;
    }

    // RESPONSE checked and read restarted only in reliable mode.
    if (reliable) {
        FrameResult res = got_frame(rx_buf, len, seq_id);
        if (res.good && res.cmd == Command::RESPONSE) {
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

SpiDev::FrameResult SpiDev::got_frame(const uint8_t* rx_buf, size_t len, uint16_t expected_seq_id) {
    Command cmd = SpiFrame::get_cmd_id(rx_buf, len);
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("handle_spi_frame, rx_buf's len %u != %u!", len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return {cmd, false};
    }

    auto spi_frame_opt = SpiFrame::fromBytes(rx_buf, len);
    if (!spi_frame_opt) {
        LOGE("handle_spi_frame, invalid frame %u, %s", static_cast<uint8_t>(cmd),
            SpiCommon::bytesToShortHexString(rx_buf, len).c_str());
        return {cmd, false};
    }

    const SpiFrame& spi_frame = *spi_frame_opt;

    // verify seq_id
    if (spi_frame.seq_id_ != expected_seq_id) {
        LOGE("handle_spi_frame, %u seq_id mismatch exp=%u got=%u",
             static_cast<uint8_t>(cmd), expected_seq_id, spi_frame.seq_id_);
        return {cmd, false};
    }

    switch (spi_frame.cmd_id_) {
        case Command::ACK_STATUS:
            if (!spi_frame.is_ack_success()) {
                LOGE("handle_spi_frame, got NACK!");
                return {cmd, false};
            }
            return {cmd, true};

        case Command::RESPONSE:
            LOGI("handle_spi_frame, got RESPONSE, len=%u", spi_frame.payload_len_);
            return {cmd, true};

        default:
            LOGE("handle_spi_frame, unsupported cmd_id=%u", spi_frame.cmd_id_);
            return {cmd, false};
    }
}

// ===== public SpiDev interface =====
int SpiDev::write(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable) {
    LOGI("%s write, begin", (reliable ? "reliable" : "unreliable"));
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("write, tx_buf's len %u != %u!", len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return -1;
    }

    int result;
    int retry_count = reliable ? SPI_RETRY_COUNT : 1;

    int i;
    for (i = 0; i < retry_count; i++) {
        if (write1(tx_buf, rx_buf, len, reliable) == 0) break;
    }

    result = (i < retry_count) ? 0 : -1;
    LOGI("%s write, return %d", (reliable ? "reliable" : "unreliable"), result);
    return result;
}

int SpiDev::read(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable) {
    LOGI("%s read, begin", (reliable ? "reliable" : "unreliable"));
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("read, tx_buf or rx_buf's len %u != %u!", len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return -1;
    }

    int result;
    int retry_count = reliable ? SPI_RETRY_COUNT : 1;
    uint16_t seq_id = SpiFrame::get_seq_id(tx_buf, len);

    int i;
    for (i = 0; i < retry_count; i++) {
        if (write(tx_buf, rx_buf, len, reliable) != 0) {
            LOGE("read, write failed!");
            continue;
        }
        if (!reliable && (read1(rx_buf, len, seq_id, UNRELIABLE) != 0)) {
            LOGE("read, read failed!");
        } else {
            break;
        }
    }

    result = (i < retry_count) ? 0 : -1;
    LOGI("%s read, return %d", (reliable ? "reliable" : "unreliable"), result);
    return result;
}

// this function applies to MCU-initiated read and write operations.
int SpiDev::read_async(uint8_t* rx_buf, size_t len, bool reliable) {
    LOGI("%s read_async, begin", (reliable ? "reliable" : "unreliable"));
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("read_async, rx_buf's len %u != %u!", len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return -1;
    }
    
    std::vector<uint8_t> dummy(SpiCommon::MAX_SPI_FRAME_SIZE);
    // If the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    if (transfer_with_retry(dummy.data(), rx_buf, len, SPI_RETRY_COUNT, RELIABLE) < 0) {
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

    LOGI("%s read_async, return 0", (reliable ? "reliable" : "unreliable"));
    return 0;
}
