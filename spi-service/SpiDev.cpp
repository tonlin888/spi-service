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

int SpiDev::transfer_locked(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    std::lock_guard<std::mutex> lock(mtx_);
    return do_transfer(tx_buf, rx_buf, len);
}

int SpiDev::transfer_reliable_locked(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    for (int i = 0; i < SPI_RETRY_COUNT; ++i) {
        bool transfer_ok = false;

        {
            std::lock_guard<std::mutex> lock(mtx_);
            transfer_ok = (do_transfer(tx_buf, rx_buf, len) == 0);
        }

        if (transfer_ok && rx_buf && rx_buf[0] == MCU_READY_FLAG) {
            return 0; // Transfer succeeded, MCU is ready
        }

        // Transfer failed or MCU not ready, wait before retrying
        usleep(SPI_RETRY_DELAY_MS * 1000);
    }

    LOGE("transfer_reliable_locked failed!");
    return -1;
}

int SpiDev::write1(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    LOGI("write1");
    std::vector<uint8_t> dummy(SpiCommon::MAX_SPI_FRAME_SIZE);
    uint8_t* rx_ptr = (rx_buf != nullptr) ? rx_buf : dummy.data();

    // Unreliable mode: transfer data once, no need to check MCU_READY_FLAG
    if (transfer_locked(tx_buf, rx_buf, len) != 0) {
        LOGE("write1, transfer_locked failed!");
        return -1;
    }
    return 0;
}

int SpiDev::write1_reliable(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    LOGI("write1_reliable, begin");
    std::vector<uint8_t> dummy(SpiCommon::MAX_SPI_FRAME_SIZE);
    uint8_t* rx_ptr = (rx_buf != nullptr) ? rx_buf : dummy.data();

    // if the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    if (transfer_reliable_locked(tx_buf, rx_ptr, len) != 0) {
        LOGE("write1_reliable, transfer_reliable_locked failed!");
        return -1;
    }

    // check ACK / RESPONSE
    uint16_t seq_id = SpiFrame::get_seq_id(tx_buf, len);
    FrameResult res = verify_rx_frame(rx_ptr, len, seq_id);

    // rx_buf != nullptr while we are in reading sequence
    if (rx_buf != nullptr && res.cmd == Command::RESPONSE) {
        if (res.good) {
           LOGI("write1_reliable, got RESPONSE");
           send_ack(seq_id, SpiFrame::MCU_ACK_SUCCESS);
        } else {
           LOGE("write1_reliable, got RESPONSE failed!");
           send_ack(seq_id, SpiFrame::MCU_ACK_FAIL);
           return -1;

        }
    } else if (!(res.cmd == Command::ACK_STATUS && res.good))  {
        LOGI("write1_reliable, got ACK failed on attempt %d! %s good=%d", SpiFrame::commandToStr(res.cmd).c_str(), res.good);
        return -1;
    }
    LOGE("write1_reliable, return 0");
    return 0;
}

int SpiDev::read1(uint8_t* rx_buf, size_t len, uint16_t seq_id, bool reliable) {
    LOGI("read1");
    std::vector<uint8_t> dummy(SpiCommon::MAX_SPI_FRAME_SIZE);

    // If the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    if (transfer_reliable_locked(dummy.data(), rx_buf, len) < 0) {
        LOGE("read1, transfer_reliable_locked failed!");
        return -1;
    }

    // RESPONSE checked and read restarted only in reliable mode.
    if (reliable) {
        FrameResult res = verify_rx_frame(rx_buf, len, seq_id);
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

SpiDev::FrameResult SpiDev::verify_rx_frame(const uint8_t* rx_buf, size_t len, uint16_t expected_seq_id) {
    Command cmd = SpiFrame::get_cmd_id(rx_buf, len);
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("verify_rx_frame, %s rx_buf's len %u != %u!", SpiFrame::commandToStr(cmd).c_str(), len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return {cmd, false};
    }

    auto spi_frame_opt = SpiFrame::fromBytes(rx_buf, len);
    if (!spi_frame_opt) {
        LOGE("verify_rx_frame, %s invalid frame %s", SpiFrame::commandToStr(cmd).c_str(),
            SpiCommon::bytesToShortHexString(rx_buf, len).c_str());
        return {cmd, false};
    }

    const SpiFrame& spi_frame = *spi_frame_opt;

    // verify seq_id
    if (spi_frame.seq_id_ != expected_seq_id) {
        LOGE("verify_rx_frame, %s seq_id mismatch exp=%u got=%u",
            SpiFrame::commandToStr(cmd).c_str(), expected_seq_id, spi_frame.seq_id_);
        return {cmd, false};
    }

    switch (spi_frame.cmd_id_) {
        case Command::ACK_STATUS:
            if (!spi_frame.is_ack_success()) {
                LOGE("verify_rx_frame, Negative ACK!");
                return {cmd, false};
            }
            LOGI("verify_rx_frame, ACK");
            return {cmd, true};

        case Command::RESPONSE:
            LOGI("verify_rx_frame, RESPONSE, len=%u", spi_frame.payload_len_);
            return {cmd, true};

        default:
            LOGE("verify_rx_frame, %s unsupported!", SpiFrame::commandToStr(spi_frame.cmd_id_).c_str());
            return {cmd, false};
    }
}

// ===== public SpiDev interface =====
int SpiDev::write(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable) {
    LOGI("%s write, begin", SPI_MODE_STR(reliable));
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("%s write, buf's len %u != %u!", SPI_MODE_STR(reliable), len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return -1;
    }

    int result = false;
    if (reliable) {
        for (int attempt = 0; attempt < SPI_RETRY_COUNT; attempt++) {
            LOGI("%s write, write1_reliable on attempt %d", SPI_MODE_STR(reliable), attempt);
            if ((result = write1_reliable(tx_buf, rx_buf, len)) == 0) break;
        }
    } else {
        result = write1(tx_buf, rx_buf, len);
    }
    LOGI("%s write, return %d", SPI_MODE_STR(reliable), result);
    return result;
}

int SpiDev::read(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len, bool reliable) {
    LOGI("%s read, begin", SPI_MODE_STR(reliable));
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("%s read, buf's len %u != %u!", SPI_MODE_STR(reliable), len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return -1;
    }

    if (reliable) {
        if (write(tx_buf, rx_buf, len, RELIABLE) != 0) {
            LOGE("%s read, write failed!", SPI_MODE_STR(reliable));
            return -1;
        }
    } else {
        write1(tx_buf, rx_buf, len);
        uint16_t seq_id = SpiFrame::get_seq_id(tx_buf, len);
        if ((read1(rx_buf, len, seq_id, UNRELIABLE) != 0)) {
            LOGE("%s read, read1 failed!", SPI_MODE_STR(reliable));
            return -1;
        }
    }

    LOGI("%s read, return 0", SPI_MODE_STR(reliable));
    return 0;
}

// this function applies to MCU-initiated read or write operations.
int SpiDev::read_async(uint8_t* rx_buf, size_t len, bool reliable) {
    LOGI("%s read_async, begin", SPI_MODE_STR(reliable));
    if (len != SpiCommon::MAX_SPI_FRAME_SIZE) {
        LOGE("%s read_async, buf's len %u != %u!", SPI_MODE_STR(reliable), len, SpiCommon::MAX_SPI_FRAME_SIZE);
        return -1;
    }

    std::vector<uint8_t> dummy(SpiCommon::MAX_SPI_FRAME_SIZE);
    // If the beginning of the received data is not MCU_READY_FLAG, retry reading up to SPI_RETRY_COUNT times.
    if (transfer_reliable_locked(dummy.data(), rx_buf, len) < 0) {
        LOGE("%s read_async, transfer_reliable_locked failed!", SPI_MODE_STR(reliable));
        return -1;
    }

    // checksum
    if (reliable) {
        auto spi_frame_opt  = SpiFrame::fromBytes(rx_buf, len);
        if (spi_frame_opt) {
            send_ack(spi_frame_opt->seq_id_, SpiFrame::MCU_ACK_SUCCESS);
        } else {
            LOGE("%s read_async, invalid frame!", SPI_MODE_STR(reliable));
            send_ack(0, SpiFrame::MCU_ACK_FAIL);
            return -1;
        }
    }

    LOGI("%s read_async, return 0", SPI_MODE_STR(reliable));
    return 0;
}
