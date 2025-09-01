#include "common.h"
#include "SPIManager.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <linux/spi/spidev.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

SPIManager::SPIManager(MessageQueue<Packet>& mq)
    : spi_fd_(-1),
      device_(DEFAULT_SPI_DEV),
      msg_queue_(mq),
      epoll_fd_(-1),
      gpio_fd_(-1),
      running_(false)
{
    LOGI("SPIManager constructor");
    spi_fd_ = open(device_.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
        LOGE("open %s fail: %s", device_.c_str(), strerror(errno));
        return;
    }
    
    // Basic configuration
    uint8_t mode = SPI_MODE_0;
    uint32_t speed = SPI_SPEED;

    if (ioctl(spi_fd_, SPI_IOC_WR_MODE, &mode) < 0) {
        LOGW("set mode fail: %s", strerror(errno));
    }
    if (ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        LOGW("set speed fail: %s", strerror(errno));
    }
    
    LOGI("SPI opened: %s", device_.c_str());
    
    // 初始化 GPIO 和 epoll
    if (!initGpio()) {
        LOGE("Failed to initialize GPIO, stopping SPIManager");
        return;
    }
}

SPIManager::~SPIManager() {
    if (spi_fd_ >= 0) {
        close(spi_fd_);
        LOGI("SPI closed: %s", device_.c_str());
    }
}

void SPIManager::start() {
    running_ = true;
    spi_thread_ = std::thread(&SPIManager::run, this);
}

void SPIManager::stop() {
    running_ = false;

    if (gpio_thread_.joinable()) {
        gpio_thread_.join(); // 等待執行緒結束
    }
    if (gpio_fd_ >= 0) close(gpio_fd_);
    if (epoll_fd_ >= 0) close(epoll_fd_);
    
    msg_queue_.stop();
    if (spi_thread_.joinable()) {
        spi_thread_.join();
    }
}

void SPIManager::run() {
    LOGI("[SPI] Manager running...\n");
    State state = State::IDLE;
    std::vector<uint8_t> recv_buf(DATA_LENGTH);
    
     while (running_) {
        auto pkt_opt = msg_queue_.pop();
        if (!pkt_opt) {
            LOGE("got std::nullopt from msg_queue_pop(), stop running");
            break; // queue 停止
        }

        Packet& pkt = *pkt_opt;

        switch (pkt.source) {
            case PacketSource::IPC: {
                auto& ipc = std::get<IPCData>(pkt.payload);
                LOGI("Processing IPC packet from fd=%d, len=%zu", ipc.client_fd, ipc.data.size());

                // 分塊寫入 SPI
                size_t offset = 0;
                while (offset < ipc.data.size()) {
                    size_t chunk = std::min(DATA_LENGTH, ipc.data.size() - offset);

                    // Debug log
                    const uint8_t* chunk_ptr = ipc.data.data() + offset;
                    LOGI("Sent to slave: %s", bytesToHexString(chunk_ptr, chunk).c_str());

                    spiTransfer(ipc.data.data() + offset, nullptr, chunk);
                    offset += chunk;
                }
                break;
            }

            case PacketSource::GPIO: {
                auto& gpio = std::get<GPIOData>(pkt.payload);
                LOGI("Processing GPIO packet, level_low=%d", gpio.level_low);

                // 如果 MCU 拉低電平表示有資料要讀
                if (gpio.level_low) {
                    spiTransfer(nullptr, recv_buf.data(), recv_buf.size());
                    LOGI("Received from slave: %s", bytesToHexString(recv_buf).c_str());
                }
                break;
            }
        }
    }
}

int SPIManager::spiTransfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
    struct spi_ioc_transfer tr{};
    tr.len = len;
    tr.speed_hz = SPI_SPEED;
    tr.tx_buf = (unsigned long)tx_buf;
    tr.rx_buf = (unsigned long)rx_buf;

    std::lock_guard<std::mutex> lock(spi_mutex_);
    int ret = ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) {
        LOGE("SPI transfer failed: %s", strerror(errno));
        return -1;
    }
    return 0;
}

bool SPIManager::initGpio() {
    // 開啟 GPIO 檔案
    gpio_fd_ = open(GPIO_PATH, O_RDONLY | O_NONBLOCK);
    if (gpio_fd_ < 0) {
        LOGE("Failed to open GPIO: %s, err: %s", GPIO_PATH, strerror(errno));
        return false;
    }

    // 建立 epoll 實例
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
        LOGE("Failed to create epoll instance: %s", strerror(errno));
        close(gpio_fd_);
        return false;
    }

    // 設定 epoll 監控 GPIO 檔案描述符
    struct epoll_event ev;
    ev.events = EPOLLPRI | EPOLLERR; // 監控高優先級事件和錯誤
    ev.data.fd = gpio_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, gpio_fd_, &ev) < 0) {
        LOGE("Failed to add GPIO to epoll: %s", strerror(errno));
        close(gpio_fd_);
        close(epoll_fd_);
        return false;
    }

    // 清除初始的 GPIO 中斷狀態
    char dummy[8];
    lseek(gpio_fd_, 0, SEEK_SET);
    read(gpio_fd_, dummy, sizeof(dummy));

    // 啟動 GPIO 監聽執行緒
    gpio_thread_ = std::thread(&SPIManager::gpioMonitorThread, this);

    return true;
}

// Use the SOC wakeup PIN as an MCU interrupt to notify the SOC.
// When the MCU has data transmission needs, pull the PIN low until the transmission is complete, then pull it high.
bool SPIManager::checkGpioLevelAndNotify(bool &last_level_low) {
    char value[2];
    lseek(gpio_fd_, 0, SEEK_SET);
    ssize_t bytes_read = read(gpio_fd_, value, 1);
    if (bytes_read <= 0) {
        LOGE("Failed to read GPIO level: %s", strerror(errno));
        return last_level_low; // 讀取失敗就回傳舊值
    }

    bool new_low = (value[0] == '0');
    if (new_low != last_level_low) {
        last_level_low = new_low;

        Packet pkt;
        pkt.source = PacketSource::GPIO;
        pkt.payload = GPIOData{new_low};
        msg_queue_.push(pkt);

        LOGI("GPIO level changed to %s", new_low ? "low" : "high");
    }
    return last_level_low;
}

void SPIManager::gpioMonitorThread() {
    struct epoll_event events[1];
    bool level_low = false; // 紀錄上一次電平狀態

    while (running_) {
        int nfds = epoll_wait(epoll_fd_, events, 1, 100);
        if (!running_) break;

        if (nfds < 0) {
            LOGE("epoll_wait failed: %s", strerror(errno));
            continue;
        }

        if (nfds == 1 && (events[0].events & (EPOLLPRI | EPOLLERR))) {
            // 第一次先檢查並送事件
            checkGpioLevelAndNotify(level_low);

            // -------- Loop Polling 檢查電平變化 --------
            const int max_poll_ms = 500;
            for (int elapsed = 0; elapsed < max_poll_ms && running_; elapsed += 10) {
                usleep(10 * 1000);
                bool now_low = checkGpioLevelAndNotify(level_low);

                // 如果回到 High，可以提前結束
                if (!now_low) break;
            }
        }
    }
}

