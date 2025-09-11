#include "common.h"
#include "SpiManager.h"
#include "SpiFrame.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>

#ifdef SIMULATE_GPIO_BEHAVIOR
#include <sys/inotify.h> // simulate gpio
#endif

#include <linux/spi/spidev.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

#undef LOG_TAG
#define LOG_TAG "spi-service/SpiManager"

using Direction = SpiFrame::Direction;
using Command = SpiFrame::Command;

SpiManager::SpiManager(MessageQueue<Packet>& tx_mq, MessageQueue<Packet>& rx_mq, SeqMapper& seq_mapper)
    : spi_fd_(-1),
      device_(DEFAULT_SPI_DEV),
      tx_queue_(tx_mq),
      rx_queue_(rx_mq),
      seq_mapper_(seq_mapper),
      epoll_fd_(-1),
      gpio_fd_(-1),
      running_(false)
{
    LOGI("SpiManager constructor");
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

    // initialize GPIO and epoll
    if (!init_gpio()) {
        LOGE("Failed to initialize GPIO, stopping SpiManager");
        return;
    }

#ifdef SIMULATE_GPIO_BEHAVIOR
    if (!init_mock_gpio()) {
        LOGE("Failed to initialize MOCK GPIO, stopping SpiManager");
        return;
    }
#endif
}

SpiManager::~SpiManager() {
    if (spi_fd_ >= 0) {
        close(spi_fd_);
        LOGI("SPI closed: %s", device_.c_str());
    }
}

void SpiManager::start() {
    running_ = true;
    spi_thread_ = std::thread(&SpiManager::run, this);
}

void SpiManager::stop() {
    running_ = false;

    if (gpio_thread_.joinable()) {
        gpio_thread_.join(); // wait for thread to finish
    }
    if (gpio_fd_ >= 0) close(gpio_fd_);
    if (epoll_fd_ >= 0) close(epoll_fd_);

    tx_queue_.stop();
    if (spi_thread_.joinable()) {
        spi_thread_.join();
    }
}

void SpiManager::run() {
    LOGI("Manager running...\n");
    std::vector<uint8_t> recv_buf(MAX_PAYLOAD);

     while (running_) {
        auto pkt_opt = tx_queue_.pop();
        if (!pkt_opt) {
            LOGE("got std::nullopt from tx_queue_pop()");
            break; // queue stopped
        }

        Packet& pkt_tx = *pkt_opt;

        switch (pkt_tx.source) {
            case PacketSource::IPC: {
                auto& ipc = std::get<IPCData>(pkt_tx.payload);
                LOGI("Processing IPC packet, %s", ipc.toString().c_str());

                size_t offset = 0;
                uint16_t mcu_cmd = static_cast<uint16_t>(ipc.data[0]) | (static_cast<uint16_t>(ipc.data[1]) << 8);
                uint16_t seq_id = seq_id_;
                Command cmd = Command::WRITE_UNREL;

                if (ipc.flow == MessageFlow::EXECUTE) {
                    LOGI("MessageFlow::EXECUTE");
                    seq_id = seq_mapper_.add_mapping(ipc.seq, ipc.client_fd, static_cast<SpiCommon::McuCommand>(mcu_cmd));
                    cmd = Command::READ_UNREL;
                } else if (ipc.flow == MessageFlow::SET) {
                    SeqMapper::Entry entry;
                    seq_mapper_.find_mapping(ipc.seq, entry);
                    LOGI("MessageFlow::SET, mcu_cmd=%u, client_fd=%d, cmd=%u", mcu_cmd, entry.client_fd, static_cast<uint16_t>(entry.cmd));
                    if (mcu_cmd == static_cast<uint16_t>(entry.cmd)) {
                        // Reply to MCU read using MCU's sequence ID
                        seq_id = entry.seq;
                        cmd = Command::RESPONSE;
                        seq_mapper_.remove_mapping(ipc.seq);
                    }
                }

                while (offset < ipc.data.size()) {
                    SpiFrame spi_frame = SpiFrame(Direction::SOC2MCU, seq_id, cmd, ipc.data, offset);
                    if (!spi_frame.is_valid()) {
                        LOGI("Invalid frame at offset %zu, data size=%zu", offset, ipc.data.size());
                        offset = ipc.data.size();
                        seq_mapper_.remove_mapping(seq_id);
                        break;
                    }
                    std::vector<uint8_t> bytes = spi_frame.toBytes();
                    LOGI("Send to slave: %s\n", bytesToHexString(bytes).c_str());
                    spi_transfer(bytes.data(), nullptr, bytes.size());
                    offset = spi_frame.next_;
                }
                seq_id_++;
                break;
            }

            case PacketSource::GPIO: {
                auto& gpio = std::get<GPIOData>(pkt_tx.payload);
                LOGI("Processing GPIO packet, GPIO level=%u", gpio.level);

                // GPIO low ==> MCU has data to read
                if (isSpiReadRequired(gpio.level)) {
                    // send SERVICE_MCU_REQUEST
                    SpiFrame spi_frame = SpiFrame(Direction::SOC2MCU, seq_id_, Command::SERVICE_MCU_REQUEST, std::vector<uint8_t>{});

                    tx_queue_.push_front(Packet{PacketSource::READ, std::monostate{}});
                    LOGI("Push READ Packet");
                }
                break;
            }

            case PacketSource::READ: {
                LOGI("Processing READ packet");

                spi_transfer(nullptr, recv_buf.data(), recv_buf.size());
                LOGI("Received from slave: %s", bytesToHexString(recv_buf).c_str());

#ifdef SIMULATE_GPIO_BEHAVIOR
                recv_buf = makeTestSpiFrame(seq_id_).toBytes();
#endif
                auto spi_frame_opt  = SpiFrame::fromBytes(recv_buf);
                if (spi_frame_opt) {
                    auto pkt_up_opt = gen_ipc_packet(*spi_frame_opt);
                    if (pkt_up_opt) rx_queue_.push(*pkt_up_opt);
                } else {
                    LOGE("Failed to generate IPC Packet from SPI frame");
                }
                break;
            }
        }
    }
}

int SpiManager::spi_transfer(const uint8_t* tx_buf, uint8_t* rx_buf, size_t len) {
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

bool SpiManager::isSpiReadRequired(uint8_t gpio_level) {
    return (gpio_level != 1); // GPIO High ==> MCU no data to write
}

bool SpiManager::init_gpio() {
    // open GPIO
    gpio_fd_ = open(GPIO_PATH, O_RDONLY | O_NONBLOCK);
    if (gpio_fd_ < 0) {
        LOGE("Failed to open GPIO: %s, err: %s", GPIO_PATH, strerror(errno));
        return false;
    }

    // create an epoll instance
    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
        LOGE("Failed to create epoll instance: %s", strerror(errno));
        close(gpio_fd_);
        return false;
    }

    // Configure epoll to monitor the GPIO file descriptor
    struct epoll_event ev;
    ev.events = EPOLLPRI | EPOLLERR; // Monitor high-priority events and errors
    ev.data.fd = gpio_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, gpio_fd_, &ev) < 0) {
        LOGE("Failed to add GPIO to epoll: %s", strerror(errno));
        close(gpio_fd_);
        close(epoll_fd_);
        return false;
    }

    // Clear the initial GPIO interrupt status
    char dummy[8];
    lseek(gpio_fd_, 0, SEEK_SET);
    read(gpio_fd_, dummy, sizeof(dummy));

    // Start the GPIO monitoring thread
    gpio_thread_ = std::thread(&SpiManager::gpioMonitorThread, this);

    return true;
}

// Use the SOC wakeup PIN as an MCU interrupt to notify the SOC.
// When the MCU has data transmission needs, pull the PIN low until the transmission is complete, then pull it high.
uint8_t SpiManager::check_gpio_level_and_notify(uint8_t &last_level) {
    char value[2];
    lseek(gpio_fd_, 0, SEEK_SET);
    ssize_t bytes_read = read(gpio_fd_, value, 1);
    if (bytes_read <= 0) {
        LOGE("Failed to read GPIO level: %s", strerror(errno));
        return last_level; // Return the previous value if reading fails
    }

    uint8_t new_level = (value[0] - 0x30);
    if (new_level != last_level) {
        last_level = new_level;
        tx_queue_.push_front(Packet{PacketSource::GPIO, GPIOData{new_level}});
        LOGI("Push GPIO Packet, GPIO level=%u", new_level);
    }
    return last_level;
}

void SpiManager::gpioMonitorThread() {
    struct epoll_event events[1];
    uint8_t level = 1; // Record the last GPIO level state, default is high

    while (running_) {
        int nfds = epoll_wait(epoll_fd_, events, 1, 100);
        if (!running_) break;

        if (nfds < 0) {
            LOGE("epoll_wait failed: %s", strerror(errno));
            continue;
        }

        if (nfds == 1 && (events[0].events & (EPOLLPRI | EPOLLERR))) {
            // On the first check, trigger and send the event
            check_gpio_level_and_notify(level);

            // -------- Loop polling to check for level changes --------
            const int max_poll_ms = 500;
            for (int elapsed = 0; elapsed < max_poll_ms && running_; elapsed += 10) {
                usleep(10 * 1000);
                uint8_t now_level = check_gpio_level_and_notify(level);

                // If it returns to High, the loop can terminate early
                if (now_level == 1) break;
            }
        }
    }
}

std::optional<Packet> SpiManager::gen_ipc_packet(const SpiFrame& spi_frame) {
    Packet pkt_rx;
    pkt_rx.source = PacketSource::IPC;

    if (spi_frame.cmd_id_ == Command::WRITE_UNREL) {
        LOGI("Command::WRITE_UNREL");
        pkt_rx.payload = IPCData{-1, MessageFlow::NOTIFY, spi_frame.seq_id_, spi_frame.payload_}; // client fd -1 means broadcast
        return pkt_rx;
    } else if  (spi_frame.cmd_id_ == Command::READ_UNREL) {
        LOGI("Command::READ_UNREL");
        uint16_t mcu_cmd = static_cast<uint16_t>(spi_frame.payload_[0]) | (static_cast<uint16_t>(spi_frame.payload_[1]) << 8);
        uint16_t seq_id = seq_mapper_.add_mapping(spi_frame.seq_id_, -1, static_cast<SpiCommon::McuCommand>(mcu_cmd));
        pkt_rx.payload = IPCData{-1, MessageFlow::NOTIFY, seq_id, spi_frame.payload_}; // client fd -1 means broadcast
        return pkt_rx;
    } else if (spi_frame.cmd_id_ == Command::RESPONSE) {
        LOGI("Command::RESPONSE");
        SeqMapper::Entry entry;
        seq_mapper_.find_mapping(spi_frame.seq_id_, entry);

        if (entry.client_fd < 0) { // No matching client_fd found
            LOGE("no (client fd, IPC's SEQ_ID) map to this SPI's SEQ_ID(%d)", spi_frame.seq_id_);
            return std::nullopt;
        } else {
            seq_mapper_.remove_mapping(spi_frame.seq_id_);
        }

        pkt_rx.payload = IPCData{entry.client_fd, MessageFlow::RESPONSE, entry.seq, spi_frame.payload_};
        return pkt_rx;
    }
    else {
        LOGE("Unknown spi_frame.cmd_id_ = %d", spi_frame.cmd_id_);
        return std::nullopt;
    }
}

#ifdef SIMULATE_GPIO_BEHAVIOR
bool SpiManager::init_mock_gpio() {
    // Create the file to ensure it exists
    int fd = open(MOCK_GPIO_PATH, O_CREAT | O_RDWR, 0644);
    if (fd < 0) {
        LOGE("Failed to create mock gpio file: %s", strerror(errno));
        return false;
    }
    close(fd);

    inotify_fd_ = inotify_init();
    if (inotify_fd_ < 0) {
        LOGE("Failed to init inotify: %s", strerror(errno));
        return false;
    }

    watch_fd_ = inotify_add_watch(inotify_fd_, MOCK_GPIO_PATH, IN_MODIFY);
    if (watch_fd_ < 0) {
        LOGE("Failed to add watch: %s", strerror(errno));
        close(inotify_fd_);
        return false;
    }

    // Open the mock file (used instead of gpio_fd_)
    gpio_fd2_ = open(MOCK_GPIO_PATH, O_RDONLY);
    if (gpio_fd2_ < 0) {
        LOGE("Failed to open mock gpio file: %s", strerror(errno));
        return false;
    }

    // Start the monitoring thread
    gpio_thread2_ = std::thread(&SpiManager::mockGpioThread, this);
    return true;
}

uint8_t SpiManager::check_mock_gpio_level_and_notify(uint8_t &last_level) {
    char value[2];
    lseek(gpio_fd2_, 0, SEEK_SET);
    ssize_t bytes_read = read(gpio_fd2_, value, 1);
    if (bytes_read <= 0) {
        LOGE("Failed to read MOCK GPIO level: %s", strerror(errno));
        return last_level; // Return the previous value if reading fails
    }

    uint8_t new_level = (value[0] - 0x30);
    if (new_level != last_level) {
        last_level = new_level;

        Packet pkt;
        pkt.source = PacketSource::GPIO;
        pkt.payload = GPIOData{new_level};
        tx_queue_.push(pkt);

        LOGI("MOCK GPIO level changed to %u", new_level);
    } else {
        LOGI("MOCK GPIO level no changed, %u", last_level);
    }
    return last_level;
}

void SpiManager::mockGpioThread() {
    char buf[1024];
    uint8_t level = 1; // default high

    while (running_) {
        int len = read(inotify_fd_, buf, sizeof(buf));
        LOGI("mockGpioThread, len=%d", len);
        if (len <= 0) {
            usleep(100 * 1000); // 0.1s
            continue;
        }

        // If the file is modified → check the GPIO level and send the event
        check_mock_gpio_level_and_notify(level);
    }
}

SpiFrame SpiManager::makeTestSpiFrame(uint16_t seq_id) {
    char value[2];
    std::vector<uint8_t> payload;
    Command cmd = Command::WRITE_UNREL;

    lseek(gpio_fd2_, 0, SEEK_SET);
    ssize_t bytes_read = read(gpio_fd2_, value, 1);

    uint8_t gpio_level = 0;
    if (bytes_read <= 0) {
        LOGE("makeTestSpiFrame, Failed to read MOCK GPIO level: %s", strerror(errno));
    } else {
        gpio_level = (value[0] - 0x30);
    }

    // sub cmd(2) + data
    switch (gpio_level) {
        case 0:
            // MCU write to SoC
            cmd = Command::WRITE_UNREL;
            payload = {0x04, 0x00, 0x12, 0x13};
            break;
        case 1:
            // GPIO High ==> MCU no data to write
            break;
        case 2:
            // MCU response to SOC
            cmd = Command::RESPONSE;
            payload = {0x06, 0x00, 0xED, 0xEE, 0xEF};
            break;
        case 3:
            // MCU read to SOC
            cmd = Command::READ_UNREL;
            payload = {0x07, 0x00, 0xC1, 0xC2, 0xC3};
            break;
        default:
            payload = {0x02, 0x00, 0x11, 0x12, 0x13};
            break;
    }

    SpiFrame frame(Direction::MCU2SOC, seq_id, cmd, payload);

    LOGI("makeTestSpiFrame: %s", frame.toString().c_str());

    return frame;
}
#endif // SIMULATE_GPIO_BEHAVIOR