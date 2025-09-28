#include "common.h"
#include "SpiManager.h"
#include "SpiFrame.h"
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>

#include <linux/spi/spidev.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>

#undef LOG_TAG
#define LOG_TAG "spi-service/SpiManager"

using Direction = SpiFrame::Direction;
using Command = SpiFrame::Command;
using MsgType = SpiCommon::MsgType;

SpiManager::SpiManager(MessageQueue<Packet>& tx_mq, MessageQueue<Packet>& rx_mq, SeqMapper& seq_mapper)
    : tx_queue_(tx_mq),
      rx_queue_(rx_mq),
      seq_mapper_(seq_mapper),
      epoll_fd_(-1),
      gpio_fd_(-1),
      running_(false)
{
    LOGI("SpiManager constructor");

    spi_dev_.open_dev();

    // initialize GPIO and epoll
    if (!init_gpio()) {
        LOGE("Failed to initialize GPIO, stopping SpiManager");
        return;
    }
}

SpiManager::~SpiManager() {
    spi_dev_.close_dev();
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
    std::vector<uint8_t> recv_buf(SpiCommon::MAX_SPI_FRAME_SIZE);

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

                auto spi_frame_opt = gen_spi_frame(ipc);
                if (!spi_frame_opt) {
                    break;
                }

                bool got_response = send_spi_frame(*spi_frame_opt, recv_buf);
                if (got_response) {
                    auto pkt_up_opt = parse_spi_response(recv_buf);
                    if (pkt_up_opt) rx_queue_.push(*pkt_up_opt);
                }

                seq_id_++;
                break;
            }

            case PacketSource::GPIO: {
                auto& gpio = std::get<GPIOData>(pkt_tx.payload);
                LOGI("Processing GPIO packet, GPIO level=%u", gpio.level);

                // GPIO low ==> MCU has data to read
                if (gpio.level == 0) {
                    // send SERVICE_MCU_REQUEST
                    SpiFrame spi_frame = SpiFrame(Direction::SOC2MCU, seq_id_, Command::SERVICE_MCU_REQUEST, std::vector<uint8_t>{});
                    send_spi_frame(spi_frame, recv_buf);
                }
                break;
            }

            case PacketSource::READ: {
                LOGI("Processing READ packet");

                if (spi_dev_.read_async(recv_buf.data(), recv_buf.size(), SpiDev::RELIABLE) == 0) {
                    auto pkt_up_opt = parse_spi_response(recv_buf);
                    if (pkt_up_opt) rx_queue_.push(*pkt_up_opt);
                } else {
                    tx_queue_.push_front(Packet{PacketSource::READ, std::monostate{}});
                    LOGE("Push READ Packet again!");
                }
                LOGI("Received from slave: %s", SpiCommon::bytesToHexString(recv_buf).c_str());
                break;
            }
        }
    }
}

std::optional<Packet> SpiManager::gen_ipc_packet(const SpiFrame& spi_frame) {
    Packet pkt_rx;
    pkt_rx.source = PacketSource::IPC;

    switch (spi_frame.cmd_id_) {
        case Command::WRITE_UNREL:
        case Command::WRITE: {
            LOGI("generate IPC message from Command::%s",
                 spi_frame.cmd_id_ == Command::WRITE ? "WRITE" : "WRITE_UNREL");

            // Broadcast to all clients
            pkt_rx.payload = IPCData{
                -1,
                MsgType::NOTIFY,
                spi_frame.seq_id_,
                spi_frame.get_effective_payload()
            };
            return pkt_rx;
        }

        case Command::READ_UNREL:
        case Command::READ: {
            LOGI("generate IPC message from Command::%s",
                 spi_frame.cmd_id_ == Command::READ ? "READ" : "READ_UNREL");

            uint16_t mcu_cmd = static_cast<uint16_t>(spi_frame.payload_[0]) |
                               (static_cast<uint16_t>(spi_frame.payload_[1]) << 8);

            // Create a new mapping (temporarily save client_fd=-1, waiting for subsequent update)
            uint16_t seq_id = seq_mapper_.add_mapping(
                spi_frame.seq_id_, -1, static_cast<SpiCommon::McuCommand>(mcu_cmd));

            // Broadcast to all clients
            pkt_rx.payload = IPCData{
                -1,
                MsgType::NOTIFY,
                seq_id,
                spi_frame.get_effective_payload()
            };
            return pkt_rx;
        }

        case Command::RESPONSE: {
            LOGI("generate IPC message from Command::RESPONSE");

            SeqMapper::Entry entry;
            seq_mapper_.find_mapping(spi_frame.seq_id_, entry);

            if (entry.client_fd < 0) {
                LOGE("No client's seq id maps to this spi's seq id(%u)", spi_frame.seq_id_);
                return std::nullopt;
            }

            // Successfully found mapping -> Remove
            seq_mapper_.remove_mapping(spi_frame.seq_id_);

            pkt_rx.payload = IPCData{
                entry.client_fd,
                MsgType::RESPONSE,
                entry.seq,
                spi_frame.get_effective_payload()
            };
            return pkt_rx;
        }

        default: {
            LOGE("Unknown spi_frame.cmd_id_ = %u", static_cast<uint8_t>(spi_frame.cmd_id_));
            return std::nullopt;
        }
    }
}

// The "unreliable" command is not necessary.
//#define RUN_UNRELIABLE_MODE 1
std::optional<SpiFrame> SpiManager::gen_spi_frame(const IPCData& ipc) {
    uint16_t mcu_cmd = static_cast<uint16_t>(ipc.data[0]) |
                       (static_cast<uint16_t>(ipc.data[1]) << 8);

    uint16_t seq_id = 0;
    Command cmd = Command::UNKNOWN_CMD;

    switch (ipc.typ) {
        case MsgType::EXECUTE_REQ: {
            LOGI("generate SPI frame from EXECUTE_REQ");
            // Create a new mapping: IPC seq -> MCU seq
            seq_id = seq_mapper_.add_mapping(
                ipc.seq, ipc.client_fd,
                static_cast<SpiCommon::McuCommand>(mcu_cmd));
#if defined(RUN_UNRELIABLE_MODE)
            cmd = Command::READ_UNREL;
#else
            cmd = Command::READ;
#endif

            break;
        }

        case MsgType::SET_REQ: {
            LOGI("generate SPI frame from SET_REQ");
            SeqMapper::Entry entry;
            seq_mapper_.find_mapping(ipc.seq, entry);

            if (entry.client_fd == ipc.client_fd) {
                // Reply to MCU read using MCU's sequence ID
                seq_id = entry.seq;
                cmd = Command::RESPONSE;
                seq_mapper_.remove_mapping(ipc.seq);
            } else {
                seq_id = seq_mapper_.allocate_mapped_seq();
#if defined(RUN_UNRELIABLE_MODE)
                cmd = Command::WRITE_UNREL;
#else
                cmd = Command::WRITE;
#endif
            }
            break;
        }

        default: {
            LOGE("gen_spi_frame, got %sOnly, but only EXECUTE_REQ and SET_REQ need to be sent to the MCU!",
                SpiCommon::MsgTypeToStr(ipc.typ).c_str());
            return std::nullopt;
        }
    }
    return SpiFrame(Direction::SOC2MCU, seq_id, cmd, ipc.data);
}

std::optional<Packet> SpiManager::parse_spi_response(const std::vector<uint8_t>& recv_buf)
{
    auto spi_frame_opt = SpiFrame::fromBytes(recv_buf);
    if (!spi_frame_opt) {
        LOGE("parse_spi_response, invalid frame!");
        return std::nullopt;
    }

    auto pkt_up_opt = gen_ipc_packet(*spi_frame_opt);
    if (!pkt_up_opt) return std::nullopt;

    // If it is READ / READ_UNREL, set the waiting response
    auto& ipc = std::get<IPCData>(pkt_up_opt->payload);
    if (spi_frame_opt->cmd_id_ == Command::READ ||
        spi_frame_opt->cmd_id_ == Command::READ_UNREL)
    {
        uint16_t mcu_cmd = static_cast<uint16_t>(ipc.data[0]) |
                           (static_cast<uint16_t>(ipc.data[1]) << 8);
        tx_queue_.set_waiting_response(ipc.seq, mcu_cmd);
    }

    return pkt_up_opt;
}

bool SpiManager::send_spi_frame(const SpiFrame& frame, std::vector<uint8_t>& recv_buf)
{
    bool got_response = false;
    std::vector<uint8_t> bytes = frame.toBytes();
    LOGI("--- send_spi_frame %s begin ---", SpiFrame::commandToStr(frame.cmd_id_).c_str());

    switch (frame.cmd_id_) {
        case Command::WRITE_UNREL:
        case Command::RESPONSE:
            // No matter whether the transmission is reliable or unreliable,
            // the MCU will not send an ACK after RESPONSE message be received.
            spi_dev_.write(bytes.data(), nullptr, bytes.size(), SpiDev::UNRELIABLE);
            break;
        case Command::WRITE:
            spi_dev_.write(bytes.data(), nullptr, bytes.size(), SpiDev::RELIABLE);
            break;
        case Command::READ_UNREL:
            got_response = (spi_dev_.read(bytes.data(), recv_buf.data(),
                                 bytes.size(), SpiDev::UNRELIABLE) == 0);
            break;
        case Command::READ:
            got_response = (spi_dev_.read(bytes.data(), recv_buf.data(),
                                 bytes.size(), SpiDev::RELIABLE) == 0);
            break;
        case Command::SERVICE_MCU_REQUEST:
            if (spi_dev_.write(bytes.data(), nullptr, bytes.size(), SpiDev::RELIABLE) == 0) {
                tx_queue_.push_front(Packet{PacketSource::READ, std::monostate{}});
                LOGI("Push READ packet");
            }
            break;
        default:
            LOGE("send_spi_frame, invalid command %u", static_cast<uint8_t>(frame.cmd_id_));
    }
    LOGI("--- send_spi_frame %s end ---", SpiFrame::commandToStr(frame.cmd_id_).c_str());
    return got_response;
}

// ====================== Handle GPIO event =============================

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
            LOGI("GPIO has been triggered");
            // On the first check, trigger and send the event
            check_gpio_level_and_notify(level);

            // -------- Loop polling to check for level changes --------
            const int max_poll_ms = 500;
            for (int elapsed = 0; elapsed < max_poll_ms && running_; elapsed += 10) {
                usleep(10 * 1000);
                uint8_t now_level = check_gpio_level_and_notify(level);

                // If it returns to High, the loop can terminate early
                if (now_level == 1) {
                    LOGI("gpioMonitorThread, GPIO level change to 1");
                    tx_queue_.pop_front_if_read();
                    tx_queue_.clear_waiting_response();
                    break;
                }
            }
        }
    }
}