#include "IpcManager.h"
#include "common.h"
#include "ClientMessage.h"
#include "SpiCommon.h"

#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <string_view>

#undef LOG_TAG
#define LOG_TAG "spi-service/IpcManager"

using MsgType = SpiCommon::MsgType;
using ErrorCode = SpiCommon::ErrorCode;

void IpcManager::init_directory() {
    // 1. Extract directory part from socket path
    std::string dir = socket_path_;
    size_t pos = dir.find_last_of('/');
    if (pos != std::string::npos) {
        dir = dir.substr(0, pos);
    } else {
        dir = ".";
    }
    LOGI("init_directory, dir: %s", dir.c_str());

    // 2. Create directory (support multi-level paths)
    size_t p = 0;
    while ((p = dir.find('/', p + 1)) != std::string::npos) {
        std::string subdir = dir.substr(0, p);
        if (!subdir.empty()) {
            mkdir(subdir.c_str(), 0777); // ignore EEXIST if already exists
        }
    }
    mkdir(dir.c_str(), 0777); // last level

    // 3. Remove old socket file if exists
    if (unlink(socket_path_.c_str()) < 0 && errno != ENOENT) {
        LOGE("unlink failed: %s", strerror(errno));
    }
}

void IpcManager::init_socket() {
    init_directory();

    listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        LOGE("socket failed: %s", strerror(errno));
        throw std::runtime_error("Failed to create socket");
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path)-1);
    unlink(socket_path_.c_str());

    if (bind(listen_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        LOGE("bind failed on fd=%d: %s %s", listen_fd_, strerror(errno), socket_path_.c_str());
        throw std::runtime_error("Failed to bind socket");
    }
    if (listen(listen_fd_, 5) < 0) {
        LOGE("listen failed on fd=%d: %s", listen_fd_, strerror(errno));
        throw std::runtime_error("Failed to listen on socket");
    }

    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
       LOGE("epoll_create1 failed: %s", strerror(errno));
       throw std::runtime_error("Failed to create epoll instance");
    }

    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = listen_fd_;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, listen_fd_, &ev);

    LOGI("Listening on %s\n", socket_path_.c_str());
}

void IpcManager::accept_client() {
    int client_fd = accept(listen_fd_, nullptr, nullptr);
    if (client_fd < 0) {
        LOGE("accept failed, client_fd=%d: %s", client_fd, strerror(errno));
        return;
    }

    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = client_fd;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, client_fd, &ev);
    LOGI("Client connected fd=%d", client_fd);
}

// Process raw bytes into commands (register or unregister)
int IpcManager::process_cmd_bytes(int client_fd, const std::vector<uint8_t>& buf, IpcManager::CmdMapAction act) {
    // Must be even size, because each command is uint16_t (little endian)
    if (buf.size() % 2 != 0) {
        LOGE("Invalid buffer size=%zu (must be even)!", buf.size());
        return -1;
    }

    // Parse each 16-bit value
    for (size_t i = 0; i < buf.size(); i += 2) {
        uint16_t val = static_cast<uint16_t>(buf[i]) | (static_cast<uint16_t>(buf[i + 1]) << 8);

        int group = static_cast<int>(val) - SpiCommon::CMD_GROUP_ID_BASE;
        if (group >= 0 && group < static_cast<int>(SpiCommon::CMD_GROUP.size())) {
            LOGI("process_cmd_bytes, value 0x%04X matched group %d", val, group);
            for (uint16_t gval : SpiCommon::CMD_GROUP[group]) {
                if (act == CmdMapAction::REGISTER) {
                    LOGI("process_cmd_bytes, Register command 0x%04X", gval);
                    register_cmd(client_fd, gval);
                } else if (act == CmdMapAction::UNREGISTER){
                    LOGI("process_cmd_bytes, Unregister command 0x%04X", gval);
                    unregister_cmd(client_fd, gval);
                }
            }
        } else {
            if (act == CmdMapAction::REGISTER) {
                LOGI("process_cmd_bytes, Register command 0x%04X", val);
                register_cmd(client_fd, val);
            } else if (act == CmdMapAction::UNREGISTER) {
                LOGI("process_cmd_bytes, Unregister command 0x%04X", val);
                unregister_cmd(client_fd, val);
            }
        }
    }
    return 0;
}

void IpcManager::process_cmd(int client_fd, const ClientMessage& msg) {
    Packet pkt;
    switch (static_cast<uint8_t>(msg.msg_t_)) {
        case static_cast<uint8_t>(MsgType::REGISTER_REQ):
            LOGI("process REGISTER_REQ ...");
            unsubscribe_client(client_fd);
            process_cmd_bytes(client_fd, msg.data_, CmdMapAction::REGISTER);
            send_response(client_fd, ClientMessage(msg.seq_, MsgType::RESPONSE, ErrorCode::NONE));
            break;

        case static_cast<uint8_t>(MsgType::UNREGISTER_REQ):
            LOGI("process UNREGISTER_REQ ...");
            process_cmd_bytes(client_fd, msg.data_, CmdMapAction::UNREGISTER);
            send_response(client_fd, ClientMessage(msg.seq_, MsgType::RESPONSE, ErrorCode::NONE));
            break;

        case static_cast<uint8_t>(MsgType::EXECUTE_REQ): // pass through
        case static_cast<uint8_t>(MsgType::SET_REQ):
            LOGI("process %s size=%u. push IPC packet ...", MsgTypeToStr(msg.msg_t_).c_str(), msg.data_.size());
            tx_queue_.push(Packet{PacketSource::IPC, IPCData{client_fd, msg.msg_t_, msg.seq_, msg.data_}});
            break;

        default:
            LOGE("process unknown command!");
    }
}

void IpcManager::process_message(int client_fd, const uint8_t* buf, size_t n) {
    auto msg_opt = ClientMessage::fromBytes(buf, n);
    if (!msg_opt) {
        LOGW("Invalid ClientMessage received");
        send_response(client_fd, ClientMessage(ClientMessage::get_seq_id(buf, n), MsgType::RESPONSE, ErrorCode::INVALID_FORMAT));
        return;
    }

    ClientMessage& msg = *msg_opt;
    LOGI("Got msg: %s\n", msg.toString().c_str());

    if (msg.data_.size() > SpiCommon::MAX_IPC_DATA_SIZE) {
        // Send error back to client if data size exceeds limit
        send_response(client_fd, ClientMessage(msg.seq_, MsgType::RESPONSE, ErrorCode::DATA_TOO_LONG));
        LOGW("Received %zd bytes from fd=%d, exceed MAX_IPC_DATA_SIZE=%d, dropped",
             msg.data_.size(), client_fd, SpiCommon::MAX_IPC_DATA_SIZE);
        return;
    }

    process_cmd(client_fd, msg);
}

void IpcManager::handle_client(int client_fd) {
    uint8_t buf[SpiCommon::MAX_IPC_PACKET_SIZE];
    ssize_t n = recv(client_fd, buf, sizeof(buf), 0);
    LOGI("Received %d, bytes from fd=%d: %s", n, client_fd, SpiCommon::bytesToHexString(buf, n).c_str());

    if (n > 0) {
        process_message(client_fd, buf, n);
    } else if (n == 0) {
        LOGI("Client closed connection, fd=%d", client_fd);
        remove_client(client_fd);
    } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            LOGE("recv() failed on fd=%d: no data temporarily (EAGAIN || EWOULDBLOCK)", client_fd);
        } else if (errno == EINTR) {
            LOGE("recv() failed on fd=%d: interrupt (EINTR)", client_fd);
        } else {
            LOGE("recv() failed on fd=%d: %s", client_fd, strerror(errno));
            remove_client(client_fd);
        }
    }
}

void IpcManager::remove_client(int client_fd) {
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, client_fd, nullptr);
    close(client_fd);

    seq_mapper_.remove_client(client_fd);
    unsubscribe_client(client_fd);
    LOGI("Client disconnected fd=%d", client_fd);
}

// Function to send response back to client
void IpcManager::send_response(int client_fd, const ClientMessage& msg) {
    std::vector<uint8_t> buf = msg.toBytes();

    ssize_t sent = send(client_fd, buf.data(), buf.size(), 0);
    if (sent < 0) {
        LOGE("send_response failed: %s", strerror(errno));
    } else {
        LOGI("send_response ok: fd=%d, %zu bytes, %s", client_fd, buf.size(), msg.toString().c_str());
    }
}

void IpcManager::unsubscribe_client(int client_fd) {
    for (auto it = cmd_map_.begin(); it != cmd_map_.end(); ) {
        it->second.erase(client_fd);
        if (it->second.empty()) {
            it = cmd_map_.erase(it);
        } else {
            ++it;
        }
    }
}

void IpcManager::register_cmd(int client_fd, uint16_t cmd_id) {
    cmd_map_[cmd_id].insert(client_fd);
}

void IpcManager::unregister_cmd(int client_fd, uint16_t cmd_id) {
    if (cmd_map_.count(cmd_id)) {
        cmd_map_[cmd_id].erase(client_fd);
        if (cmd_map_[cmd_id].empty()) {
            cmd_map_.erase(cmd_id);
        }
    }
}

void IpcManager::start() {
    running_ = true;
    ipc_tx_thread_ = std::thread(&IpcManager::tx_run, this);
    ipc_rx_thread_ = std::thread(&IpcManager::rx_run, this);
}

void IpcManager::stop() {
    running_ = false;
    if (ipc_tx_thread_.joinable())
        ipc_tx_thread_.join();
    if (ipc_rx_thread_.joinable())
        ipc_rx_thread_.join();
}

void IpcManager::tx_run() {
    epoll_event events[MAX_EVENTS];

    while (running_) {
        int nfds = epoll_wait(epoll_fd_, events, MAX_EVENTS, -1);
        if (nfds < 0) continue;

        for (int i = 0; i < nfds; ++i) {
            if (events[i].data.fd == listen_fd_) {
                accept_client();
            } else {
                handle_client(events[i].data.fd);
            }
        }
    }
}

void IpcManager::rx_run() {
    while (running_) {
        auto pkt_opt = rx_queue_.pop();
        if (!pkt_opt) {
            LOGE("got std::nullopt from rx_queue_pop(), stop running");
            break; // queue stopped
        }

        Packet& pkt = *pkt_opt;
        if (PacketSource::IPC == pkt.source) {
            auto& ipc = std::get<IPCData>(pkt.payload);
            LOGI("Processing IPC packet: %s", ipc.toString().c_str());

            send_ipc_to_clients(ipc);
         } else {
             LOGW("skip pkt.source = %u", static_cast<uint8_t>(pkt.source));
         }
    }
}

void IpcManager::send_ipc_to_clients(const IPCData& ipc) {

    uint16_t cmd_id = static_cast<uint16_t>(SpiCommon::McuCommand::SUB_CMD_INVALID);
    if (ipc.data.size() >= 2) {
        cmd_id = static_cast<uint16_t>(ipc.data[0]) | (static_cast<uint16_t>(ipc.data[1]) << 8);
        LOGI("send_ipc_to_clients, cmd_id=0x%04X", cmd_id);
    } else {
        LOGW("send_ipc_to_clients, ipc.data has less than 2 bytes");
    }

    if (ipc.typ == MsgType::NOTIFY) {
        auto it = cmd_map_.find(cmd_id);
        if (it == cmd_map_.end()) {
            LOGW("No clients registered for cmd_id=0x%04X", cmd_id);
            return;
        }

        uint16_t seq = (ipc.seq > 0) ? ipc.seq : seq_;
        std::vector<uint8_t> msg = ClientMessage(seq, MsgType::NOTIFY, ErrorCode::NONE, ipc.data).toBytes();
        for (int fd : it->second) {
            ssize_t n = send(fd, msg.data(), msg.size(), 0);
            seq_mapper_.update_client_if_unset(ipc.seq, fd);
            if (n < 0) {
                LOGE("Failed to send to client fd=%d: %s!", fd, strerror(errno));
            } else {
                LOGI("Sent %zd bytes to client fd=%d for cmd_id=0x%04X, %s", n, fd, cmd_id, SpiCommon::bytesToHexString(msg).c_str());
            }
        }
    } else if (ipc.typ == MsgType::RESPONSE) {
        std::vector<uint8_t> msg = ClientMessage(ipc.seq, MsgType::RESPONSE, ErrorCode::NONE, ipc.data).toBytes();
        ssize_t n = send(ipc.client_fd, msg.data(), msg.size(), 0);
        if (n < 0) {
            LOGE("Failed to send to client fd=%d: %s!", ipc.client_fd, strerror(errno));
        } else {
            LOGI("Sent %zd bytes to client fd=%d for cmd_id=0x%04X, %s", n, ipc.client_fd, cmd_id, SpiCommon::bytesToHexString(msg).c_str());
        }
    } else {
        LOGE("Unknown message type %d!", ipc.typ);
    }
}
