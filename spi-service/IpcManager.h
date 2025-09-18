#pragma once
#include "MessageQueue.h"
#include "Packet.h"
#include "ClientMessage.h"
#include "SeqMapper.h"
#include <atomic>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>

#undef LOG_TAG
#define LOG_TAG "spi-service/IpcManager"

class IpcManager {
public:
    static constexpr const size_t MAX_EVENTS = 10;

    IpcManager(MessageQueue<Packet>& tx_mq, MessageQueue<Packet>& rx_mq, SeqMapper& seq_mapper, const char* socket_path = SpiCommon::IPC_SOCKET_PATH)
        : tx_queue_(tx_mq), rx_queue_(rx_mq), seq_mapper_(seq_mapper), socket_path_(socket_path) {}
    ~IpcManager() {
        if (listen_fd_ >= 0) {
            close(listen_fd_);
        }
        if (epoll_fd_ >= 0) {
            close(epoll_fd_);
        }
        if (!socket_path_.empty()) {
            unlink(socket_path_.c_str());
        }
    }

    void init_socket();

    void tx_run(); // transmit thread entry
    void rx_run(); // receive thread entry
    void start();
    void stop();

private:
    std::atomic<bool> running_;
    std::thread ipc_tx_thread_;
    std::thread ipc_rx_thread_;
    MessageQueue<Packet>& tx_queue_;
    MessageQueue<Packet>& rx_queue_;
    SeqMapper& seq_mapper_;
    std::string socket_path_;
    int listen_fd_ = -1;
    int epoll_fd_ = -1;
    uint16_t seq_{0};

    // cmd_id -> set of client_fd
    std::unordered_map<uint16_t, std::unordered_set<int>> cmd_map_;
    // SPI sequence id -> IPC sequence id, client_fd

    void init_directory();
    void accept_client();
    void handle_client(int client_fd);
    void remove_client(int client_fd);
    void send_response(int client_fd, const ClientMessage& msg);
    void process_cmd(int client_fd, const ClientMessage& msg);
    void process_message(int client_fd, const uint8_t* buf, size_t n);
    void register_cmd(int client_fd, uint16_t cmd_id);
    void unregister_cmd(int client_fd, uint16_t cmd_id);
    void unsubscribe_client(int client_fd);
    void send_ipc_to_clients(const IPCData& ipc);
};
