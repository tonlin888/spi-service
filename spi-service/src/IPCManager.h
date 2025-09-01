#pragma once
#include <atomic>
#include <thread>
#include <unistd.h>
#include "MessageQueue.h"
#include "Packet.h"

class IPCManager {
public:
    static constexpr const char* IPC_SOCKET_PATH = "/tmp/spi-service/ipc.sock";
    
    IPCManager(MessageQueue<Packet>& mq, const char* socket_path = IPC_SOCKET_PATH)
        : msg_queue_(mq), socket_path_(socket_path) {}
    ~IPCManager() {
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

    void run(); // thread entry
    void start();
    void stop();

private:
    std::atomic<bool> running_;
    std::thread ipc_thread_;
    MessageQueue<Packet>& msg_queue_;
    std::string socket_path_;
    int listen_fd_ = -1;
    int epoll_fd_ = -1;

    void ensureDirExistsAndClean();
    void accept_client();
    void handle_client(int client_fd);
    void remove_client(int client_fd);
};
