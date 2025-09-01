#include "common.h"
#include "IPCManager.h"
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/stat.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define MAX_EVENTS 10
#define BUF_SIZE 512


void IPCManager::ensureDirExistsAndClean() {
    // 1. 找出目錄部分
    std::string dir = socket_path_;
    size_t pos = dir.find_last_of('/');
    if (pos != std::string::npos) {
        dir = dir.substr(0, pos);
    } else {
        dir = ".";
    }
    LOGI("[IPC] ensureDirExistsAndClean, dir: %s", dir.c_str());

    // 2. 建立目錄 (支援多層)
    size_t p = 0;
    while ((p = dir.find('/', p + 1)) != std::string::npos) {
        std::string subdir = dir.substr(0, p);
        if (!subdir.empty()) {
            mkdir(subdir.c_str(), 0777); // 已存在會回 -1, 忽略 EEXIST
        }
    }
    mkdir(dir.c_str(), 0777); // 最後一層

    // 3. 刪除舊的 socket file
    if (unlink(socket_path_.c_str()) < 0 && errno != ENOENT) {
        LOGE("[IPC] unlink failed: %s", strerror(errno));
    }
}

void IPCManager::init_socket() {
    ensureDirExistsAndClean();

    listen_fd_ = socket(AF_UNIX, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        LOGE("[IPC] socket failed: %s", strerror(errno));
        throw std::runtime_error("Failed to create socket");
    }

    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path)-1);
    unlink(socket_path_.c_str());

    if (bind(listen_fd_, (sockaddr*)&addr, sizeof(addr)) < 0) {
        LOGE("[IPC] bind failed on fd=%d: %s %s", listen_fd_, strerror(errno), socket_path_.c_str());
        throw std::runtime_error("Failed to bind socket");
    }
    if (listen(listen_fd_, 5) < 0) {
        LOGE("[IPC] listen failed on fd=%d: %s", listen_fd_, strerror(errno));
        throw std::runtime_error("Failed to listen on socket");
    }

    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
       LOGE("[IPC] epoll_create1 failed: %s", strerror(errno));
       throw std::runtime_error("Failed to create epoll instance");
    }

    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = listen_fd_;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, listen_fd_, &ev);

    LOGI("[IPC] Listening on %s\n", socket_path_.c_str());
}
    
void IPCManager::accept_client() {
    int client_fd = accept(listen_fd_, nullptr, nullptr);
    if (client_fd < 0) {
        LOGE("[IPC] accept failedclient_fd=%d: %s", client_fd, strerror(errno));
        return;
    }

    epoll_event ev{};
    ev.events = EPOLLIN;
    ev.data.fd = client_fd;
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, client_fd, &ev);
    LOGI("[IPC] Client connected fd=%d", client_fd);
}

void IPCManager::handle_client(int client_fd) {
    uint8_t buf[BUF_SIZE];
    ssize_t n = recv(client_fd, buf, sizeof(buf), 0);
    if (n > 0) {
        Packet pkt;
        pkt.source = PacketSource::IPC;
        pkt.payload = IPCData{client_fd, std::vector<uint8_t>(buf, buf + n)};
        msg_queue_.push(pkt);
        LOGI("[IPC] Received %d, bytes from fd=%d", n, client_fd);
    } else if (n == 0) {
        LOGI("[IPC] Client closed connection, fd=%d", client_fd);
        remove_client(client_fd);
    } else {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            LOGE("[IPC] recv() failed on fd=%d: no data temporarily (EAGAIN || EWOULDBLOCK)", client_fd);
        } else if (errno == EINTR) {
            LOGE("[IPC] recv() failed on fd=%d: interrupt (EINTR)", client_fd);
        } else {
            LOGE("[IPC] recv() failed on fd=%d: %s", client_fd, strerror(errno));
            remove_client(client_fd);
        }
    }
}

void IPCManager::remove_client(int client_fd) {
    epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, client_fd, nullptr);
    close(client_fd);
    LOGI("[IPC] Client disconnected fd=%d", client_fd);
}

void IPCManager::start() {
    running_ = true;
    ipc_thread_ = std::thread(&IPCManager::run, this);
}

void IPCManager::stop() {
    running_ = false;
    if (ipc_thread_.joinable())
        ipc_thread_.join();
}

void IPCManager::run() {
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
