#pragma once
#include "common.h"
#include "Packet.h"
#include <deque>
#include <mutex>
#include <condition_variable>
#include <optional>

template <typename T>
class MessageQueue {
public:
    // Push message to the back (normal priority)
    void push(const T& msg) {
        {
            // When pushing: If it's not a correct response, put it in pending_queue_.
            // If a correct response is received, put it in the front of the queue and flush the previously stored data.
            std::lock_guard<std::mutex> lock(mutex_);
            if (is_ipc_msg(msg) && waiting_response_ != 0) {
                if (get_seq(msg) != waiting_response_ || get_mcu_cmd(msg) != waiting_mcu_cmd_) {
                    LOGI("push, got message(%u, %u) waiting(%u, %u)",
                        get_seq(msg), get_mcu_cmd(msg), waiting_response_, waiting_mcu_cmd_);
                    pending_queue_.push_back(msg);
                    return;
                }
                queue_.push_front(msg);
                flush_pending_locked();
                cond_.notify_one();
                return;
            }
            queue_.push_back(msg);
        }
        cond_.notify_one();
    }

    // Push message to the front (high priority)
    void push_front(const T& msg) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_front(msg);
        }
        cond_.notify_one();
    }

    // remove all packets of given source type
    uint8_t remove_all_of(PacketSource type) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return 0;
        }
    
        uint8_t removed_count = 0;
        for (auto it = queue_.begin(); it != queue_.end();) {
            if (it->source == type) {
                LOGI("remove_all_of, removing packet type=%d", static_cast<uint8_t>(type));
                it = queue_.erase(it);
                ++removed_count;
            } else {
                ++it;
            }
        }
        return removed_count;
    }

    // Blocking pop (wait until there is data or the queue is stopped)
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return stopped_ || !queue_.empty(); });

        if (stopped_ && queue_.empty()) {
            return std::nullopt;  // Queue has been stopped
        }

        T val = queue_.front();
        queue_.pop_front();
        return val;
    }

    // Non-blocking pop
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }
        T val = queue_.front();
        queue_.pop_front();
        return val;
    }

    // Stop the queue and notify all waiting threads
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        cond_.notify_all();
    }

    // Check if the queue is empty
    bool empty() {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

    void set_waiting_response(uint16_t seq, uint16_t mcu_cmd) {
        LOGI("set_waiting_response, seq=%u mcu_cmd=%u", seq, mcu_cmd);
        std::lock_guard<std::mutex> lock(mutex_);
        waiting_response_ = seq;
        waiting_mcu_cmd_ = mcu_cmd;
    }

    void clear_waiting_response() {
        if (waiting_response_ || waiting_mcu_cmd_) {
            LOGI("clear_waiting_response, waiting_response_=%u waiting_mcu_cmd_=%u",
                waiting_response_, waiting_mcu_cmd_);
        }
        std::lock_guard<std::mutex> lock(mutex_);
        waiting_response_ = 0;
        waiting_mcu_cmd_ = 0;
        flush_pending_locked();
    }

private:
    bool is_ipc_msg(const T& msg) const {
        if constexpr (std::is_same_v<T, Packet>) {
            return (msg.source == PacketSource::IPC);
        }
    }
    uint16_t get_seq(const T& msg) const {
        if constexpr (std::is_same_v<T, Packet>) {
            if (msg.source == PacketSource::IPC && std::holds_alternative<IPCData>(msg.payload)) {
                const IPCData& ipc_msg = std::get<IPCData>(msg.payload);
                return ipc_msg.seq;
            }
        }
        return 0;
    }
    uint16_t get_mcu_cmd(const T& msg) const {
        if constexpr (std::is_same_v<T, Packet>) {
            if (msg.source == PacketSource::IPC && std::holds_alternative<IPCData>(msg.payload)) {
                const IPCData& ipc_msg = std::get<IPCData>(msg.payload);
                if (ipc_msg.data.size() >= 2) {
                    return (static_cast<uint16_t>(ipc_msg.data[0]) | (static_cast<uint16_t>(ipc_msg.data[1]) << 8));
                }
            }
        }
        return UINT16_MAX;
    }

    void flush_pending_locked() {
        size_t count = pending_queue_.size();
        if (count > 0) {
            LOGI("Moving %zu item(s) from pending_queue_ back to queue_", count);
        }
        for (auto& m : pending_queue_) {
            queue_.push_back(m);
        }
        pending_queue_.clear();
    }

    std::deque<T> queue_;
    std::deque<T> pending_queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool stopped_ = false;
    uint16_t waiting_response_{0};
    uint16_t waiting_mcu_cmd_{0};
};
