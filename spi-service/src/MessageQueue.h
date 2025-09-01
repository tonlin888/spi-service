#pragma once
#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>

template <typename T>
class MessageQueue {
public:
    
    void push(const T& msg) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push(msg);
        }
        cond_.notify_one();
    }

    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return stopped_ || !queue_.empty(); });
        
        if (stopped_ && queue_.empty()) {
            return std::nullopt;  // 表示 queue 已停
        }

        T val = queue_.front();
        queue_.pop();
        return val;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        cond_.notify_all();
    }
    
    bool empty() {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }

private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool stopped_ = false;
};
