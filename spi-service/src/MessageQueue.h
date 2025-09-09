#pragma once
#include <deque>
#include <mutex>
#include <condition_variable>
#include <optional>

template <typename T>
class MessageQueue {
public:
    // push 到最後 (一般優先度)
    void push(const T& msg) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_back(msg);
        }
        cond_.notify_one();
    }

    // push 到最前面 (高優先度)
    void push_front(const T& msg) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            queue_.push_front(msg);
        }
        cond_.notify_one();
    }

    // 阻塞 pop (等待有資料或 stop)
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        cond_.wait(lock, [this]{ return stopped_ || !queue_.empty(); });

        if (stopped_ && queue_.empty()) {
            return std::nullopt;  // queue 已停
        }

        T val = queue_.front();
        queue_.pop_front();
        return val;
    }

    // 非阻塞 pop
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.empty()) {
            return std::nullopt;
        }
        T val = queue_.front();
        queue_.pop_front();
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
    std::deque<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool stopped_ = false;
};
