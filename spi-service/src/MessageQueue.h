#pragma once
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
            std::lock_guard<std::mutex> lock(mutex_);
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

private:
    std::deque<T> queue_;
    std::mutex mutex_;
    std::condition_variable cond_;
    bool stopped_ = false;
};
