#pragma once

#include "common.h"
#include <unordered_map>
#include <cstdint>
#include <mutex>
#include <vector>
#include <algorithm>
#include <iostream>

#undef LOG_TAG
#define LOG_TAG "spi-service/SeqMapper"

class SeqMapper {
public:
    struct Entry {
        uint16_t ipc_seq;
        int client_fd;
        uint64_t order; // Insertion order
    };

    SeqMapper()
        : next_spi_seq_(1), next_order_(1) {}

    // Add a mapping. SPI_SEQ auto-increments. Keep at most MAX_ENTRIES entries, remove the oldest if exceeded.
    uint16_t add_mapping(uint16_t ipc_seq, int client_fd) {
        std::lock_guard<std::mutex> lock(mu_);
        uint16_t spi_seq = next_spi_seq_++;
        if (next_spi_seq_ == 0) next_spi_seq_ = 1; // avoid spi_seq = 0

        // Remove the oldest if exceeding MAX_ENTRIES
        if (spi_to_entry_.size() >= MAX_ENTRIES) {
            auto it = std::min_element(
                spi_to_entry_.begin(), spi_to_entry_.end(),
                [](const std::pair<const uint16_t, Entry> &a, const std::pair<const uint16_t, Entry> &b) { return a.second.order < b.second.order; }
            );
            if (it != spi_to_entry_.end()) {
                spi_to_entry_.erase(it->first);
            }
        }

        spi_to_entry_[spi_seq] = Entry{ipc_seq, client_fd, next_order_++};
        LOGI("add_mapping, spi_seq=%u = {ipc_seq=%u, client_fd=%d}", spi_seq, ipc_seq, client_fd);
        return spi_seq;
    }

    // Find mapping. If not found, client_fd = -1
    void find_mapping(uint16_t spi_seq, Entry &out_entry) {
        std::lock_guard<std::mutex> lock(mu_);
        auto it = spi_to_entry_.find(spi_seq);

        if (it == spi_to_entry_.end()) {
            // Not found, return default values
            out_entry.client_fd = -1;
            out_entry.ipc_seq = 0;
            out_entry.order = 0;
            LOGI("find_mapping, spi_seq=%u not found", spi_seq);
        } else {
            out_entry = it->second;
            LOGI("find_mapping, spi_seq=%u = {ipc_seq=%u, client_fd=%d}", out_entry.ipc_seq, out_entry.client_fd);
        }
    }

    // Remove a single mapping by SPI_SEQ
    bool remove_mapping(uint16_t spi_seq) {
        std::lock_guard<std::mutex> lock(mu_);
        LOGI("remove_mapping, spi_seq=%u", spi_seq);
        return spi_to_entry_.erase(spi_seq) > 0;
    }

    // Remove all mappings for a specific client_fd
    std::vector<uint16_t> remove_client(int client_fd) {
        std::lock_guard<std::mutex> lock(mu_);
        LOGI("remove_client, client_fd=%d", client_fd);

        std::vector<uint16_t> removed;
        for (auto it = spi_to_entry_.begin(); it != spi_to_entry_.end();) {
            if (it->second.client_fd == client_fd) {
                removed.push_back(it->first);
                it = spi_to_entry_.erase(it);
            } else {
                ++it;
            }
        }
        return removed;
    }

    // Get the number of current mappings
    size_t size() {
        std::lock_guard<std::mutex> lock(mu_);
        return spi_to_entry_.size();
    }

    // Debug: print all mappings in insertion order
    void print_all() {
        std::lock_guard<std::mutex> lock(mu_);
        std::vector<std::pair<uint16_t, Entry>> entries(spi_to_entry_.begin(), spi_to_entry_.end());

        std::sort(entries.begin(), entries.end(),
                  [](auto &a, auto &b) { return a.second.order < b.second.order; });

        std::cout << "---- SeqMapper current mappings ----\n";
        for (auto &kv : entries) {
            std::cout << "SPI_SEQ=" << kv.first
                      << " IPC_SEQ=" << kv.second.ipc_seq
                      << " FD=" << kv.second.client_fd
                      << " ORDER=" << kv.second.order << "\n";
        }
        std::cout << "------------------------------------------\n";
    }

private:
    std::unordered_map<uint16_t, Entry> spi_to_entry_;
    uint16_t next_spi_seq_;
    uint64_t next_order_;
    static constexpr size_t MAX_ENTRIES = 10;
    std::mutex mu_;
};
