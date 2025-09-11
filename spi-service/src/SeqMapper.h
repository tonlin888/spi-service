#pragma once

#include "common.h"
#include "SpiCommon.h"
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
        uint16_t seq;
        int client_fd;
        SpiCommon::McuCommand cmd;
        uint64_t order; // Insertion order
    };

    SeqMapper()
        : next_mapped_seq_(1), next_order_(1) {}

    // Add a mapping. mapped_seq auto-increments. Keep at most MAX_ENTRIES entries, remove the oldest if exceeded.
    uint16_t add_mapping(uint16_t seq, int client_fd, SpiCommon::McuCommand cmd = SpiCommon::McuCommand::SUB_CMD_INVALID) {
        std::lock_guard<std::mutex> lock(mu_);
        uint16_t mapped_seq = next_mapped_seq_++;
        if (next_mapped_seq_ == 0) next_mapped_seq_ = 1; // avoid mapped_seq = 0

        // Remove the oldest if exceeding MAX_ENTRIES
        if (mapped_to_entry_.size() >= MAX_ENTRIES) {
            auto it = std::min_element(
                mapped_to_entry_.begin(), mapped_to_entry_.end(),
                [](const std::pair<const uint16_t, Entry> &a, const std::pair<const uint16_t, Entry> &b) { return a.second.order < b.second.order; }
            );
            if (it != mapped_to_entry_.end()) {
                mapped_to_entry_.erase(it->first);
            }
        }

        mapped_to_entry_[mapped_seq] = Entry{seq, client_fd, cmd, next_order_++};
        LOGI("add_mapping, mapped_seq=%u -> {seq=%u, client_fd=%d, cmd=%u}", mapped_seq, seq, client_fd, static_cast<uint16_t>(cmd));
        return mapped_seq;
    }

    // Find mapping. If not found, client_fd = -1
    void find_mapping(uint16_t mapped_seq, Entry &out_entry) {
        std::lock_guard<std::mutex> lock(mu_);
        auto it = mapped_to_entry_.find(mapped_seq);

        if (it == mapped_to_entry_.end()) {
            // Not found, return default values
            out_entry.client_fd = -1;
            out_entry.seq = 0;
            out_entry.cmd = SpiCommon::McuCommand::SUB_CMD_INVALID;
            out_entry.order = 0;
            LOGI("find_mapping, mapped_seq=%u not found", mapped_seq);
        } else {
            out_entry = it->second;
            LOGI("find_mapping, mapped_seq=%u = {seq=%u, client_fd=%d, cmd=%u}", mapped_seq, out_entry.seq, out_entry.client_fd, out_entry.cmd);
        }
    }

    // Remove a single mapping by mapped_seq
    bool remove_mapping(uint16_t mapped_seq) {
        std::lock_guard<std::mutex> lock(mu_);
        LOGI("remove_mapping, mapped_seq=%u", mapped_seq);
        return mapped_to_entry_.erase(mapped_seq) > 0;
    }

    // Remove all mappings for a specific client_fd
    std::vector<uint16_t> remove_client(int client_fd) {
        std::lock_guard<std::mutex> lock(mu_);
        LOGI("remove_client, client_fd=%d", client_fd);

        std::vector<uint16_t> removed;
        for (auto it = mapped_to_entry_.begin(); it != mapped_to_entry_.end();) {
            if (it->second.client_fd == client_fd) {
                removed.push_back(it->first);
                it = mapped_to_entry_.erase(it);
            } else {
                ++it;
            }
        }
        return removed;
    }

    // Get the number of current mappings
    size_t size() {
        std::lock_guard<std::mutex> lock(mu_);
        return mapped_to_entry_.size();
    }

    // Debug: print all mappings in insertion order
    void print_all() {
        std::lock_guard<std::mutex> lock(mu_);
        std::vector<std::pair<uint16_t, Entry>> entries(mapped_to_entry_.begin(), mapped_to_entry_.end());

        std::sort(entries.begin(), entries.end(),
                  [](auto &a, auto &b) { return a.second.order < b.second.order; });

        std::cout << "---- SeqMapper current mappings ----\n";
        for (auto &kv : entries) {
            std::cout << "mapped_seq=" << kv.first
                      << " seq=" << kv.second.seq
                      << " fd=" << kv.second.client_fd
                      << " cmd=" << static_cast<uint16_t>(kv.second.cmd)
                      << " order=" << kv.second.order << "\n";
        }
        std::cout << "------------------------------------------\n";
    }

private:
    std::unordered_map<uint16_t, Entry> mapped_to_entry_;
    uint16_t next_mapped_seq_;
    uint64_t next_order_;
    static constexpr size_t MAX_ENTRIES = 10;
    std::mutex mu_;
};
