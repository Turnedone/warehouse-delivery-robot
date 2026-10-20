// Priority queue of delivery orders: highest priority first, first come first served within
// a priority. Kept free of ROS (the Handle is whatever the caller wants to carry along, the
// task manager stores its action goal handle) so the ordering rules are unit tested directly.
#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace wdr_mission {

struct Order {
    std::string id;
    std::string pickup;
    std::string dropoff;
    uint8_t priority = 0;
    double enqueue_time_s = 0.0;
};

template <typename Handle>
class OrderQueue {
public:
    struct Entry {
        Order order;
        Handle handle;
        uint64_t seq;  // arrival order, breaks priority ties
    };

    explicit OrderQueue(std::size_t capacity) : capacity_(capacity) {}

    // False if the queue is full.
    bool push(Order order, Handle handle) {
        if (entries_.size() >= capacity_) return false;
        entries_.push_back({std::move(order), std::move(handle), next_seq_++});
        sort();
        return true;
    }

    // Puts a popped entry back where it was (its original arrival number), for an order that
    // was interrupted before it could finish. Ignores the capacity: the order was already accepted.
    void restore(Entry entry) {
        entries_.push_back(std::move(entry));
        sort();
    }

    std::optional<Entry> pop() {
        if (entries_.empty()) return std::nullopt;
        Entry e = std::move(entries_.front());
        entries_.erase(entries_.begin());
        return e;
    }

    // Removes and returns every entry matching pred (for example, orders whose goal is being
    // canceled while they wait).
    std::vector<Entry> remove_if(const std::function<bool(const Entry&)>& pred) {
        std::vector<Entry> removed;
        auto it = entries_.begin();
        while (it != entries_.end()) {
            if (pred(*it)) {
                removed.push_back(std::move(*it));
                it = entries_.erase(it);
            } else {
                ++it;
            }
        }
        return removed;
    }

    // 1-based position in the queue, 0 if not queued.
    std::size_t position(const std::string& order_id) const {
        for (std::size_t i = 0; i < entries_.size(); ++i) {
            if (entries_[i].order.id == order_id) return i + 1;
        }
        return 0;
    }

    bool contains(const std::string& order_id) const { return position(order_id) != 0; }
    std::size_t size() const { return entries_.size(); }
    bool empty() const { return entries_.empty(); }
    std::size_t capacity() const { return capacity_; }

private:
    void sort() {
        std::stable_sort(entries_.begin(), entries_.end(), [](const Entry& a, const Entry& b) {
            if (a.order.priority != b.order.priority) return a.order.priority > b.order.priority;
            return a.seq < b.seq;
        });
    }

    std::size_t capacity_;
    std::vector<Entry> entries_;
    uint64_t next_seq_ = 0;
};

}  // namespace wdr_mission
