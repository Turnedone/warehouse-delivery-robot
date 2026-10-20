#include <gtest/gtest.h>

#include "wdr_mission/order_queue.hpp"

using wdr_mission::Order;
using wdr_mission::OrderQueue;

namespace {
Order order(const std::string& id, uint8_t priority = 0) { return Order{id, "shelf_a1", "pack_1", priority, 0.0}; }
}  // namespace

TEST(OrderQueue, FirstComeFirstServed) {
    OrderQueue<int> q(10);
    q.push(order("a"), 1);
    q.push(order("b"), 2);
    q.push(order("c"), 3);
    EXPECT_EQ(q.pop()->order.id, "a");
    EXPECT_EQ(q.pop()->order.id, "b");
    EXPECT_EQ(q.pop()->order.id, "c");
    EXPECT_FALSE(q.pop().has_value());
}

TEST(OrderQueue, HigherPriorityJumpsAheadButKeepsFifoWithinPriority) {
    OrderQueue<int> q(10);
    q.push(order("low1"), 0);
    q.push(order("high1", 5), 0);
    q.push(order("low2"), 0);
    q.push(order("high2", 5), 0);
    EXPECT_EQ(q.position("high1"), 1u);
    EXPECT_EQ(q.position("high2"), 2u);
    EXPECT_EQ(q.position("low1"), 3u);
    EXPECT_EQ(q.pop()->order.id, "high1");
    EXPECT_EQ(q.pop()->order.id, "high2");
    EXPECT_EQ(q.pop()->order.id, "low1");
    EXPECT_EQ(q.pop()->order.id, "low2");
}

TEST(OrderQueue, RejectsWhenFull) {
    OrderQueue<int> q(2);
    EXPECT_TRUE(q.push(order("a"), 0));
    EXPECT_TRUE(q.push(order("b"), 0));
    EXPECT_FALSE(q.push(order("c"), 0));
    EXPECT_EQ(q.size(), 2u);
}

TEST(OrderQueue, RemoveIfTakesOutCanceledOrders) {
    OrderQueue<int> q(10);
    q.push(order("a"), 1);
    q.push(order("b"), 2);
    q.push(order("c"), 3);
    const auto removed = q.remove_if([](const auto& e) { return e.handle == 2; });
    ASSERT_EQ(removed.size(), 1u);
    EXPECT_EQ(removed[0].order.id, "b");
    EXPECT_FALSE(q.contains("b"));
    EXPECT_EQ(q.position("c"), 2u);
}

TEST(OrderQueue, HandleTravelsWithTheOrder) {
    OrderQueue<std::string> q(10);
    q.push(order("a"), "handle-a");
    EXPECT_EQ(q.pop()->handle, "handle-a");
}
