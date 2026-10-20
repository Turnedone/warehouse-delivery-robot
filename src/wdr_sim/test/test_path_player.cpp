#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>

#include "wdr_sim/path_player.hpp"

using wdr_sim::PathPlayer;

TEST(PathPlayer, GoesOutAndBack) {
    PathPlayer p({{0.0, 0.0}, {10.0, 0.0}}, 1.0);
    EXPECT_NEAR(p.pose_at(0.0).x, 0.0, 1e-9);
    EXPECT_NEAR(p.pose_at(4.0).x, 4.0, 1e-9);
    EXPECT_NEAR(p.pose_at(4.0).yaw, 0.0, 1e-9);
    EXPECT_NEAR(p.pose_at(14.0).x, 6.0, 1e-9);  // on the way back
    EXPECT_NEAR(std::abs(p.pose_at(14.0).yaw), M_PI, 1e-9);
    EXPECT_NEAR(p.pose_at(20.0).x, 0.0, 1e-9);  // one full period
}

TEST(PathPlayer, FollowsCorners) {
    PathPlayer p({{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}}, 1.0);
    EXPECT_DOUBLE_EQ(p.length(), 4.0);
    const auto pose = p.pose_at(3.0);
    EXPECT_NEAR(pose.x, 2.0, 1e-9);
    EXPECT_NEAR(pose.y, 1.0, 1e-9);
    EXPECT_NEAR(pose.yaw, M_PI / 2.0, 1e-9);
}

TEST(PathPlayer, RejectsBadInput) {
    EXPECT_THROW(PathPlayer({{0.0, 0.0}}, 1.0), std::invalid_argument);
    EXPECT_THROW(PathPlayer({{0.0, 0.0}, {1.0, 0.0}}, 0.0), std::invalid_argument);
    EXPECT_THROW(PathPlayer({{1.0, 1.0}, {1.0, 1.0}}, 1.0), std::invalid_argument);
}
