#include <gtest/gtest.h>

#include "wdr_sim/battery_model.hpp"

using wdr_sim::BatteryModel;
using wdr_sim::BatteryParams;

TEST(BatteryModel, IdleDrainMatchesEnergyBudget) {
    BatteryParams p;
    p.capacity_wh = 10.0;
    p.idle_power_w = 10.0;
    BatteryModel b(p, 1.0);
    b.update(1800.0, 0.0, false);  // half an hour at 10 W from 10 Wh
    EXPECT_NEAR(b.soc(), 0.5, 1e-9);
    EXPECT_LT(b.current_a(), 0.0);
    EXPECT_FALSE(b.charging());
}

TEST(BatteryModel, DrivingDrainsFasterThanIdling) {
    BatteryParams p;
    BatteryModel idle(p, 1.0);
    BatteryModel driving(p, 1.0);
    idle.update(60.0, 0.0, false);
    driving.update(60.0, 0.5, false);
    EXPECT_LT(driving.soc(), idle.soc());
}

TEST(BatteryModel, DrainScaleSpeedsUpDischarge) {
    BatteryParams p;
    BatteryModel normal(p, 1.0);
    p.drain_scale = 10.0;
    BatteryModel fast(p, 1.0);
    normal.update(60.0, 0.0, false);
    fast.update(60.0, 0.0, false);
    EXPECT_NEAR(1.0 - fast.soc(), 10.0 * (1.0 - normal.soc()), 1e-9);
}

TEST(BatteryModel, ChargesOnDockWithPositiveCurrent) {
    BatteryParams p;
    BatteryModel b(p, 0.2);
    b.update(60.0, 0.0, true);
    EXPECT_GT(b.soc(), 0.2);
    EXPECT_TRUE(b.charging());
    EXPECT_GT(b.current_a(), 0.5);  // above SimpleChargingDock's default charging threshold
}

TEST(BatteryModel, StaysWithinBoundsAndStopsChargingWhenFull) {
    BatteryParams p;
    BatteryModel b(p, 0.99);
    b.update(1e6, 0.0, true);
    EXPECT_DOUBLE_EQ(b.soc(), 1.0);
    b.update(1.0, 0.0, true);
    EXPECT_FALSE(b.charging());
    EXPECT_DOUBLE_EQ(b.current_a(), 0.0);
    b.update(1e7, 1.0, false);
    EXPECT_DOUBLE_EQ(b.soc(), 0.0);
}

TEST(BatteryModel, VoltageFollowsStateOfCharge) {
    BatteryParams p;
    BatteryModel b(p, 0.0);
    EXPECT_DOUBLE_EQ(b.voltage(), p.empty_voltage);
    b.set_soc(1.0);
    EXPECT_DOUBLE_EQ(b.voltage(), p.full_voltage);
    b.set_soc(7.0);  // clamped
    EXPECT_DOUBLE_EQ(b.soc(), 1.0);
}
