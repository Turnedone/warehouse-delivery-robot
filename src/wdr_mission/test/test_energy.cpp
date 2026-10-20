#include <gtest/gtest.h>

#include "wdr_mission/energy.hpp"

using wdr_mission::energy_fraction;
using wdr_mission::EnergyModel;
using wdr_mission::route_length_m;

TEST(Energy, RouteLengthIncludesDetour) {
    EnergyModel m;
    m.detour_factor = 1.5;
    EXPECT_DOUBLE_EQ(route_length_m(m, {{0, 0}, {3, 4}, {3, 10}}), (5.0 + 6.0) * 1.5);
    EXPECT_DOUBLE_EQ(route_length_m(m, {{1, 1}}), 0.0);
}

TEST(Energy, FractionFollowsThePowerModel) {
    EnergyModel m;
    m.capacity_wh = 10.0;  // 36 kJ
    m.idle_power_w = 10.0;
    m.drive_power_w_per_mps = 0.0;
    m.cruise_speed_mps = 1.0;
    // 360 m at 1 m/s = 360 s at 10 W = 3.6 kJ = 10 % of the battery
    EXPECT_NEAR(energy_fraction(m, 360.0, 0.0), 0.10, 1e-12);
    // plus 360 s of waiting at 10 W: another 10 %
    EXPECT_NEAR(energy_fraction(m, 360.0, 360.0), 0.20, 1e-12);
}

TEST(Energy, DrainScaleScalesTheEstimate) {
    EnergyModel m;
    const double normal = energy_fraction(m, 50.0, 10.0);
    m.scale = 15.0;
    EXPECT_NEAR(energy_fraction(m, 50.0, 10.0), 15.0 * normal, 1e-12);
}

TEST(Energy, TypicalOrderIsSmallAtRealDrain) {
    // A 50 m order at real drain: ~167 s at 13.5 W = ~2.2 kJ, about 2.4 % of the 26 Wh battery
    EnergyModel m;
    EXPECT_NEAR(energy_fraction(m, 50.0, 10.0), 0.024, 0.001);
}
