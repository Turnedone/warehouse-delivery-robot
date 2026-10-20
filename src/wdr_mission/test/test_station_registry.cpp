#include <gtest/gtest.h>

#include <stdexcept>

#include "wdr_mission/mission_state.hpp"
#include "wdr_mission/station_registry.hpp"
#include "wdr_interfaces/action/deliver_order.hpp"

using wdr_mission::StationRegistry;
using Result = wdr_interfaces::action::DeliverOrder::Result;

namespace {
const char* kStations = R"(
frame_id: map
stations:
  shelf_a1: {type: pickup, x: 6.0, y: 11.2, yaw: 1.5708}
  pack_1: {type: dropoff, x: 22.0, y: 12.5, yaw: 0.0}
  home_dock: {type: dock, x: 0.75, y: 1.5, yaw: 3.14159}
)";
}  // namespace

TEST(StationRegistry, ParsesStations) {
    const auto r = StationRegistry::parse(kStations);
    const auto s = r.find("shelf_a1");
    ASSERT_TRUE(s.has_value());
    EXPECT_DOUBLE_EQ(s->x, 6.0);
    EXPECT_DOUBLE_EQ(s->y, 11.2);
    EXPECT_EQ(s->type, "pickup");
    EXPECT_EQ(r.frame_id(), "map");
    EXPECT_EQ(r.names().size(), 3u);
    EXPECT_EQ(r.names("dropoff"), std::vector<std::string>{"pack_1"});
}

TEST(StationRegistry, ValidatesOrders) {
    const auto r = StationRegistry::parse(kStations);
    EXPECT_EQ(r.validate_order("shelf_a1", "pack_1"), "");
    EXPECT_NE(r.validate_order("nowhere", "pack_1").find("unknown pickup"), std::string::npos);
    EXPECT_NE(r.validate_order("shelf_a1", "nowhere").find("unknown dropoff"), std::string::npos);
    EXPECT_NE(r.validate_order("pack_1", "pack_1").find("not a pickup"), std::string::npos);
    EXPECT_NE(r.validate_order("shelf_a1", "home_dock").find("not a dropoff"), std::string::npos);
}

TEST(StationRegistry, RejectsMalformedFiles) {
    EXPECT_THROW(StationRegistry::parse("foo: bar"), std::runtime_error);
    EXPECT_THROW(StationRegistry::parse("stations:\n  s1: {type: pickup, x: 1.0}\n"), std::runtime_error);
    EXPECT_THROW(StationRegistry::load("/nonexistent/stations.yaml"), std::runtime_error);
}

TEST(MissionState, FailurePhaseMapsToResultCode) {
    using wdr_mission::error_code_for_phase;
    EXPECT_EQ(error_code_for_phase("TO_PICKUP"), Result::PICKUP_UNREACHABLE);
    EXPECT_EQ(error_code_for_phase("WAITING_FOR_LOAD"), Result::LOAD_TIMEOUT);
    EXPECT_EQ(error_code_for_phase("TO_DROPOFF"), Result::DROPOFF_UNREACHABLE);
    EXPECT_EQ(error_code_for_phase("WAITING_FOR_UNLOAD"), Result::UNLOAD_TIMEOUT);
    EXPECT_EQ(error_code_for_phase("CHARGING"), Result::CHARGING_FAILED);
    EXPECT_EQ(error_code_for_phase("UNDOCKING"), Result::CHARGING_FAILED);
    EXPECT_EQ(error_code_for_phase(""), Result::UNKNOWN);
}

TEST(MissionState, RetriesDoNotChangeThePhase) {
    EXPECT_FALSE(wdr_mission::is_phase_state("NAV_RETRY"));
    EXPECT_FALSE(wdr_mission::is_phase_state("TO_EXCEPTION_AREA"));
    EXPECT_TRUE(wdr_mission::is_phase_state("TO_DROPOFF"));
}
