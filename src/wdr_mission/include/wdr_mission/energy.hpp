// Energy estimate for one order, so the robot charges *before* an order it could not finish
// (rather than only below a fixed threshold). ROS-free and unit tested.
#pragma once

#include <vector>

namespace wdr_mission {

struct EnergyModel {
    double capacity_wh = 26.0;            // battery capacity
    double idle_power_w = 6.0;            // draw while standing (computer, lidar)
    double drive_power_w_per_mps = 25.0;  // extra draw per m/s
    double cruise_speed_mps = 0.30;       // average speed including slow zones and turns
    double detour_factor = 1.4;           // path length / straight-line length (aisles, racks)
    double scale = 1.0;                   // simulation drain multiplier (battery_sim drain_scale)
};

struct Point2D {
    double x;
    double y;
};

// Straight-line length through the points, times the detour factor.
double route_length_m(const EnergyModel& m, const std::vector<Point2D>& points);

// Fraction of a full battery (0..1) needed to drive route_m and stand for wait_s.
double energy_fraction(const EnergyModel& m, double route_m, double wait_s);

}  // namespace wdr_mission
