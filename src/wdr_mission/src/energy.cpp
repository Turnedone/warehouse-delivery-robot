#include "wdr_mission/energy.hpp"

#include <cmath>

namespace wdr_mission {

double route_length_m(const EnergyModel& m, const std::vector<Point2D>& points) {
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
    }
    return length * m.detour_factor;
}

double energy_fraction(const EnergyModel& m, double route_m, double wait_s) {
    const double drive_s = m.cruise_speed_mps > 0.0 ? route_m / m.cruise_speed_mps : 0.0;
    const double drive_j = drive_s * (m.idle_power_w + m.drive_power_w_per_mps * m.cruise_speed_mps);
    const double wait_j = wait_s * m.idle_power_w;
    return (drive_j + wait_j) * m.scale / (m.capacity_wh * 3600.0);
}

}  // namespace wdr_mission
