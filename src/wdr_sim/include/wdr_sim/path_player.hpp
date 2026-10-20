// Moves a point back and forth along a polyline at constant speed (for moving obstacles).
#pragma once

#include <vector>

namespace wdr_sim {

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
};

class PathPlayer {
public:
    struct Point {
        double x;
        double y;
    };

    // path needs at least two points; speed in m/s (> 0).
    PathPlayer(std::vector<Point> path, double speed);

    // Pose after t seconds: out along the path, then back, repeating. yaw faces the direction of
    // travel.
    Pose2D pose_at(double t_s) const;
    double length() const { return total_length_; }

private:
    std::vector<Point> path_;
    std::vector<double> cumulative_;  // distance from the start to each point
    double speed_;
    double total_length_ = 0.0;
};

}  // namespace wdr_sim
