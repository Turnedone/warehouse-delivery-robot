#include "wdr_sim/path_player.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace wdr_sim {

PathPlayer::PathPlayer(std::vector<Point> path, double speed) : path_(std::move(path)), speed_(speed) {
    if (path_.size() < 2) throw std::invalid_argument("PathPlayer needs at least two points");
    if (!(speed_ > 0.0)) throw std::invalid_argument("PathPlayer speed must be positive");
    cumulative_.push_back(0.0);
    for (size_t i = 1; i < path_.size(); ++i) {
        total_length_ += std::hypot(path_[i].x - path_[i - 1].x, path_[i].y - path_[i - 1].y);
        cumulative_.push_back(total_length_);
    }
    if (total_length_ <= 0.0) throw std::invalid_argument("PathPlayer path has zero length");
}

Pose2D PathPlayer::pose_at(double t_s) const {
    const double period = 2.0 * total_length_;
    double s = std::fmod(std::max(0.0, t_s) * speed_, period);
    const bool returning = s > total_length_;
    if (returning) s = period - s;

    size_t i = 1;
    while (i + 1 < path_.size() && cumulative_[i] < s) ++i;
    const Point& a = path_[i - 1];
    const Point& b = path_[i];
    const double seg = cumulative_[i] - cumulative_[i - 1];
    const double f = seg > 0.0 ? (s - cumulative_[i - 1]) / seg : 0.0;

    Pose2D pose;
    pose.x = a.x + f * (b.x - a.x);
    pose.y = a.y + f * (b.y - a.y);
    pose.yaw = std::atan2(b.y - a.y, b.x - a.x);
    if (returning) pose.yaw = std::atan2(a.y - b.y, a.x - b.x);
    return pose;
}

}  // namespace wdr_sim
