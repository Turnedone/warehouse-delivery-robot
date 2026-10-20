// Objects shared between the task manager node and the behavior tree, placed on the BT
// blackboard as shared_ptrs. Each is thread-safe: ROS callbacks write them from executor
// threads while the tree reads them from the mission thread.
#pragma once

#include <atomic>
#include <mutex>
#include <set>
#include <string>

namespace wdr_mission {

// Load/unload confirmations from the operator (service /wdr/operator_confirm).
class OperatorPanel {
public:
    void confirm(const std::string& step) {
        std::lock_guard<std::mutex> lock(mutex_);
        confirmed_.insert(step);
    }
    // Consumes a confirmation for `step`, if one has arrived.
    bool take(const std::string& step) {
        std::lock_guard<std::mutex> lock(mutex_);
        return confirmed_.erase(step) > 0;
    }
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        confirmed_.clear();
    }

private:
    std::mutex mutex_;
    std::set<std::string> confirmed_;
};

// Latest battery reading (from /battery_state).
class BatteryMonitor {
public:
    void update(double percentage, bool charging) {
        percentage_.store(percentage);
        charging_.store(charging);
        has_data_.store(true);
    }
    double percentage() const { return percentage_.load(); }
    bool charging() const { return charging_.load(); }
    bool has_data() const { return has_data_.load(); }

private:
    std::atomic<double> percentage_{0.0};
    std::atomic<bool> charging_{false};
    std::atomic<bool> has_data_{false};
};

// Latest robot position in the map frame (from TF, updated by the task manager).
class RobotPose {
public:
    void update(double x, double y) {
        std::lock_guard<std::mutex> lock(mutex_);
        x_ = x;
        y_ = y;
        valid_ = true;
    }
    bool get(double& x, double& y) const {
        std::lock_guard<std::mutex> lock(mutex_);
        x = x_;
        y = y_;
        return valid_;
    }

private:
    mutable std::mutex mutex_;
    double x_ = 0.0;
    double y_ = 0.0;
    bool valid_ = false;
};

// Receives mission state changes from the tree's ReportState nodes.
class MissionReporter {
public:
    virtual ~MissionReporter() = default;
    virtual void report_state(const std::string& state) = 0;
};

}  // namespace wdr_mission
