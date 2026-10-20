#include "wdr_mission/bt_nodes.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "wdr_mission/energy.hpp"
#include "wdr_mission/mission_context.hpp"
#include "wdr_mission/station_registry.hpp"

namespace wdr_mission {

namespace {

template <typename T>
std::shared_ptr<T> from_blackboard(const BT::TreeNode& node, const char* key) {
    auto value = node.config().blackboard->get<std::shared_ptr<T>>(key);
    if (!value) throw BT::RuntimeError(node.name(), ": blackboard entry '", key, "' is empty");
    return value;
}

rclcpp::Node::SharedPtr ros_node(const BT::TreeNode& node) {
    return node.config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

}  // namespace

// ---------------------------------------------------------------- GetStationPose

BT::PortsList GetStationPose::providedPorts() {
    return {BT::InputPort<std::string>("station", "Station name from stations.yaml"),
            BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Station pose in the map frame")};
}

BT::NodeStatus GetStationPose::tick() {
    const auto name = getInput<std::string>("station");
    if (!name) throw BT::RuntimeError("GetStationPose: missing 'station': ", name.error());
    const auto registry = from_blackboard<StationRegistry>(*this, "stations");
    const auto station = registry->find(name.value());
    if (!station) {
        RCLCPP_ERROR(ros_node(*this)->get_logger(), "unknown station '%s'", name.value().c_str());
        return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = registry->frame_id();
    pose.header.stamp = ros_node(*this)->now();
    pose.pose.position.x = station->x;
    pose.pose.position.y = station->y;
    pose.pose.orientation.z = std::sin(station->yaw / 2.0);
    pose.pose.orientation.w = std::cos(station->yaw / 2.0);
    setOutput("pose", pose);
    return BT::NodeStatus::SUCCESS;
}

// ---------------------------------------------------------------- WaitForOperator

BT::PortsList WaitForOperator::providedPorts() {
    return {BT::InputPort<std::string>("step", "load or unload"),
            BT::InputPort<double>("timeout_s", 120.0, "Fail after this long (<= 0: never)"),
            BT::InputPort<double>("auto_confirm_s", -1.0, "Simulated operator delay (< 0: off)")};
}

BT::NodeStatus WaitForOperator::onStart() {
    node_ = ros_node(*this);
    start_ = node_->now();
    RCLCPP_INFO(node_->get_logger(), "waiting for the operator to confirm '%s'",
                getInput<std::string>("step").value_or("?").c_str());
    return onRunning();
}

BT::NodeStatus WaitForOperator::onRunning() {
    const std::string step = getInput<std::string>("step").value_or("");
    const double timeout = getInput<double>("timeout_s").value_or(120.0);
    const double auto_confirm = getInput<double>("auto_confirm_s").value_or(-1.0);
    const double waited = (node_->now() - start_).seconds();

    if (from_blackboard<OperatorPanel>(*this, "operator")->take(step)) {
        RCLCPP_INFO(node_->get_logger(), "operator confirmed '%s'", step.c_str());
        return BT::NodeStatus::SUCCESS;
    }
    if (auto_confirm >= 0.0 && waited >= auto_confirm) {
        RCLCPP_INFO(node_->get_logger(), "simulated operator confirmed '%s' after %.1f s", step.c_str(), waited);
        return BT::NodeStatus::SUCCESS;
    }
    if (timeout > 0.0 && waited >= timeout) {
        RCLCPP_WARN(node_->get_logger(), "no '%s' confirmation within %.0f s", step.c_str(), timeout);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------- WaitForCharge

BT::PortsList WaitForCharge::providedPorts() {
    return {BT::InputPort<double>("target", 0.9, "Battery fraction to wait for"),
            BT::InputPort<double>("timeout_s", 3600.0, "Fail after this long (<= 0: never)")};
}

BT::NodeStatus WaitForCharge::onStart() {
    node_ = ros_node(*this);
    start_ = node_->now();
    return onRunning();
}

BT::NodeStatus WaitForCharge::onRunning() {
    const double target = getInput<double>("target").value_or(0.9);
    const double timeout = getInput<double>("timeout_s").value_or(3600.0);
    const auto battery = from_blackboard<BatteryMonitor>(*this, "battery");
    if (battery->has_data() && battery->percentage() >= target) {
        RCLCPP_INFO(node_->get_logger(), "charged to %.0f%%", battery->percentage() * 100.0);
        return BT::NodeStatus::SUCCESS;
    }
    const double waited = (node_->now() - start_).seconds();
    if (timeout > 0.0 && waited >= timeout) {
        RCLCPP_ERROR(node_->get_logger(), "battery did not reach %.0f%% within %.0f s", target * 100.0, timeout);
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}

// ---------------------------------------------------------------- BatteryBelow

BT::PortsList BatteryBelow::providedPorts() {
    return {BT::InputPort<double>("threshold", "Battery fraction (0..1)")};
}

BT::NodeStatus BatteryBelow::tick() {
    const double threshold = getInput<double>("threshold").value_or(0.3);
    const auto battery = from_blackboard<BatteryMonitor>(*this, "battery");
    if (!battery->has_data()) return BT::NodeStatus::SUCCESS;
    return battery->percentage() < threshold ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------- EnoughBatteryForOrder

BT::PortsList EnoughBatteryForOrder::providedPorts() {
    return {BT::InputPort<double>("reserve", 0.15, "Charge to keep after the order and the drive back")};
}

BT::NodeStatus EnoughBatteryForOrder::tick() {
    const double reserve = getInput<double>("reserve").value_or(0.15);
    const auto battery = from_blackboard<BatteryMonitor>(*this, "battery");
    const auto stations = from_blackboard<StationRegistry>(*this, "stations");
    const auto model = from_blackboard<EnergyModel>(*this, "energy");
    const auto& bb = *config().blackboard;
    if (!battery->has_data()) return BT::NodeStatus::FAILURE;

    double rx = 0.0;
    double ry = 0.0;
    std::vector<Point2D> route;
    if (from_blackboard<RobotPose>(*this, "robot_pose")->get(rx, ry)) route.push_back({rx, ry});
    const bool loaded = bb.get<bool>("loaded");
    for (const auto& key : {std::string("pickup"), std::string("dropoff"), std::string("dock_id")}) {
        if (key == "pickup" && loaded) continue;
        const auto st = stations->find(bb.get<std::string>(key));
        if (st) route.push_back({st->x, st->y});
    }
    const double wait_s = 2.0 * std::max(0.0, bb.get<double>("operator_auto_confirm_s"));
    const double route_m = route_length_m(*model, route);
    const double needed = energy_fraction(*model, route_m, wait_s);
    const double have = battery->percentage();
    if (have - needed >= reserve) return BT::NodeStatus::SUCCESS;
    RCLCPP_WARN(ros_node(*this)->get_logger(),
                "battery %.0f%% is not enough for this order: ~%.0f m needs ~%.0f%%, plus %.0f%% reserve", have * 100.0,
                route_m, needed * 100.0, reserve * 100.0);
    return BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------- ReportState

BT::PortsList ReportState::providedPorts() {
    return {BT::InputPort<std::string>("state", "Mission state name")};
}

BT::NodeStatus ReportState::tick() {
    const auto s = getInput<std::string>("state");
    if (!s) throw BT::RuntimeError("ReportState: missing 'state': ", s.error());
    from_blackboard<MissionReporter>(*this, "reporter")->report_state(s.value());
    return BT::NodeStatus::SUCCESS;
}

}  // namespace wdr_mission

BT_REGISTER_NODES(factory) {
    factory.registerNodeType<wdr_mission::GetStationPose>("GetStationPose");
    factory.registerNodeType<wdr_mission::WaitForOperator>("WaitForOperator");
    factory.registerNodeType<wdr_mission::WaitForCharge>("WaitForCharge");
    factory.registerNodeType<wdr_mission::BatteryBelow>("BatteryBelow");
    factory.registerNodeType<wdr_mission::EnoughBatteryForOrder>("EnoughBatteryForOrder");
    factory.registerNodeType<wdr_mission::ReportState>("ReportState");
}
