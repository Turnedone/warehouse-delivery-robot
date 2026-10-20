// Custom BehaviorTree.CPP nodes for the delivery mission. They are built into a plugin
// library (wdr_mission_bt_nodes) that the task manager loads next to Nav2's own BT plugins
// (NavigateToPose, ClearEntireCostmap, Wait, DockRobot, UndockRobot).
//
// Blackboard entries they rely on (set by the task manager):
//   node          rclcpp::Node::SharedPtr                 clock for timeouts
//   stations      std::shared_ptr<StationRegistry>
//   operator      std::shared_ptr<OperatorPanel>
//   battery       std::shared_ptr<BatteryMonitor>
//   reporter      std::shared_ptr<MissionReporter>
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wdr_mission {

// Looks up a station by name and outputs its pose (map frame) as a navigation goal.
//   <GetStationPose station="{pickup}" pose="{pickup_pose}"/>
class GetStationPose : public BT::SyncActionNode {
public:
    GetStationPose(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// Waits for the operator to confirm a load/unload step. A simulated operator confirms after
// auto_confirm_s seconds (negative: never, wait for the /wdr/operator_confirm service).
// Fails after timeout_s seconds (<= 0: wait forever).
//   <WaitForOperator step="load" timeout_s="120" auto_confirm_s="{operator_auto_confirm_s}"/>
class WaitForOperator : public BT::StatefulActionNode {
public:
    WaitForOperator(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time start_;
};

// Waits on the charger until the battery reaches `target` (0..1). Fails after timeout_s.
//   <WaitForCharge target="{resume_battery}" timeout_s="1800"/>
class WaitForCharge : public BT::StatefulActionNode {
public:
    WaitForCharge(const std::string& name, const BT::NodeConfig& config) : BT::StatefulActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override {}

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Time start_;
};

// SUCCESS when the battery is below `threshold` (0..1), or when no battery reading has
// arrived yet: the robot does not start a delivery without knowing its charge.
// Reads the task manager's BatteryMonitor rather than subscribing itself (see engineering log:
// a condition that subscribes inside the tree only sees messages once it has been ticked).
//   <BatteryBelow threshold="{low_battery}"/>
class BatteryBelow : public BT::ConditionNode {
public:
    BatteryBelow(const std::string& name, const BT::NodeConfig& config) : BT::ConditionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

// Tells the task manager which mission state the tree is in (feedback, events, error codes).
//   <ReportState state="TO_PICKUP"/>
class ReportState : public BT::SyncActionNode {
public:
    ReportState(const std::string& name, const BT::NodeConfig& config) : BT::SyncActionNode(name, config) {}
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

}  // namespace wdr_mission
