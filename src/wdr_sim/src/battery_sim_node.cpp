// Simulated battery: drains with time and speed, charges while the robot sits on the dock.
//
// Publishes sensor_msgs/BatteryState on /battery_state, the topic both Nav2's IsBatteryLow
// BT condition and opennav_docking's SimpleChargingDock (use_battery_status) listen to.
// "On the dock" is decided from TF (map -> base_link within dock_radius of the dock pose),
// standing in for the charging contacts of a real dock.

#include <chrono>
#include <cmath>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wdr_interfaces/srv/set_battery_level.hpp"
#include "wdr_sim/battery_model.hpp"

namespace wdr_sim {

class BatterySimNode : public rclcpp::Node {
public:
    BatterySimNode() : Node("battery_sim") {
        BatteryParams p;
        p.capacity_wh = declare_parameter("capacity_wh", p.capacity_wh);
        p.idle_power_w = declare_parameter("idle_power_w", p.idle_power_w);
        p.drive_power_w_per_mps = declare_parameter("drive_power_w_per_mps", p.drive_power_w_per_mps);
        p.charge_power_w = declare_parameter("charge_power_w", p.charge_power_w);
        p.drain_scale = declare_parameter("drain_scale", p.drain_scale);
        p.charge_scale = declare_parameter("charge_scale", p.charge_scale);
        const double initial = declare_parameter("initial_percentage", 1.0);
        dock_x_ = declare_parameter("dock_x", 0.0);
        dock_y_ = declare_parameter("dock_y", 0.0);
        dock_radius_ = declare_parameter("dock_radius", 0.12);
        global_frame_ = declare_parameter("global_frame", std::string("map"));
        base_frame_ = declare_parameter("base_frame", std::string("base_link"));
        const double rate_hz = declare_parameter("rate_hz", 5.0);

        model_ = std::make_unique<BatteryModel>(p, initial);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        pub_ = create_publisher<sensor_msgs::msg::BatteryState>("battery_state", rclcpp::SystemDefaultsQoS());
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry& msg) {
                speed_ = std::hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y);
            });
        set_srv_ = create_service<wdr_interfaces::srv::SetBatteryLevel>(
            "~/set_level", [this](const std::shared_ptr<wdr_interfaces::srv::SetBatteryLevel::Request> req,
                                  std::shared_ptr<wdr_interfaces::srv::SetBatteryLevel::Response> res) {
                if (req->percentage < 0.0F || req->percentage > 1.0F) {
                    res->success = false;
                    res->message = "percentage must be between 0.0 and 1.0";
                    return;
                }
                model_->set_soc(req->percentage);
                res->success = true;
                res->message = "battery set";
                RCLCPP_INFO(get_logger(), "battery set to %.0f%%", req->percentage * 100.0);
            });
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this] { tick(); });
    }

private:
    bool on_charger() {
        try {
            const auto t = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
            return std::hypot(t.transform.translation.x - dock_x_, t.transform.translation.y - dock_y_) <= dock_radius_;
        } catch (const tf2::TransformException&) {
            return false;  // no localization yet: not on the charger
        }
    }

    void tick() {
        const rclcpp::Time now = get_clock()->now();
        // dt from the ROS clock (sim time in simulation), so faster-than-real-time runs drain
        // the battery by simulated time, not wall time.
        const double dt = last_.nanoseconds() == 0 ? 0.0 : (now - last_).seconds();
        last_ = now;
        const bool charger = on_charger();
        if (dt > 0.0 && dt < 5.0) model_->update(dt, speed_, charger);

        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp = now;
        msg.percentage = static_cast<float>(model_->soc());
        msg.voltage = static_cast<float>(model_->voltage());
        msg.current = static_cast<float>(model_->current_a());
        msg.design_capacity = 26.0F / 14.4F;  // Ah
        msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
        msg.present = true;
        if (model_->charging()) {
            msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        } else if (charger) {
            msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
        } else {
            msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        }
        pub_->publish(msg);
    }

    std::unique_ptr<BatteryModel> model_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<wdr_interfaces::srv::SetBatteryLevel>::SharedPtr set_srv_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_{0, 0, RCL_ROS_TIME};
    double speed_ = 0.0;
    double dock_x_ = 0.0;
    double dock_y_ = 0.0;
    double dock_radius_ = 0.12;
    std::string global_frame_;
    std::string base_frame_;
};

}  // namespace wdr_sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int code = 0;
    try {
        rclcpp::spin(std::make_shared<wdr_sim::BatterySimNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("battery_sim"), "%s", e.what());
        code = 1;
    }
    rclcpp::shutdown();
    return code;
}
