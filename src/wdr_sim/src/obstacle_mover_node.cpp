// Moves the warehouse's dynamic obstacles (forklift, picking cart, worker) back and forth along
// the paths in obstacles.yaml by calling Gazebo's set_pose service through ros_gz_bridge.
//
// The obstacles are not in the static map, so Nav2 only knows about them through the lidar:
// they exercise the costmaps' obstacle layers, MPPI's local avoidance and global replanning.
// Their true poses are also published as markers (/wdr/obstacles) for RViz and the rosbag.

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wdr_sim/path_player.hpp"

namespace wdr_sim {

class ObstacleMoverNode : public rclcpp::Node {
public:
    ObstacleMoverNode() : Node("obstacle_mover") {
        const auto file = declare_parameter("obstacles_file", std::string());
        const double rate_hz = declare_parameter("rate_hz", 10.0);
        const YAML::Node root = YAML::LoadFile(file);
        const auto world = root["world_name"].as<std::string>();
        for (const auto& ob : root["obstacles"]) {
            std::vector<PathPlayer::Point> path;
            for (const auto& p : ob["path"]) path.push_back({p[0].as<double>(), p[1].as<double>()});
            obstacles_.push_back(
                {ob["name"].as<std::string>(), ob["height"].as<double>(), PathPlayer(path, ob["speed"].as<double>())});
        }
        client_ = create_client<SetEntityPose>("/world/" + world + "/set_pose");
        markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/wdr/obstacles", 10);
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate_hz), [this] { tick(); });
        RCLCPP_INFO(get_logger(), "moving %zu obstacles in world '%s'", obstacles_.size(), world.c_str());
    }

private:
    using SetEntityPose = ros_gz_interfaces::srv::SetEntityPose;

    struct Obstacle {
        std::string name;
        double height;
        PathPlayer player;
    };

    void tick() {
        const rclcpp::Time now = get_clock()->now();
        if (now.nanoseconds() == 0) return;  // no /clock yet
        if (start_.nanoseconds() == 0) start_ = now;
        const double t = (now - start_).seconds();
        if (!client_->service_is_ready()) return;
        // At most one batch of requests in flight: if Gazebo falls behind, skip an update
        // instead of queueing a backlog of stale poses.
        if (pending_ > 0) return;

        visualization_msgs::msg::MarkerArray markers;
        int id = 0;
        for (const auto& ob : obstacles_) {
            const Pose2D p = ob.player.pose_at(t);
            auto req = std::make_shared<SetEntityPose::Request>();
            req->entity.name = ob.name;
            req->entity.type = ros_gz_interfaces::msg::Entity::MODEL;
            req->pose.position.x = p.x;
            req->pose.position.y = p.y;
            req->pose.position.z = ob.height / 2.0;
            req->pose.orientation.z = std::sin(p.yaw / 2.0);
            req->pose.orientation.w = std::cos(p.yaw / 2.0);
            ++pending_;
            client_->async_send_request(req, [this](rclcpp::Client<SetEntityPose>::SharedFuture) { --pending_; });

            visualization_msgs::msg::Marker m;
            m.header.frame_id = "map";
            m.header.stamp = now;
            m.ns = "moving_obstacles";
            m.id = id++;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.pose = req->pose;
            m.scale.x = 0.6;
            m.scale.y = 0.6;
            m.scale.z = ob.height;
            m.color.r = 1.0F;
            m.color.g = 0.4F;
            m.color.a = 0.6F;
            m.text = ob.name;
            markers.markers.push_back(m);
        }
        markers_pub_->publish(markers);
    }

    std::vector<Obstacle> obstacles_;
    rclcpp::Client<SetEntityPose>::SharedPtr client_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_{0, 0, RCL_ROS_TIME};
    int pending_ = 0;
};

}  // namespace wdr_sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wdr_sim::ObstacleMoverNode>());
    rclcpp::shutdown();
    return 0;
}
