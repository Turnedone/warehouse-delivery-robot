// Task manager: accepts DeliverOrder goals, queues them, and runs the delivery behavior tree
// for one order at a time.
//
//   /deliver_order        wdr_interfaces/action/DeliverOrder   orders in, results out
//   /wdr/order_events     wdr_interfaces/msg/OrderEvent        every state change (for bags)
//   /wdr/operator_confirm wdr_interfaces/srv/OperatorConfirm   operator: loaded / unloaded
//   /battery_state        sensor_msgs/msg/BatteryState         (subscribed)
//
// Threads: the ROS executor (action server callbacks, subscriptions, the confirm service)
// and one mission thread that pops orders and ticks the tree. Goals are accepted with
// ACCEPT_AND_DEFER: a queued order stays in the ACCEPTED state until the mission thread
// starts it, so clients can tell "queued" from "executing" from the goal status alone.

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wdr_interfaces/action/deliver_order.hpp"
#include "wdr_interfaces/msg/order_event.hpp"
#include "wdr_interfaces/srv/operator_confirm.hpp"
#include "wdr_mission/mission_context.hpp"
#include "wdr_mission/mission_state.hpp"
#include "wdr_mission/order_queue.hpp"
#include "wdr_mission/station_registry.hpp"

namespace wdr_mission {

using DeliverOrder = wdr_interfaces::action::DeliverOrder;
using GoalHandle = rclcpp_action::ServerGoalHandle<DeliverOrder>;
using OrderEvent = wdr_interfaces::msg::OrderEvent;
using OperatorConfirm = wdr_interfaces::srv::OperatorConfirm;
using namespace std::chrono_literals;

// Nav2's engine, plus access to its factory so several trees can be created from one file.
class MissionEngine : public nav2_behavior_tree::BehaviorTreeEngine {
public:
    using BehaviorTreeEngine::BehaviorTreeEngine;
    void register_trees(const std::string& file) { factory_.registerBehaviorTreeFromFile(file); }
    BT::Tree create(const std::string& tree_id, const BT::Blackboard::Ptr& blackboard) {
        return factory_.createTree(tree_id, blackboard);
    }
};

class TaskManager : public rclcpp::Node {
public:
    TaskManager() : Node("task_manager") {
        stations_file_ = declare_parameter("stations_file", std::string());
        tree_file_ = declare_parameter("behavior_tree", std::string());
        plugins_ = declare_parameter("plugin_lib_names", std::vector<std::string>{});
        loop_ms_ = declare_parameter("bt_loop_duration_ms", 10);
        server_timeout_ms_ = declare_parameter("server_timeout_ms", 20);
        wait_for_service_ms_ = declare_parameter("wait_for_service_timeout_ms", 1000);
        queue_capacity_ = declare_parameter("queue_capacity", 20);
        low_battery_ = declare_parameter("low_battery", 0.30);
        resume_battery_ = declare_parameter("resume_battery", 0.90);
        nav_attempts_ = declare_parameter("nav_attempts", 3);
        operator_timeout_s_ = declare_parameter("operator_timeout_s", 120.0);
        operator_auto_confirm_s_ = declare_parameter("operator_auto_confirm_s", 3.0);
        charge_timeout_s_ = declare_parameter("charge_timeout_s", 3600.0);
        dock_id_ = declare_parameter("dock_id", std::string("home_dock"));
        exception_station_ = declare_parameter("exception_station", std::string("qa_hold"));
        start_docked_ = declare_parameter("start_docked", true);
        idle_dock_after_s_ = declare_parameter("idle_dock_after_s", 10.0);
        groot_port_ = declare_parameter("groot_port", 0);
        nav2_managers_ = declare_parameter(
            "nav2_lifecycle_managers",
            std::vector<std::string>{"lifecycle_manager_localization", "lifecycle_manager_navigation"});
    }

    // Second construction step: needs shared_from_this().
    void init() {
        stations_ = std::make_shared<StationRegistry>(StationRegistry::load(stations_file_));
        queue_ = std::make_unique<OrderQueue<std::shared_ptr<GoalHandle>>>(static_cast<size_t>(queue_capacity_));

        // Nav2's BT action/service nodes talk to their servers through this node; each BT
        // node spins its own callback group on it, so it is not added to an executor.
        // The explicit __node remap overrides the launch file's process-wide
        // "__node:=task_manager", which would otherwise give this node the same name.
        client_node_ = std::make_shared<rclcpp::Node>(
            "task_manager_bt_client",
            rclcpp::NodeOptions()
                .arguments({"--ros-args", "-r", "__node:=task_manager_bt_client", "--"})
                .parameter_overrides({{"use_sim_time", get_parameter("use_sim_time").as_bool()}}));
        engine_ = std::make_unique<MissionEngine>(plugins_, client_node_);
        engine_->register_trees(tree_file_);

        reporter_ = std::make_shared<Reporter>(this);
        blackboard_ = BT::Blackboard::create();
        blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
        blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(loop_ms_));
        blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(server_timeout_ms_));
        blackboard_->set<std::chrono::milliseconds>("wait_for_service_timeout",
                                                    std::chrono::milliseconds(wait_for_service_ms_));
        blackboard_->set<std::shared_ptr<StationRegistry>>("stations", stations_);
        blackboard_->set<std::shared_ptr<OperatorPanel>>("operator", operator_);
        blackboard_->set<std::shared_ptr<BatteryMonitor>>("battery", battery_);
        blackboard_->set<std::shared_ptr<MissionReporter>>("reporter", reporter_);
        blackboard_->set<double>("low_battery", low_battery_);
        blackboard_->set<double>("resume_battery", resume_battery_);
        blackboard_->set<int>("nav_attempts", static_cast<int>(nav_attempts_));
        blackboard_->set<double>("operator_timeout_s", operator_timeout_s_);
        blackboard_->set<double>("operator_auto_confirm_s", operator_auto_confirm_s_);
        blackboard_->set<double>("charge_timeout_s", charge_timeout_s_);
        blackboard_->set<std::string>("dock_id", dock_id_);
        blackboard_->set<std::string>("exception_station", exception_station_);
        // Persist across orders: where the robot is and what it carries.
        blackboard_->set<bool>("docked", start_docked_);
        blackboard_->set<bool>("loaded", false);
        blackboard_->set<std::string>("order_id", "");
        blackboard_->set<std::string>("pickup", "");
        blackboard_->set<std::string>("dropoff", "");

        cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        events_pub_ = create_publisher<OrderEvent>("/wdr/order_events", rclcpp::QoS(100).reliable());
        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            "/battery_state", rclcpp::SystemDefaultsQoS(), [this](const sensor_msgs::msg::BatteryState& m) {
                battery_->update(m.percentage,
                                 m.power_supply_status == sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING);
            });
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(), [this](const nav_msgs::msg::Odometry& m) { on_odom(m); });
        confirm_srv_ = create_service<OperatorConfirm>(
            "/wdr/operator_confirm",
            [this](const std::shared_ptr<OperatorConfirm::Request> req,
                   std::shared_ptr<OperatorConfirm::Response> res) { on_confirm(*req, *res); },
            rclcpp::ServicesQoS(), cb_group_);
        action_server_ = rclcpp_action::create_server<DeliverOrder>(
            this, "/deliver_order",
            [this](const rclcpp_action::GoalUUID&, std::shared_ptr<const DeliverOrder::Goal> goal) {
                return on_goal(*goal);
            },
            [this](const std::shared_ptr<GoalHandle>&) { return rclcpp_action::CancelResponse::ACCEPT; },
            [this](const std::shared_ptr<GoalHandle>& gh) { on_accepted(gh); }, rcl_action_server_get_default_options(),
            cb_group_);

        for (const auto& name : nav2_managers_) {
            nav2_checks_.push_back({name, create_client<std_srvs::srv::Trigger>("/" + name + "/is_active"), {}, false});
        }
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        idle_since_ = now();
        worker_ = std::thread([this] { mission_loop(); });
        RCLCPP_INFO(get_logger(), "task manager ready: %zu stations, queue capacity %ld", stations_->names().size(),
                    queue_capacity_);
    }

    ~TaskManager() override {
        stop_ = true;
        queue_cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

private:
    class Reporter : public MissionReporter {
    public:
        explicit Reporter(TaskManager* tm) : tm_(tm) {}
        void report_state(const std::string& s) override { tm_->on_state(s); }

    private:
        TaskManager* tm_;
    };

    struct Current {
        Order order;
        std::shared_ptr<GoalHandle> handle;
        std::string state;
        std::string phase;
        uint32_t retries = 0;
        double distance_m = 0.0;
    };

    // ------------------------------------------------------------ action server callbacks

    rclcpp_action::GoalResponse on_goal(const DeliverOrder::Goal& goal) {
        const std::string why = stations_->validate_order(goal.pickup_station, goal.dropoff_station);
        std::lock_guard<std::mutex> lock(mutex_);
        std::string reason = why;
        if (reason.empty() && queue_->size() >= queue_->capacity()) reason = "order queue is full";
        if (reason.empty() && !goal.order_id.empty() &&
            (queue_->contains(goal.order_id) || (current_ && current_->order.id == goal.order_id))) {
            reason = "order id '" + goal.order_id + "' is already queued";
        }
        if (!reason.empty()) {
            RCLCPP_WARN(get_logger(), "rejected order %s -> %s: %s", goal.pickup_station.c_str(),
                        goal.dropoff_station.c_str(), reason.c_str());
            Order o{goal.order_id, goal.pickup_station, goal.dropoff_station, goal.priority, 0.0};
            publish_event(o, OrderEvent::REJECTED, "", 0, reason);
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
    }

    void on_accepted(const std::shared_ptr<GoalHandle>& gh) {
        const auto goal = gh->get_goal();
        Order o;
        o.pickup = goal->pickup_station;
        o.dropoff = goal->dropoff_station;
        o.priority = goal->priority;
        o.enqueue_time_s = now().seconds();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal->order_id.empty()) {
                char buf[32];
                std::snprintf(buf, sizeof(buf), "ORD-%04u", ++order_counter_);
                o.id = buf;
            } else {
                o.id = goal->order_id;
            }
            queue_->push(o, gh);
            RCLCPP_INFO(get_logger(), "queued %s: %s -> %s (priority %u, position %zu)", o.id.c_str(), o.pickup.c_str(),
                        o.dropoff.c_str(), o.priority, queue_->position(o.id));
            publish_event(o, OrderEvent::QUEUED, state::kQueued, 0,
                          "position " + std::to_string(queue_->position(o.id)));
        }
        queue_cv_.notify_all();
    }

    void on_confirm(const OperatorConfirm::Request& req, OperatorConfirm::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (req.step != OperatorConfirm::Request::LOAD && req.step != OperatorConfirm::Request::UNLOAD) {
            res.accepted = false;
            res.message = "step must be 'load' or 'unload'";
            return;
        }
        if (!current_) {
            res.accepted = false;
            res.message = "no order is being executed";
            return;
        }
        if (!req.order_id.empty() && req.order_id != current_->order.id) {
            res.accepted = false;
            res.message = "order " + req.order_id + " is not the current order (" + current_->order.id + ")";
            return;
        }
        operator_->confirm(req.step);
        res.accepted = true;
        res.message = req.step + " confirmed for " + current_->order.id;
    }

    // ------------------------------------------------------------ callbacks from the tree

    void on_state(const std::string& s) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (current_) {
            current_->state = s;
            if (is_phase_state(s)) current_->phase = s;
            if (s == state::kNavRetry) ++current_->retries;
            RCLCPP_INFO(get_logger(), "[%s] %s", current_->order.id.c_str(), s.c_str());
            publish_event(current_->order, OrderEvent::STATE, s, 0, "");
            publish_feedback_locked();
        } else {
            RCLCPP_INFO(get_logger(), "[idle] %s", s.c_str());
            publish_event(Order{}, OrderEvent::STATE, s, 0, "");
        }
    }

    void on_odom(const nav_msgs::msg::Odometry& m) {
        const double x = m.pose.pose.position.x;
        const double y = m.pose.pose.position.y;
        std::lock_guard<std::mutex> lock(mutex_);
        if (have_last_odom_ && current_) current_->distance_m += std::hypot(x - last_x_, y - last_y_);
        last_x_ = x;
        last_y_ = y;
        have_last_odom_ = true;
    }

    // ------------------------------------------------------------ mission thread

    // Both trees are built once and reused for every order, as bt_navigator does. Building them
    // needs the Nav2 action servers to be up (Nav2's BT action nodes look for their server when
    // constructed), so this is retried from the mission loop; orders wait in the queue meanwhile.
    bool ensure_trees() {
        if (trees_ready_) return true;
        try {
            mission_tree_ = engine_->create("DeliverOrder", blackboard_);
            dock_tree_ = engine_->create("ReturnToDock", blackboard_);
        } catch (const std::exception& e) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "waiting for Nav2 before building the trees (%s)",
                                 e.what());
            return false;
        }
        if (groot_port_ > 0) engine_->addGrootMonitoring(&mission_tree_, static_cast<uint16_t>(groot_port_));
        trees_ready_ = true;
        RCLCPP_INFO(get_logger(), "behavior trees ready");
        return true;
    }

    // Orders wait in the queue until Nav2 is active, the robot is localized and the battery is
    // reporting. Starting earlier makes every Nav2 goal bounce off servers that are still
    // activating (each retry fails in milliseconds), the first undock fail (the dock cannot tell
    // the robot is on it) and the first battery check meaningless.
    bool robot_ready() {
        if (robot_ready_) return true;
        std::string missing;
        for (auto& check : nav2_checks_) {
            if (check.active) continue;
            missing += " " + check.name;
            if (check.pending.valid()) {
                if (check.pending.wait_for(0s) != std::future_status::ready) continue;
                check.active = check.pending.get()->success;
                check.pending = {};
            } else if (check.client->service_is_ready()) {
                check.pending = check.client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>())
                                    .future.share();
            }
        }
        if (!battery_->has_data()) missing += " battery_state";
        if (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero))
            missing += " localization(map->base_link)";
        if (!missing.empty()) {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "waiting for:%s", missing.c_str());
            return false;
        }
        robot_ready_ = true;
        RCLCPP_INFO(get_logger(), "robot ready: localized, battery at %.0f%%", battery_->percentage() * 100.0);
        return true;
    }

    void mission_loop() {
        while (rclcpp::ok() && !stop_) {
            cancel_queued_orders();
            if (!robot_ready() || !ensure_trees()) {
                std::this_thread::sleep_for(1s);
                continue;
            }
            std::optional<OrderQueue<std::shared_ptr<GoalHandle>>::Entry> next;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                queue_cv_.wait_for(lock, 200ms, [this] { return stop_ || !queue_->empty(); });
                if (stop_) break;
                next = queue_->pop();
            }
            if (next) {
                execute(*next);
                idle_since_ = now();
            } else {
                maybe_return_to_dock();
            }
        }
    }

    // Goals canceled while still waiting in the queue never start.
    void cancel_queued_orders() {
        std::vector<OrderQueue<std::shared_ptr<GoalHandle>>::Entry> canceled;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            canceled = queue_->remove_if([](const auto& e) { return e.handle->is_canceling(); });
        }
        for (auto& e : canceled) {
            auto result = std::make_shared<DeliverOrder::Result>();
            result->success = false;
            result->error_code = DeliverOrder::Result::CANCELED;
            result->error_msg = "canceled while queued";
            result->order_id = e.order.id;
            result->queue_time_s = now().seconds() - e.order.enqueue_time_s;
            e.handle->canceled(result);
            publish_event(e.order, OrderEvent::CANCELED, state::kQueued, result->error_code, result->error_msg);
            RCLCPP_INFO(get_logger(), "%s canceled while queued", e.order.id.c_str());
        }
    }

    void execute(OrderQueue<std::shared_ptr<GoalHandle>>::Entry& entry) {
        auto gh = entry.handle;
        auto result = std::make_shared<DeliverOrder::Result>();
        result->order_id = entry.order.id;
        const double start_s = now().seconds();
        result->queue_time_s = start_s - entry.order.enqueue_time_s;

        if (gh->is_canceling()) {  // canceled between leaving the queue and starting
            result->success = false;
            result->error_code = DeliverOrder::Result::CANCELED;
            result->error_msg = "canceled while queued";
            gh->canceled(result);
            publish_event(entry.order, OrderEvent::CANCELED, state::kQueued, result->error_code, result->error_msg);
            return;
        }
        gh->execute();  // ACCEPTED -> EXECUTING
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_ = Current{entry.order, gh, "", "", 0, 0.0};
            operator_->clear();
        }
        blackboard_->set<std::string>("order_id", entry.order.id);
        blackboard_->set<std::string>("pickup", entry.order.pickup);
        blackboard_->set<std::string>("dropoff", entry.order.dropoff);
        publish_event(entry.order, OrderEvent::STARTED, "", 0, "");
        RCLCPP_INFO(get_logger(), "starting %s: %s -> %s", entry.order.id.c_str(), entry.order.pickup.c_str(),
                    entry.order.dropoff.c_str());

        nav2_behavior_tree::BtStatus status = nav2_behavior_tree::BtStatus::FAILED;
        std::string error;
        try {
            auto last_feedback = std::chrono::steady_clock::now();
            status = engine_->run(
                &mission_tree_,
                [&] {
                    if (std::chrono::steady_clock::now() - last_feedback > 1s) {
                        last_feedback = std::chrono::steady_clock::now();
                        {
                            std::lock_guard<std::mutex> lock(mutex_);
                            publish_feedback_locked();
                        }
                        cancel_queued_orders();  // a queued order canceled now must not wait for this one
                    }
                },
                [&] { return stop_ || !rclcpp::ok() || gh->is_canceling(); }, std::chrono::milliseconds(loop_ms_));
            engine_->haltAllActions(mission_tree_);  // cancels any Nav2/docking goal still running
        } catch (const std::exception& e) {
            error = e.what();
            RCLCPP_ERROR(get_logger(), "behavior tree error: %s", e.what());
        }

        Current done;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            done = *current_;
            current_.reset();
        }
        result->duration_s = now().seconds() - start_s;
        result->distance_m = done.distance_m;
        result->retries = done.retries;

        if (status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
            result->success = true;
            result->error_code = DeliverOrder::Result::NONE;
            gh->succeed(result);
            publish_event(done.order, OrderEvent::SUCCEEDED, done.state, 0,
                          "duration " + std::to_string(result->duration_s) + " s");
            RCLCPP_INFO(get_logger(), "%s delivered in %.1f s, %.1f m, %u retries", done.order.id.c_str(),
                        result->duration_s, result->distance_m, result->retries);
        } else if (status == nav2_behavior_tree::BtStatus::CANCELED) {
            result->success = false;
            result->error_code = DeliverOrder::Result::CANCELED;
            result->error_msg = "canceled during " + done.phase;
            if (gh->is_canceling()) {
                gh->canceled(result);
            } else {
                gh->abort(result);  // shutting down
            }
            publish_event(done.order, OrderEvent::CANCELED, done.state, result->error_code, result->error_msg);
            RCLCPP_WARN(get_logger(), "%s canceled", done.order.id.c_str());
        } else {
            result->success = false;
            result->error_code = error.empty() ? error_code_for_phase(done.phase) : DeliverOrder::Result::UNKNOWN;
            result->error_msg = error.empty() ? "failed during " + done.phase : error;
            gh->abort(result);
            publish_event(done.order, OrderEvent::FAILED, done.state, result->error_code, result->error_msg);
            RCLCPP_ERROR(get_logger(), "%s failed: %s", done.order.id.c_str(), result->error_msg.c_str());
        }
    }

    // When nothing is queued for a while, drive back to the charger. Halted as soon as an
    // order arrives, so the robot never makes a customer wait for an idle-time dock.
    void maybe_return_to_dock() {
        if (blackboard_->get<bool>("docked")) return;
        const double idle = (now() - idle_since_).seconds();
        if (idle < idle_dock_after_s_) return;
        if (last_dock_failure_.nanoseconds() != 0 && (now() - last_dock_failure_).seconds() < 30.0) return;
        try {
            const auto status = engine_->run(
                &dock_tree_, [this] { cancel_queued_orders(); },
                [this] {
                    std::lock_guard<std::mutex> lock(mutex_);
                    return stop_ || !rclcpp::ok() || !queue_->empty();
                },
                std::chrono::milliseconds(loop_ms_));
            engine_->haltAllActions(dock_tree_);
            if (status == nav2_behavior_tree::BtStatus::FAILED) {
                RCLCPP_WARN(get_logger(), "could not return to the dock; retrying in 30 s");
                last_dock_failure_ = now();
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "return-to-dock tree error: %s", e.what());
            last_dock_failure_ = now();
        }
    }

    // ------------------------------------------------------------ outputs

    void publish_feedback_locked() {
        if (!current_ || !current_->handle->is_executing()) return;
        auto fb = std::make_shared<DeliverOrder::Feedback>();
        fb->order_id = current_->order.id;
        fb->state = current_->state;
        fb->queue_position = 0;
        fb->battery_percentage = static_cast<float>(battery_->percentage());
        fb->retries = current_->retries;
        current_->handle->publish_feedback(fb);
    }

    void publish_event(const Order& o, const std::string& event, const std::string& s, uint16_t code,
                       const std::string& detail) {
        OrderEvent e;
        e.stamp = now();
        e.order_id = o.id;
        e.pickup_station = o.pickup;
        e.dropoff_station = o.dropoff;
        e.event = event;
        e.state = s;
        e.error_code = code;
        e.detail = detail;
        e.battery_percentage = static_cast<float>(battery_->percentage());
        events_pub_->publish(e);
    }

    // parameters
    std::string stations_file_, tree_file_, dock_id_, exception_station_;
    std::vector<std::string> plugins_;
    int64_t loop_ms_, server_timeout_ms_, wait_for_service_ms_, queue_capacity_, nav_attempts_, groot_port_;
    double low_battery_, resume_battery_, operator_timeout_s_, operator_auto_confirm_s_, charge_timeout_s_;
    double idle_dock_after_s_;
    bool start_docked_;

    std::shared_ptr<StationRegistry> stations_;
    std::shared_ptr<OperatorPanel> operator_ = std::make_shared<OperatorPanel>();
    std::shared_ptr<BatteryMonitor> battery_ = std::make_shared<BatteryMonitor>();
    std::shared_ptr<Reporter> reporter_;
    rclcpp::Node::SharedPtr client_node_;
    std::unique_ptr<MissionEngine> engine_;
    BT::Blackboard::Ptr blackboard_;
    BT::Tree mission_tree_;
    BT::Tree dock_tree_;
    bool trees_ready_ = false;  // mission thread only
    bool robot_ready_ = false;  // mission thread only
    struct Nav2Check {
        std::string name;
        rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client;
        std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr> pending;
        bool active;
    };
    std::vector<std::string> nav2_managers_;
    std::vector<Nav2Check> nav2_checks_;  // mission thread only
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::CallbackGroup::SharedPtr cb_group_;
    rclcpp::Publisher<OrderEvent>::SharedPtr events_pub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Service<OperatorConfirm>::SharedPtr confirm_srv_;
    rclcpp_action::Server<DeliverOrder>::SharedPtr action_server_;

    // guarded by mutex_
    std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::unique_ptr<OrderQueue<std::shared_ptr<GoalHandle>>> queue_;
    std::optional<Current> current_;
    unsigned order_counter_ = 0;
    double last_x_ = 0.0, last_y_ = 0.0;
    bool have_last_odom_ = false;

    // mission thread only
    rclcpp::Time idle_since_;
    rclcpp::Time last_dock_failure_{0, 0, RCL_ROS_TIME};

    std::atomic<bool> stop_{false};
    std::thread worker_;
};

}  // namespace wdr_mission

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<wdr_mission::TaskManager>();
    node->init();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
