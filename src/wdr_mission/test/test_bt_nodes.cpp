// Runs small trees built from the custom BT nodes against a real BehaviorTree.CPP factory
// (no Nav2 servers needed).
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "wdr_mission/bt_nodes.hpp"
#include "wdr_mission/mission_context.hpp"
#include "wdr_mission/station_registry.hpp"

using namespace wdr_mission;

namespace {

class RecordingReporter : public MissionReporter {
public:
    void report_state(const std::string& s) override { states.push_back(s); }
    std::vector<std::string> states;
};

class BtNodesTest : public ::testing::Test {
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override {
        factory.registerNodeType<GetStationPose>("GetStationPose");
        factory.registerNodeType<WaitForOperator>("WaitForOperator");
        factory.registerNodeType<WaitForCharge>("WaitForCharge");
        factory.registerNodeType<ReportState>("ReportState");
        factory.registerNodeType<BatteryBelow>("BatteryBelow");
        node = std::make_shared<rclcpp::Node>("bt_nodes_test");
        bb = BT::Blackboard::create();
        bb->set<rclcpp::Node::SharedPtr>("node", node);
        bb->set<std::shared_ptr<StationRegistry>>(
            "stations", std::make_shared<StationRegistry>(StationRegistry::parse(
                            "stations:\n  shelf_a1: {type: pickup, x: 6.0, y: 11.2, yaw: 1.5708}\n")));
        bb->set<std::shared_ptr<OperatorPanel>>("operator", panel);
        bb->set<std::shared_ptr<BatteryMonitor>>("battery", battery);
        bb->set<std::shared_ptr<MissionReporter>>("reporter", reporter);
    }

    BT::NodeStatus run(const std::string& body, std::chrono::milliseconds limit = std::chrono::seconds(5)) {
        const std::string xml =
            R"(<root BTCPP_format="4"><BehaviorTree ID="T">)" + body + R"(</BehaviorTree></root>)";
        auto tree = factory.createTreeFromText(xml, bb);
        const auto deadline = std::chrono::steady_clock::now() + limit;
        BT::NodeStatus status = tree.tickOnce();
        while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            status = tree.tickOnce();
        }
        tree.haltTree();
        return status;
    }

    BT::BehaviorTreeFactory factory;
    rclcpp::Node::SharedPtr node;
    BT::Blackboard::Ptr bb;
    std::shared_ptr<OperatorPanel> panel = std::make_shared<OperatorPanel>();
    std::shared_ptr<BatteryMonitor> battery = std::make_shared<BatteryMonitor>();
    std::shared_ptr<RecordingReporter> reporter = std::make_shared<RecordingReporter>();
};

}  // namespace

TEST_F(BtNodesTest, GetStationPoseOutputsTheStation) {
    EXPECT_EQ(run(R"(<GetStationPose station="shelf_a1" pose="{p}"/>)"), BT::NodeStatus::SUCCESS);
    const auto pose = bb->get<geometry_msgs::msg::PoseStamped>("p");
    EXPECT_EQ(pose.header.frame_id, "map");
    EXPECT_DOUBLE_EQ(pose.pose.position.x, 6.0);
    EXPECT_NEAR(pose.pose.orientation.z, std::sin(1.5708 / 2.0), 1e-9);
}

TEST_F(BtNodesTest, GetStationPoseFailsForUnknownStation) {
    EXPECT_EQ(run(R"(<GetStationPose station="nowhere" pose="{p}"/>)"), BT::NodeStatus::FAILURE);
}

TEST_F(BtNodesTest, WaitForOperatorSucceedsOnConfirmation) {
    panel->confirm("load");
    EXPECT_EQ(run(R"(<WaitForOperator step="load" timeout_s="5"/>)"), BT::NodeStatus::SUCCESS);
    EXPECT_FALSE(panel->take("load"));  // the confirmation was consumed
}

TEST_F(BtNodesTest, WaitForOperatorIgnoresTheOtherStep) {
    panel->confirm("unload");
    EXPECT_EQ(run(R"(<WaitForOperator step="load" timeout_s="0.3"/>)"), BT::NodeStatus::FAILURE);
}

TEST_F(BtNodesTest, SimulatedOperatorConfirmsAfterDelay) {
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_EQ(run(R"(<WaitForOperator step="load" timeout_s="5" auto_confirm_s="0.2"/>)"), BT::NodeStatus::SUCCESS);
    EXPECT_GE(std::chrono::steady_clock::now() - t0, std::chrono::milliseconds(190));
}

TEST_F(BtNodesTest, WaitForChargeWaitsForTarget) {
    battery->update(0.5, true);
    EXPECT_EQ(run(R"(<WaitForCharge target="0.9" timeout_s="0.3"/>)"), BT::NodeStatus::FAILURE);
    battery->update(0.95, true);
    EXPECT_EQ(run(R"(<WaitForCharge target="0.9" timeout_s="0.3"/>)"), BT::NodeStatus::SUCCESS);
}

TEST_F(BtNodesTest, BatteryBelowTreatsMissingDataAsLow) {
    EXPECT_EQ(run(R"(<BatteryBelow threshold="0.3"/>)"), BT::NodeStatus::SUCCESS);  // no reading yet
    battery->update(0.5, false);
    EXPECT_EQ(run(R"(<BatteryBelow threshold="0.3"/>)"), BT::NodeStatus::FAILURE);
    battery->update(0.2, false);
    EXPECT_EQ(run(R"(<BatteryBelow threshold="0.3"/>)"), BT::NodeStatus::SUCCESS);
}

TEST_F(BtNodesTest, ReportStateCallsTheReporter) {
    EXPECT_EQ(run(R"(<Sequence><ReportState state="TO_PICKUP"/><ReportState state="DONE"/></Sequence>)"),
              BT::NodeStatus::SUCCESS);
    EXPECT_EQ(reporter->states, (std::vector<std::string>{"TO_PICKUP", "DONE"}));
}

TEST_F(BtNodesTest, ScriptedFlagsWorkWithCppBools) {
    // The mission tree keeps `docked` and `loaded` as C++ bools on the blackboard and flips
    // them from Script nodes; make sure the scripting language agrees with the C++ type.
    bb->set<bool>("docked", true);
    EXPECT_EQ(run(R"(<Sequence><ScriptCondition code="docked"/><Script code="docked:=false"/></Sequence>)"),
              BT::NodeStatus::SUCCESS);
    EXPECT_FALSE(bb->get<bool>("docked"));
    EXPECT_EQ(run(R"(<ScriptCondition code="!docked"/>)"), BT::NodeStatus::SUCCESS);
}
