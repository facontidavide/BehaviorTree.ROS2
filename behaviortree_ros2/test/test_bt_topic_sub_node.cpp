#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/blackboard.h>
#include <behaviortree_cpp/tree_node.h>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/executors.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <std_msgs/msg/empty.hpp>

#include <thread>

namespace
{
constexpr auto kTopicName = "/my_topic";
constexpr auto kHistoryDepth = 1;
constexpr auto kPortIdTopicName = "topic_name";

using Empty = std_msgs::msg::Empty;
}  // namespace

namespace BT
{

class SubNode : public RosTopicSubNode<Empty>
{
public:
  SubNode(const std::string& instance_name, const BT::NodeConfig& conf,
          const RosNodeParams& params)
    : RosTopicSubNode<Empty>(instance_name, conf, params)
  {}

private:
  NodeStatus onTick(const std::shared_ptr<Empty>& last_msg) override
  {
    // GIVEN if any message is received on the topic
    if(last_msg != nullptr)
    {
      return NodeStatus::SUCCESS;
    }
    return NodeStatus::FAILURE;
  }
};

class TestBtTopicSubNode : public ::testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    node_ = std::make_shared<rclcpp::Node>("ros_node");

    BT::assignDefaultRemapping<SubNode>(config_);
    config_.blackboard = Blackboard::create();

    executor_thread_ = std::thread([this]() { rclcpp::spin(node_); });
  }

  void TearDown() override
  {
    // std::this_thread::sleep_for(std::chrono::seconds(15));

    rclcpp::shutdown();
    executor_thread_.join();
  }

  void createPublisher(const rclcpp::QoS& qos)
  {
    publisher_ = node_->create_publisher<Empty>(kTopicName, qos);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Publisher<Empty>> publisher_;

  NodeConfig config_;

private:
  std::thread executor_thread_;
};

TEST_F(TestBtTopicSubNode, TopicAsParam)
{
  // Problems: exception thrown upon creation
  // The BT node's executor needs to be spun once to register its subscriber with the publisher, and then spun a second time to actually receive the message.
  // This causes sort of weird behavior -- if the subscriber is created in the constructor, then it alwaus functions like a latched topic for its first tick, and then follows the setting for remaining ticks.
  // Probably always want to create the subscriber in the first tick if possible, since this lets us check QoS too.

  // GIVEN the blackboard does not contain the topic name

  // GIVEN the input params contain the topic name
  RosNodeParams params;
  params.nh = node_;
  params.default_port_value = kTopicName;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with default QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, TopicFromStaticStringInPort)
{
  // GIVEN the input port directly contains the topic name
  config_.input_ports[kPortIdTopicName] = kTopicName;

  // GIVEN the input params do not contain the topic name
  RosNodeParams params;
  params.nh = node_;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with default QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, TopicFromBlackboard)
{
  // GIVEN the input port is remapped to point to a value on the blackboard
  config_.blackboard->set(kPortIdTopicName, kTopicName);

  // GIVEN the input params do not contain the topic name
  RosNodeParams params;
  params.nh = node_;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with default QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth));

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}

TEST_F(TestBtTopicSubNode, TopicAsParamQoSBestEffort)
{
  // GIVEN the blackboard does not contain the topic name

  // GIVEN the input params contain the topic name
  RosNodeParams params;
  params.nh = node_;
  params.default_port_value = kTopicName;

  // GIVEN we pass in the params and the blackboard when creating the BT node
  SubNode bt_node("test_node", config_, params);

  // GIVEN we create publisher with BestEffort reliability QoS settings after creating the BT node
  createPublisher(rclcpp::QoS(kHistoryDepth).best_effort());

  // GIVEN the publisher has published a message
  publisher_->publish(std_msgs::build<Empty>());

  // WHEN the BT node is ticked
  // THEN it succeeds
  EXPECT_THAT(bt_node.executeTick(), testing::Eq(NodeStatus::SUCCESS));
}
}  // namespace BT
