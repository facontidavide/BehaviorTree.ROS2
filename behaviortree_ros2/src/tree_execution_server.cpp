// Copyright 2024 Marq Rasmussen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions: The above copyright
// notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
// COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <thread>

#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_ros2/bt_utils.hpp"

#include "behaviortree_cpp/loggers/groot2_publisher.h"

// generated file
#include "bt_executor_parameters.hpp"
namespace
{
static const auto kLogger = rclcpp::get_logger("bt_action_server");
}

namespace BT
{

struct TreeExecutionServer::Pimpl
{
  Pimpl(const rclcpp::NodeOptions& node_options);

  rclcpp::Node::SharedPtr node;
  rclcpp_action::Server<ExecuteTree>::SharedPtr action_server;
  std::thread action_thread;
  // thread safe bool to keep track of cancel requests
  std::atomic<bool> cancel_requested{ false };

  std::shared_ptr<bt_server::ParamListener> param_listener;
  bt_server::Params params;

  BT::BehaviorTreeFactory factory;
  std::shared_ptr<BT::Groot2Publisher> groot_publisher;

  std::string current_tree_name;
  std::shared_ptr<BT::Tree> tree;
  BT::Blackboard::Ptr global_blackboard;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const ExecuteTree::Goal> goal);

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleExecuteTree> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTree> goal_handle);

  void execute(const std::shared_ptr<GoalHandleExecuteTree> goal_handle);
};

TreeExecutionServer::Pimpl::Pimpl(const rclcpp::NodeOptions& node_options)
  : node(std::make_unique<rclcpp::Node>("bt_action_server", node_options))
{
  param_listener = std::make_shared<bt_server::ParamListener>(node);
  params = param_listener->get_params();
  global_blackboard = BT::Blackboard::create();
}

TreeExecutionServer::~TreeExecutionServer()
{}

TreeExecutionServer::TreeExecutionServer(const rclcpp::NodeOptions& options)
  : p_(new Pimpl(options))
{
  // create the action server
  const auto action_name = p_->params.action_name;
  RCLCPP_INFO(kLogger, "Starting Action Server: %s", action_name.c_str());
  p_->action_server = rclcpp_action::create_server<ExecuteTree>(
      p_->node, action_name,
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const ExecuteTree::Goal> goal) {
        return handle_goal(uuid, std::move(goal));
      },
      [this](const std::shared_ptr<GoalHandleExecuteTree> goal_handle) {
        return handle_cancel(std::move(goal_handle));
      },
      [this](const std::shared_ptr<GoalHandleExecuteTree> goal_handle) {
        handle_accepted(std::move(goal_handle));
      });

  // register the users Plugins and BehaviorTree.xml files into the factory
  RegisterBehaviorTrees(p_->params, p_->factory, p_->node);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
TreeExecutionServer::nodeBaseInterface()
{
  return p_->node->get_node_base_interface();
}

rclcpp_action::GoalResponse
TreeExecutionServer::handle_goal(const rclcpp_action::GoalUUID& /* uuid */,
                                 std::shared_ptr<const ExecuteTree::Goal> goal)
{
  RCLCPP_INFO(kLogger, "Received goal request to execute Behavior Tree: %s",
              goal->target_tree.c_str());
  p_->cancel_requested = false;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TreeExecutionServer::handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
{
  RCLCPP_INFO(kLogger, "Received request to cancel goal");
  if(!goal_handle->is_active())
  {
    RCLCPP_WARN(kLogger, "Rejecting request to cancel goal because the server is not "
                         "processing one.");
    return rclcpp_action::CancelResponse::REJECT;
  }
  p_->cancel_requested = true;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TreeExecutionServer::handle_accepted(
    const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
{
  // Join the previous execute thread before replacing it with a new one
  if(p_->action_thread.joinable())
  {
    p_->action_thread.join();
  }
  // To avoid blocking the executor start a new thread to process the goal
  p_->action_thread = std::thread{ [=]() { execute(goal_handle); } };
}

void TreeExecutionServer::execute(
    const std::shared_ptr<GoalHandleExecuteTree> goal_handle)
{
  const auto goal = goal_handle->get_goal();
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  auto action_result = std::make_shared<ExecuteTree::Result>();

  // Loop until something happens with ROS or the node completes
  try
  {
    // This blackboard will be owned by "MainTree". It parent is p_->global_blackboard
    auto root_blackboard = BT::Blackboard::create(p_->global_blackboard);

    p_->tree = std::make_shared<BT::Tree>();
    *(p_->tree) = p_->factory.createTree(goal->target_tree, root_blackboard);
    p_->current_tree_name = goal->target_tree;

    // call user defined function after the tree has been created
    onTreeCreated(*p_->tree);
    p_->groot_publisher.reset();
    p_->groot_publisher =
        std::make_shared<BT::Groot2Publisher>(*(p_->tree), p_->params.groot2_port);

    // Loop until the tree is done or a cancel is requested
    const auto period =
        std::chrono::milliseconds(static_cast<int>(1000.0 / p_->params.tick_frequency));
    auto loop_deadline = std::chrono::steady_clock::now() + period;

    // operations to be done if the tree execution is aborted, either by
    // cancel_requested_ or by onLoopAfterTick()
    auto stop_action = [this, &action_result](BT::NodeStatus status,
                                              const std::string& message) {
      action_result->node_status = ConvertNodeStatus(status);
      action_result->error_message = message;
      RCLCPP_WARN(kLogger, action_result->error_message.c_str());
      p_->tree->haltTree();
      onTreeExecutionCompleted(status, true);
    };

    while(rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
      if(p_->cancel_requested)
      {
        stop_action(status, "Action Server canceling and halting Behavior Tree");
        goal_handle->canceled(action_result);
        return;
      }

      // tick the tree once and publish the action feedback
      status = p_->tree->tickExactlyOnce();

      if(const auto res = onLoopAfterTick(status); res.has_value())
      {
        stop_action(res.value(), "Action Server aborted by onLoopAfterTick()");
        goal_handle->abort(action_result);
        return;
      }

      if(const auto res = onLoopFeedback(); res.has_value())
      {
        auto feedback = std::make_shared<ExecuteTree::Feedback>();
        feedback->msg = res.value();
        goal_handle->publish_feedback(feedback);
      }

      const auto now = std::chrono::steady_clock::now();
      if(now < loop_deadline)
      {
        p_->tree->sleep(loop_deadline - now);
      }
      loop_deadline += period;
    }
  }
  catch(const std::exception& ex)
  {
    action_result->error_message = std::string("Behavior Tree exception:") + ex.what();
    RCLCPP_ERROR(kLogger, action_result->error_message.c_str());
    goal_handle->abort(action_result);
    return;
  }

  // call user defined execution complete function
  onTreeExecutionCompleted(status, false);

  // set the node_status result to the action
  action_result->node_status = ConvertNodeStatus(status);

  // return success or aborted for the action result
  if(status == BT::NodeStatus::SUCCESS)
  {
    RCLCPP_INFO(kLogger, "BT finished with status: %s", BT::toStr(status).c_str());
    goal_handle->succeed(action_result);
  }
  else
  {
    action_result->error_message = std::string("Behavior Tree failed during execution "
                                               "with status: ") +
                                   BT::toStr(status);
    RCLCPP_ERROR(kLogger, action_result->error_message.c_str());
    goal_handle->abort(action_result);
  }
}

const std::string& TreeExecutionServer::currentTreeName() const
{
  return p_->current_tree_name;
}

BT::Tree* TreeExecutionServer::currentTree()
{
  return p_->tree ? p_->tree.get() : nullptr;
}

BT::Blackboard::Ptr TreeExecutionServer::globalBlackboard()
{
  return p_->global_blackboard;
}

BT::BehaviorTreeFactory& TreeExecutionServer::factory()
{
  return p_->factory;
}

}  // namespace BT
