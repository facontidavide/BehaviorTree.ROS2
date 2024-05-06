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

#include <memory>
#include <optional>

#include "btcpp_ros2_interfaces/action/execute_tree.hpp"

#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace BT
{

/**
 * @brief TreeExecutionServer class hosts a ROS Action Server that is able
 * to load Behavior plugins, BehaviorTree.xml files and execute them.
 *
 * It can be customized by overriding its virtual functions.
 */
class TreeExecutionServer
{
public:
  using ExecuteTree = btcpp_ros2_interfaces::action::ExecuteTree;
  using GoalHandleExecuteTree = rclcpp_action::ServerGoalHandle<ExecuteTree>;

  /**
   * @brief Constructor for TreeExecutionServer.
   * @details This initializes a ParameterListener to read configurable options from the user and
   * starts an Action Server that takes requests to execute BehaviorTrees.
   *
   * @param options rclcpp::NodeOptions to pass to node_ when initializing it.
   * after the tree is created, while its running and after it finishes.
   */
  explicit TreeExecutionServer(const rclcpp::NodeOptions& options);

  virtual ~TreeExecutionServer();

  /**
   * @brief Gets the NodeBaseInterface of node_.
   * @details This function exists to allow running TreeExecutionServer as a component in a composable node container.
   *
   * @return A shared_ptr to the NodeBaseInterface of node_.
   */
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr nodeBaseInterface();

  // name of the tree being executed
  const std::string& currentTreeName() const;

  // tree being executed, nullptr if it doesn't exist yet.
  BT::Tree* currentTree();

  // pointer to the global blackboard
  BT::Blackboard::Ptr globalBlackboard();

  // reference to the factory, to allow users to register custom nodes
  BT::BehaviorTreeFactory& factory();

protected:
  /**
   * @brief Callback invoked after the tree is created.
   * It can be used, for instance, to initialize a logger or the global blackboard.
   *
   * @param tree The tree that was created
  */
  virtual void onTreeCreated(BT::Tree& tree)
  {}

  /**
   * @brief onLoopAfterTick invoked at each loop, after tree.tickOnce().
   * If it returns a valid NodeStatus, the tree will stop and return that status.
   * Return std::nullopt to continue the execution.
   *
   * @param status The status of the tree after the last tick
  */
  virtual std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status)
  {
    return std::nullopt;
  }

  /**
   * @brief onTreeExecutionCompleted is a callback invoked after the tree execution is completed,
   * i.e. if it returned SUCCESS/FAILURE or if the action was cancelled by the Action Client.
   *
   * @param status The status of the tree after the last tick
   * @param was_cancelled True if the action was cancelled by the Action Client
  */
  virtual void onTreeExecutionCompleted(BT::NodeStatus status, bool was_cancelled)
  {}

  /**
   * @brief onLoopFeedback is a callback invoked at each loop, after tree.tickOnce().
   * If it returns a valid string, it will be sent as feedback to the Action Client.
   *
   * If you don't want to return any feedback, return std::nullopt.
  */
  virtual std::optional<std::string> onLoopFeedback()
  {
    return std::nullopt;
  }

private:
  struct Pimpl;
  std::unique_ptr<Pimpl> p_;

  /**
   * @brief handle the goal requested: accept or reject. This implementation always accepts.
   * @param uuid Goal ID
   * @param goal A shared pointer to the specific goal
   * @return GoalResponse response of the goal processed
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const ExecuteTree::Goal> goal);

  /**
   * @brief Accepts cancellation requests of action server.
   * @param goal A server goal handle to cancel
   * @return CancelResponse response of the goal cancelled
   */
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleExecuteTree> goal_handle);

  /**
   * @brief Handles accepted goals and starts a thread to process them
   * @param goal_handle Server goal handle to process feedback and set the response when finished
   */
  void handle_accepted(const std::shared_ptr<GoalHandleExecuteTree> goal_handle);

  /**
   * @brief Background processes to execute the BehaviorTree and handle stop requests
   * @param goal_handle Server goal handle to process feedback and set the response when finished
   */
  void execute(const std::shared_ptr<GoalHandleExecuteTree> goal_handle);
};

}  // namespace BT
