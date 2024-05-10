#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include "std_srvs/srv/set_bool.hpp"

using SetBool = std_srvs::srv::SetBool;

class SetBoolService : public BT::RosServiceNode<SetBool>
{
public:
  SetBoolService(const std::string& name, const BT::NodeConfig& conf,
                 const BT::RosNodeParams& params)
    : RosServiceNode<SetBool>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<bool>("value") });
  }

  bool setRequest(Request::SharedPtr& request) override;

  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;

  virtual BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;
};

//------------------------------------------------------

// This is a workaround that can be used when we want to use SetBoolService,
// but the service name has a namespace.
// Therefore, instead of:
//
// <SetBool service_name="robotA/set_bool" value="true" />
//
// We can rewrite it as:
//
// <RobotSetBool robot="{robot_id}" command="true" />
//
// Registration in C++:
//
// factory.registerNodeType<RobotSetBool>("SetRobotBool", "set_bool", node);

class NamespacedSetBool : public BT::ActionNodeBase
{
public:
  NamespacedSetBool(const std::string& name, const BT::NodeConfig& conf,
                    const BT::RosNodeParams& params);

  NamespacedSetBool(const std::string& name, const BT::NodeConfig& conf,
                    const std::string& service_name, rclcpp::Node::SharedPtr node);

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("robot"), BT::InputPort<bool>("command") };
  }

  BT::NodeStatus tick() override;

  void halt() override
  {
    set_bool_service_->halt();
  }

private:
  BT::Blackboard::Ptr local_bb_;
  std::string service_name_;
  std::unique_ptr<SetBoolService> set_bool_service_;
};
