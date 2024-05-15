#pragma once

#include <behaviortree_ros2/bt_service_node.hpp>
#include "std_srvs/srv/set_bool.hpp"

using SetBool = std_srvs::srv::SetBool;

class SetBoolService : public BT::RosServiceNode<SetBool>
{
public:
  explicit SetBoolService(const std::string& name, const BT::NodeConfig& conf,
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

private:
  std::string service_suffix_;
};

//----------------------------------------------

class SetRobotBoolService : public SetBoolService
{
public:
  explicit SetRobotBoolService(const std::string& name, const BT::NodeConfig& conf,
                               const rclcpp::Node::SharedPtr& node,
                               const std::string& port_name)
    : SetBoolService(name, conf, BT::RosNodeParams(node)), service_suffix_(port_name)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("robot"), BT::InputPort<bool>("value") };
  }

  BT::NodeStatus tick() override;

private:
  std::string service_suffix_;
};
