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

class NamespacedSetBool : public BT::ActionNodeBase
{
public:
  NamespacedSetBool(const std::string& name, const BT::NodeConfig& conf,
                    const std::string& service_name, std::weak_ptr<rclcpp::Node> node);

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
