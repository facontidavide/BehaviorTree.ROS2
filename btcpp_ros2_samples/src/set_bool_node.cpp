#include "set_bool_node.hpp"

bool SetBoolService::setRequest(Request::SharedPtr& request)
{
  getInput("value", request->data);
  std::cout << "setRequest " << std::endl;
  return true;
}

BT::NodeStatus SetBoolService::onResponseReceived(const Response::SharedPtr& response)
{
  std::cout << "onResponseReceived " << std::endl;
  if(response->success)
  {
    RCLCPP_INFO(logger(), "SetString service succeeded.");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "SetString service failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SetBoolService::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

//------------------------------------------------------
//------------------------------------------------------
//------------------------------------------------------

NamespacedSetBool::NamespacedSetBool(const std::string& name, const BT::NodeConfig& conf,
                                     const BT::RosNodeParams& params)
  : BT::ActionNodeBase(name, conf)
  , local_bb_(BT::Blackboard::create(conf.blackboard))
  , service_name_(params.default_port_value)
{
  BT::NodeConfig impl_config;
  impl_config.blackboard = local_bb_;
  impl_config.input_ports["service_name"] = "{=}";
  impl_config.input_ports["value"] = "{=}";

  BT::RosNodeParams impl_params = params;
  impl_params.default_port_value = {};  // postpone this
  set_bool_service_ = std::make_unique<SetBoolService>(name, impl_config, impl_params);
}

NamespacedSetBool::NamespacedSetBool(const std::string& name, const BT::NodeConfig& conf,
                                     const std::string& service_name,
                                     rclcpp::Node::SharedPtr node)
  : BT::ActionNodeBase(name, conf)
  , local_bb_(BT::Blackboard::create(conf.blackboard))
  , service_name_(service_name)
{
  BT::NodeConfig impl_config;
  impl_config.blackboard = local_bb_;
  impl_config.input_ports["service_name"] = "{=}";
  impl_config.input_ports["value"] = "{=}";

  BT::RosNodeParams impl_params;
  impl_params.nh = node;
  set_bool_service_ = std::make_unique<SetBoolService>(name, impl_config, impl_params);
}

BT::NodeStatus NamespacedSetBool::tick()
{
  std::string robot;
  bool command;
  if(!getInput("robot", robot) || !getInput("command", command))
  {
    throw BT::RuntimeError("NamespacedSetBool: Missing inputs");
  }

  local_bb_->set("service_name", robot + "/" + service_name_);
  local_bb_->set("value", command);
  std::cout << "ticking " << std::endl;
  return set_bool_service_->tick();
}
