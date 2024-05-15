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
    RCLCPP_INFO(logger(), "SetBool service succeeded.");
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    RCLCPP_INFO(logger(), "SetBool service failed: %s", response->message.c_str());
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SetBoolService::onFailure(BT::ServiceNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "Error: %d", error);
  return BT::NodeStatus::FAILURE;
}

//-----------------------------------------------------------

BT::NodeStatus SetRobotBoolService::tick()
{
  std::string robot;
  if(getInput("robot", robot) && !robot.empty())
  {
    setServiceName(robot + "/" + service_suffix_);
  }
  return SetBoolService::tick();
}
