#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <memory>

void CallbackA(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = request->data;
  auto logger = rclcpp::get_logger("rclcpp");
  RCLCPP_INFO(logger, "Incoming request A: %s", request->data ? "true" : "false");
}

void CallbackB(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  response->success = request->data;
  auto logger = rclcpp::get_logger("rclcpp");
  RCLCPP_INFO(logger, "Incoming request B: %s", request->data ? "true" : "false");
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("add_two_ints_server");

  auto serviceA =
      node->create_service<std_srvs::srv::SetBool>("robotA/set_bool", &CallbackA);
  auto serviceB =
      node->create_service<std_srvs::srv::SetBool>("robotB/set_bool", &CallbackB);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
