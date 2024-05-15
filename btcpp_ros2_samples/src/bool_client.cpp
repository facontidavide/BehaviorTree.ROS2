#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "set_bool_node.hpp"

using namespace BT;

static const char* xml_text = R"(
 <root BTCPP_format="4">
     <BehaviorTree>
        <Sequence>

            <SetBoolA value="true"/>
            <SetBool  service_name="robotB/set_bool" value="true"/>
            <SetRobotBool robot="robotA" value="true"/>
            <SetRobotBool robot="robotB" value="true"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bool_client");

  BehaviorTreeFactory factory;

  // version with default port
  factory.registerNodeType<SetBoolService>("SetBoolA", BT::RosNodeParams(nh, "robotA/"
                                                                             "set_bool"));

  // version without default port
  factory.registerNodeType<SetBoolService>("SetBool", BT::RosNodeParams(nh));

  // namespace version
  factory.registerNodeType<SetRobotBoolService>("SetRobotBool", nh, "set_bool");

  auto tree = factory.createTreeFromText(xml_text);

  tree.tickWhileRunning();

  return 0;
}
