#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "set_bool_node.hpp"

using namespace BT;

static const char* xml_text = R"(
 <root BTCPP_format="4">
     <BehaviorTree>
        <Sequence>
            <RobotSetBool robot="robotA" command="true"/>
            <RobotSetBool robot="robotB" command="true"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("bool_client");

  BehaviorTreeFactory factory;
  factory.registerNodeType<NamespacedSetBool>("RobotSetBool", "set_bool", nh);

  auto tree = factory.createTreeFromText(xml_text);

  tree.tickWhileRunning();

  return 0;
}
