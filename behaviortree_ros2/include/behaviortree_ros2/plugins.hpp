// Copyright (c) 2023 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <filesystem>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{
constexpr const char* ROS_PLUGIN_SYMBOL = "BT_RegisterRosNodeFromPlugin";
}

/* Use this macro to automatically register one or more custom Nodes
*  into a factory that require access to the BT::RosNodeParams.
*  For instance:
*
*   BT_REGISTER_ROS_NODES(factory, params)
*   {
*     factory.registerNodeType<SpecialNode>("SpecialNode", params);
*   }
*
* IMPORTANT: this function MUST be declared in a cpp file, NOT a header file.
* You must add the definition [BT_PLUGIN_EXPORT] in CMakeLists.txt using:
*
*   target_compile_definitions(my_plugin_target PRIVATE  BT_PLUGIN_EXPORT )
*/

#define BT_REGISTER_ROS_NODES(factory, params)                                           \
  BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory,       \
                                                 const BT::RosNodeParams& params)

// Use this macro to generate a plugin for:
//
// - BT::RosActionNode
// - BT::RosServiceNode
// - BT::RosTopicPubNode
// - BT::RosTopicSubNode
//
// - First argument: type to register (name of the class)
// - Second argument: string with the registration name
//
// Usage example:
//   CreateRosNodePlugin(MyClassName, "MyClassName");

#define CreateRosNodePlugin(TYPE, REGISTRATION_NAME)                                     \
  BTCPP_EXPORT void BT_RegisterRosNodeFromPlugin(BT::BehaviorTreeFactory& factory,       \
                                                 const BT::RosNodeParams& params)        \
  {                                                                                      \
    factory.registerNodeType<TYPE>(REGISTRATION_NAME, params);                           \
  }

/**
 * @brief RegisterRosNode function used to load a plugin and register
 * the containing Node definition.
 *
 * @param factory   the factory where the node should be registered.
 * @param filepath  path to the plugin.
 * @param params    parameters to pass to the instances of the Node.
 */
inline void RegisterRosNode(BT::BehaviorTreeFactory& factory,
                            const std::filesystem::path& filepath,
                            const BT::RosNodeParams& params)
{
  BT::SharedLibrary loader(filepath.generic_string());
  typedef void (*Func)(BT::BehaviorTreeFactory&, const BT::RosNodeParams&);
  auto func = (Func)loader.getSymbol(BT::ROS_PLUGIN_SYMBOL);
  func(factory, params);
}
