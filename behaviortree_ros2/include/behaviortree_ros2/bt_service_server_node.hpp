/*
 * Enway GmbH - All Rights reserved.
 * Proprietary & confidential.
 *
 * -----------MENTION OF PREDECESSOR LICENSE--------------
 */
// Copyright (c) 2019 Intel Corporation
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

#include <memory>
#include <string>
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"

#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{

/**
 * @brief Abstract class use to wrap rclcpp::Service<>
 *
 * For instance, given the service described in this tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
 *
 * the corresponding wrapper would be:
 *
 * class AddTwoIntsService: public RosServiceServerNode<example_interfaces::srv::AddTwoInts>
 *
 * The derived class must reimplement the virtual methods as described below.
 *
 * The name of the service will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "service_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template <class ServiceT>
class RosServiceServerNode : public BT::ConditionNode
{
public:
  // Type definitions
  using Service = typename rclcpp::Service<ServiceT>;
  using ServicePtr = std::shared_ptr<Service>;
  using Request = typename ServiceT::Request;
  using Response = typename ServiceT::Response;

  /** To register this class into the factory, use:
   *
   *    factory.registerNodeType<>(node_name, params);
   */
  explicit RosServiceServerNode(const std::string& instance_name, const BT::NodeConfig& conf,
                                const BT::RosNodeParams& params);

  virtual ~RosServiceServerNode() = default;

  /**
   * @brief Any subclass of RosServiceServerNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return PortsList containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = { InputPort<std::string>("service_name", "", "Service name") };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  NodeStatus tick() override;

  virtual void serviceCallback(const typename Request::SharedPtr request,
                               typename Response::SharedPtr response) = 0;

protected:
  // method to set the service name programmatically
  void setServiceName(const std::string& service_name);

  std::shared_ptr<rclcpp::Node> node_;
  std::string service_name_;
  bool service_name_should_be_checked_ = false;

private:
  ServicePtr service_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;

  bool createService(const std::string& service_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
inline RosServiceServerNode<T>::RosServiceServerNode(const std::string& instance_name,
                                                     const NodeConfig& conf,
                                                     const RosNodeParams& params)
  : BT::ConditionNode(instance_name, conf)
  , node_(params.nh)
  , callback_group_(node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false))
{
  callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // check port remapping
  auto portIt = config().input_ports.find("service_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_service_name = portIt->second;

    if(isBlackboardPointer(bb_service_name))
    {
      // unknown value at construction time. postpone to tick
      service_name_should_be_checked_ = true;
    }
    else if(!bb_service_name.empty())
    {
      // "hard-coded" name in the bb_service_name. Use it.
      createService(bb_service_name);
    }
  }
  // no port value or it is empty. Use the default port value
  if(!params.default_port_value.empty())
  {
    createService(params.default_port_value);
  }
}

template <class T>
inline bool RosServiceServerNode<T>::createService(const std::string& service_name)
{
  if(service_name.empty() || service_name == "__default__placeholder__")
  {
    throw RuntimeError("service_name is empty or invalid");
  }

  service_ = node_->create_service<T>(
      service_name,
      [this](const typename Request::SharedPtr request,
             typename Response::SharedPtr response) -> void {
        serviceCallback(request, response);
      },
      rmw_qos_profile_services_default,
      callback_group_);
  RCLCPP_INFO(node_->get_logger(), "Node [%s] created service client [%s]", name().c_str(),
              service_name.c_str());
  service_name_ = service_name;
  return true;
}

template <class T>
inline void RosServiceServerNode<T>::setServiceName(const std::string& service_name)
{
  service_name_ = service_name;
  createService(service_name);
}

template <class T>
inline NodeStatus RosServiceServerNode<T>::tick()
{
  if(!rclcpp::ok())
  {
    return NodeStatus::FAILURE;
  }

  // First, check if the service is valid and that the name of the
  // service_name in the port didn't change.
  // otherwise, create a new service
  if(!service_ || (status() == NodeStatus::IDLE && service_name_should_be_checked_))
  {
    std::string service_name;
    getInput("service_name", service_name);
    if(service_name_ != service_name)
    {
      createService(service_name);
    }
  }

  if(!service_)
  {
    throw BT::RuntimeError("RosServiceServerNode: no service name was specified neither as "
                           "default or in the ports");
  }

  callback_executor_.spin_some();
  return NodeStatus::SUCCESS;
}

}  // namespace BT
