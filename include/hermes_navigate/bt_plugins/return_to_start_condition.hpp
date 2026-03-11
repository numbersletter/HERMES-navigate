// Copyright 2024 HERMES Team
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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_CONDITION_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace hermes_navigate
{

/**
 * @class ReturnToStartCondition
 * @brief BT condition node that checks whether the robot should return to its
 *        start pose.
 *
 * Reads the "return_to_start" boolean from the blackboard (set by
 * HermesNavigateNode on on_deactivate()) and returns SUCCESS when it is true,
 * FAILURE otherwise.
 *
 * BT ports:
 *   Input: "return_to_start" — bool
 */
class ReturnToStartCondition : public BT::ConditionNode
{
public:
  ReturnToStartCondition(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Logger logger);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory.
  static void registerWithFactory(BT::BehaviorTreeFactory & factory, rclcpp::Logger logger);

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger_;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_CONDITION_HPP_
