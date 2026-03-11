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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_NODE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace hermes_navigate
{

/**
 * @class ReturnToStartNode
 * @brief BT action node that writes the robot's start pose to the shared
 *        blackboard as the current navigation goal.
 *
 * The node simply copies "start_pose" to the "goal" output port (mapped to
 * the "{nav_goal}" blackboard key).  The NavigateToPose BT node in the same
 * Sequence branch then calls the Nav2 navigate_to_pose action server, which
 * autonomously plans the path back to the start position.
 *
 * BT ports:
 *   Input:  "start_pose" — geometry_msgs::msg::PoseStamped
 *   Output: "goal"       — geometry_msgs::msg::PoseStamped
 */
class ReturnToStartNode : public BT::SyncActionNode
{
public:
  ReturnToStartNode(
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

#endif  // HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_NODE_HPP_

