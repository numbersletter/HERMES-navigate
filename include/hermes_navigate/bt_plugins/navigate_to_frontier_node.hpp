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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__NAVIGATE_TO_FRONTIER_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__NAVIGATE_TO_FRONTIER_NODE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace hermes_navigate
{

/**
 * @class NavigateToFrontierNode
 * @brief Sync BT node that copies the selected frontier goal pose onto the
 *        shared blackboard for the downstream NavigateToPose BT node to consume.
 *
 * This node is responsible only for *goal-setting*: it reads the frontier goal
 * chosen by SelectFrontierNode from its "goal" input port and writes it to the
 * "nav_goal" output port (which maps to the {nav_goal} blackboard entry).
 *
 * The actual navigation is delegated to Nav2's built-in NavigateToPose BT node
 * which reads {nav_goal} from the blackboard and calls the navigate_to_pose
 * action server.
 *
 * BT ports:
 *   Input:  "goal"     — geometry_msgs::msg::PoseStamped  (from SelectFrontier)
 *   Output: "nav_goal" — geometry_msgs::msg::PoseStamped  (consumed by NavigateToPose)
 */
class NavigateToFrontierNode : public BT::SyncActionNode
{
public:
  NavigateToFrontierNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory as "NavigateToFrontier".
  static void registerWithFactory(
    BT::BehaviorTreeFactory & factory,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  BT::NodeStatus tick() override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__NAVIGATE_TO_FRONTIER_NODE_HPP_
