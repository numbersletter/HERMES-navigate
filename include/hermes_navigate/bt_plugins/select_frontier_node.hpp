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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__SELECT_FRONTIER_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__SELECT_FRONTIER_NODE_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "hermes_navigate/frontier_types.hpp"

namespace hermes_navigate
{

/**
 * @class SelectFrontierNode
 * @brief BT action node that selects the best scored frontier as the next
 *        navigation goal, with hysteresis to prevent rapid goal-switching.
 *
 * The best frontier (highest composite score) is selected. Hysteresis
 * requires a new frontier's score to exceed the current goal's score by
 * `hysteresis_factor` (fractional) before switching.  When no viable
 * frontier remains, the node sets `exploration_done = true` on the
 * blackboard.
 *
 * BT ports:
 *   Input:  "scored_frontiers"  — std::vector<ScoredFrontier>
 *   Input:  "robot_pose"        — geometry_msgs::msg::PoseStamped
 *   Output: "goal"              — geometry_msgs::msg::PoseStamped
 *   Output: "exploration_done"  — bool
 */
class SelectFrontierNode : public BT::SyncActionNode
{
public:
  SelectFrontierNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  // Hysteresis state
  bool has_active_goal_{false};
  geometry_msgs::msg::PoseStamped active_goal_;
  double active_goal_score_{0.0};

  // Parameters
  double hysteresis_factor_{0.15};   ///< Fraction above current score to switch.
  double min_frontier_score_{0.0};   ///< Minimum score to consider viable.
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__SELECT_FRONTIER_NODE_HPP_
