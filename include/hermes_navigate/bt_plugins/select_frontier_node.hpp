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
#include "behaviortree_cpp/bt_factory.h"
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
 * The best frontier (highest composite score) is selected, subject to three
 * filters applied in order:
 *   1. Its score must be at least `min_frontier_score`.
 *   2. Its goal pose must not be within `blacklist_radius_m` of any position
 *      in the `visited_frontiers` blackboard list.  Frontiers the robot has
 *      already successfully navigated to (recorded by MarkFrontierVisitedNode)
 *      are permanently excluded so the robot never revisits them.
 *   3. Its goal pose must not be within `blacklist_radius_m` of any position
 *      in the `blacklisted_goals` blackboard list.  Frontiers that Nav2
 *      failed to reach (recorded by BlacklistFrontierNode) are excluded so
 *      that the robot never wastes time retrying unreachable goals.
 *
 * Hysteresis requires a new frontier's score to exceed the current goal's
 * score by `hysteresis_factor` (fractional) before switching.  When the
 * active goal appears in either the visited or blacklisted list, hysteresis is
 * reset so a fresh frontier can be selected on the next tick.  This is the
 * core fix for the "robot gets stuck after reaching a frontier" bug: once a
 * frontier is marked visited, the hysteresis bias toward it is cleared and the
 * robot moves on to the next best unvisited frontier.
 *
 * When no non-blacklisted frontier remains but there are valid frontiers that
 * are only excluded due to blacklisting (and not visited), the node clears
 * those blacklist entries and selects the best of them as a fallback so that
 * exploration can continue rather than terminating prematurely.  Visited
 * frontiers are never retried.
 *
 * BT ports:
 *   Input:        "scored_frontiers"  — std::vector<ScoredFrontier>
 *   Input:        "robot_pose"        — geometry_msgs::msg::PoseStamped
 *   Input/Output: "blacklisted_goals" — std::vector<geometry_msgs::msg::PoseStamped>
 *   Input:        "visited_frontiers" — std::vector<geometry_msgs::msg::PoseStamped>
 *   Output:       "nav_goal"          — geometry_msgs::msg::PoseStamped
 *   Output:       "exploration_done"  — bool
 */
class SelectFrontierNode : public BT::SyncActionNode
{
public:
  SelectFrontierNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory.
  static void registerWithFactory(
    BT::BehaviorTreeFactory & factory,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

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
  double blacklist_radius_m_{0.5};   ///< Exclusion radius around each blacklisted pose [m].
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__SELECT_FRONTIER_NODE_HPP_
