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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__MARK_FRONTIER_VISITED_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__MARK_FRONTIER_VISITED_NODE_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace hermes_navigate
{

/**
 * @class MarkFrontierVisitedNode
 * @brief Sync BT node that records a successfully-reached frontier goal in the
 *        shared `visited_frontiers` list on the blackboard.
 *
 * Placed in the BT tree immediately after a successful navigation leg (inside
 * a Sequence so it only runs when NavigateToPose returns SUCCESS).  Once a
 * goal is in the visited list, SelectFrontierNode permanently excludes any
 * frontier whose goal pose is within `blacklist_radius_m` of a visited
 * position, preventing the robot from revisiting already-explored spots.
 *
 * Unlike `BlacklistFrontierNode` (which records navigation failures and can be
 * cleared via the blacklist-fallback mechanism), the visited list is never
 * pruned during an exploration run — a visited frontier stays visited.
 *
 * This node always returns SUCCESS.
 *
 * BT ports:
 *   Input:        "goal"              — geometry_msgs::msg::PoseStamped
 *   Input/Output: "visited_frontiers" — std::vector<geometry_msgs::msg::PoseStamped>
 */
class MarkFrontierVisitedNode : public BT::SyncActionNode
{
public:
  MarkFrontierVisitedNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Logger logger);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory as "MarkFrontierVisited".
  static void registerWithFactory(BT::BehaviorTreeFactory & factory, rclcpp::Logger logger);

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger_;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__MARK_FRONTIER_VISITED_NODE_HPP_
