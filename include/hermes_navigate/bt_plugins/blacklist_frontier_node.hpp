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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__BLACKLIST_FRONTIER_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__BLACKLIST_FRONTIER_NODE_HPP_

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
 * @class BlacklistFrontierNode
 * @brief Sync BT node that marks a frontier goal as unreachable by appending
 *        it to the shared blacklisted_goals list on the blackboard.
 *
 * Placed in the BT tree after all navigation retries for a frontier have been
 * exhausted.  Once a goal is blacklisted, SelectFrontierNode will skip any
 * frontier whose goal pose is within `blacklist_radius_m` of a blacklisted
 * position, ensuring the robot never attempts to navigate to an unreachable
 * location again during the current exploration run.
 *
 * This node always returns SUCCESS so that the exploration pipeline can
 * immediately re-tick the frontier selection step and pick a new goal.
 *
 * BT ports:
 *   Input:        "goal"              — geometry_msgs::msg::PoseStamped (the failed goal)
 *   Input/Output: "blacklisted_goals" — std::vector<geometry_msgs::msg::PoseStamped>
 */
class BlacklistFrontierNode : public BT::SyncActionNode
{
public:
  BlacklistFrontierNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp::Logger logger);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory as "BlacklistFrontier".
  static void registerWithFactory(BT::BehaviorTreeFactory & factory, rclcpp::Logger logger);

  BT::NodeStatus tick() override;

private:
  rclcpp::Logger logger_;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__BLACKLIST_FRONTIER_NODE_HPP_
