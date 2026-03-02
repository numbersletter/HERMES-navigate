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

#ifndef HERMES_NAVIGATE__SELECT_NEXT_FRONTIER_HPP_
#define HERMES_NAVIGATE__SELECT_NEXT_FRONTIER_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "hermes_navigate/score_frontiers.hpp"

namespace hermes_navigate
{

/**
 * @class SelectNextFrontier
 * @brief BT action node that orders scored frontiers and selects the next
 *        NavigateToPose goal.
 *
 * Ordering uses a greedy nearest-neighbor heuristic seeded by the robot's
 * current position to minimise total travel.  Hysteresis prevents rapid
 * goal-switching: if the robot already has an active frontier goal, a new
 * frontier is only accepted when its score exceeds the current goal's score
 * by `hysteresis_factor`.
 *
 * BT ports:
 *   Input:  "scored_frontiers"  — std::vector<ScoredFrontier>
 *   Input:  "robot_pose"        — geometry_msgs::msg::PoseStamped
 *   Output: "goal"              — geometry_msgs::msg::PoseStamped
 *   Output: "exploration_done"  — bool  (true when no viable frontier remains)
 */
class SelectNextFrontier : public BT::SyncActionNode
{
public:
  SelectNextFrontier(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  // Hysteresis state
  bool has_active_goal_{false};
  geometry_msgs::msg::PoseStamped active_goal_;
  double active_goal_score_{0.0};

  // Parameters
  double hysteresis_factor_;      ///< Fraction above current score to switch.
  double min_frontier_score_;     ///< Minimum score to consider a frontier viable.

  /// @brief Euclidean distance between two poses (x-y only).
  static double dist(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__SELECT_NEXT_FRONTIER_HPP_
