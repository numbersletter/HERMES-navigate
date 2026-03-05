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

#ifndef HERMES_NAVIGATE__SCORE_FRONTIERS_HPP_
#define HERMES_NAVIGATE__SCORE_FRONTIERS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"

// ScoredFrontier and Frontier are now defined in the common header
#include "hermes_navigate/frontier_types.hpp"

namespace hermes_navigate
{

/**
 * @class ScoreFrontiers
 * @brief BT action node that scores each candidate frontier using:
 *
 *   score = alpha * information_gain - beta * traversal_cost
 *
 *   - information_gain: number of unknown cells within `info_radius_m` of the
 *     frontier centroid.
 *   - traversal_cost: A* path length on the costmap from the robot's current
 *     position to the frontier goal.
 *
 * BT ports:
 *   Input:  "frontiers"         — std::vector<Frontier>
 *   Input:  "robot_pose"        — geometry_msgs::msg::PoseStamped
 *   Output: "scored_frontiers"  — std::vector<ScoredFrontier>
 */
class ScoreFrontiers : public BT::SyncActionNode
{
public:
  ScoreFrontiers(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  // Parameters
  double alpha_;          ///< Weight for information gain.
  double beta_;           ///< Weight for traversal cost.
  double info_radius_m_;  ///< Radius (m) for counting unknown cells.

  /// @brief Count unknown cells within `radius_cells` of (cx, cy).
  int countUnknownCells(
    const nav2_msgs::msg::Costmap & costmap,
    int cx, int cy, int radius_cells);

  /**
   * @brief Lightweight A* on the costmap.
   * @return Path length in metres, or -1 if no path found.
   */
  double aStarCost(
    const nav2_msgs::msg::Costmap & costmap,
    int start_x, int start_y,
    int goal_x,  int goal_y);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__SCORE_FRONTIERS_HPP_
