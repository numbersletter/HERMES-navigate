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

#ifndef HERMES_NAVIGATE__COMPUTE_FRONTIERS_HPP_
#define HERMES_NAVIGATE__COMPUTE_FRONTIERS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"

// Frontier types are now defined in the common header
#include "hermes_navigate/frontier_types.hpp"

namespace hermes_navigate
{

/**
 * @class ComputeFrontiers
 * @brief BT action node that uses BFS on the Nav2 global costmap to detect
 *        frontier cells (free cells adjacent to unknown cells), clusters them,
 *        and outputs a list of candidate frontier poses on the BT blackboard.
 *
 * BT ports:
 *   Output: "frontiers"  — std::vector<Frontier>
 */
class ComputeFrontiers : public BT::SyncActionNode
{
public:
  ComputeFrontiers(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;

  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  // Parameters
  double min_frontier_size_;    ///< Minimum cells to count as a valid frontier.
  double clustering_distance_;  ///< Max distance (m) between cells in a cluster.

  /// @brief BFS over the costmap to identify frontier cells.
  std::vector<std::pair<int, int>> detectFrontierCells(
    const nav2_msgs::msg::Costmap & costmap);

  /// @brief Group raw frontier cells into clusters using flood-fill.
  std::vector<std::vector<std::pair<int, int>>> clusterFrontiers(
    const std::vector<std::pair<int, int>> & cells,
    const nav2_msgs::msg::Costmap & costmap);

  /// @brief Compute the representative Frontier struct for a cluster.
  Frontier clusterToFrontier(
    const std::vector<std::pair<int, int>> & cluster,
    const nav2_msgs::msg::Costmap & costmap);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__COMPUTE_FRONTIERS_HPP_
