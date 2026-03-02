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

#ifndef HERMES_NAVIGATE__SELECT_COVERAGE_WAYPOINT_HPP_
#define HERMES_NAVIGATE__SELECT_COVERAGE_WAYPOINT_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

/**
 * @class SelectCoverageWaypoint
 * @brief BT action node that picks the next camera-coverage waypoint.
 *
 * Wall-segment waypoints are pre-computed from the current `/map` occupancy
 * grid: for each occupied cell (wall) the node generates a viewing pose
 * perpendicular to the wall and `standoff_distance_m` away from it, facing
 * the wall.  Waypoints that have already been marked as "viewed" by the
 * CoverageTrackerNode (read from the "viewed_cells" blackboard entry) are
 * skipped.
 *
 * BT ports:
 *   Output: "goal"  — geometry_msgs::msg::PoseStamped
 *   Returns FAILURE when all waypoints have been covered (coverage complete).
 */
class SelectCoverageWaypoint : public BT::SyncActionNode
{
public:
  SelectCoverageWaypoint(const std::string & name, const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;

  // Parameters
  double standoff_distance_m_;  ///< How far from wall to place viewing pose.

  // Precomputed waypoints
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
  std::size_t waypoint_index_{0};
  bool waypoints_computed_{false};

  /// @brief Derive wall-facing waypoints from the current occupancy grid.
  void computeWaypoints(const nav_msgs::msg::OccupancyGrid & map);

  /// @brief Compute yaw angle from dx, dy.
  static double atan2Yaw(double dy, double dx);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__SELECT_COVERAGE_WAYPOINT_HPP_
