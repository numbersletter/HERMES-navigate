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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__SELECT_WALL_VIEWPOINT_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__SELECT_WALL_VIEWPOINT_NODE_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace hermes_navigate
{

/**
 * @class SelectWallViewpointNode
 * @brief BT sync action node that generates and sequentially selects
 *        viewpoints facing every reachable wall/occupied boundary so that
 *        the front-facing camera can inspect pictures hanging on the walls.
 *
 * Algorithm (called once on the first tick after exploration ends):
 *   1. Scan the global costmap for wall-boundary cells: lethal cells (cost ≥
 *      LETHAL) that have at least one free neighbour (cost < INSCRIBED and
 *      not UNKNOWN).
 *   2. For each wall-boundary cell compute the outward normal (average
 *      direction toward free neighbours) and place a candidate viewpoint at
 *      wall_centre + normal × standoff_m.  The robot's heading at that
 *      viewpoint faces the wall (opposite of the normal).  Discard viewpoints
 *      that land inside an inflated or unknown cell.
 *   3. Cluster candidates spatially using a cluster_distance_m grid bucket.
 *      Discard clusters with fewer than min_segment_cells candidates.
 *      Use the centroid of each remaining cluster as the final viewpoint.
 *   4. Order viewpoints via a greedy nearest-neighbour tour starting from the
 *      robot's current position to minimise total travel.
 *
 * On each subsequent tick the node pops the next viewpoint from the ordered
 * list, writes it to the nav_goal blackboard key and returns SUCCESS.  When
 * the list is exhausted it sets inspection_done=true and returns FAILURE,
 * which propagates through KeepRunningUntilFailure and terminates the
 * inspection pipeline.
 *
 * Designed for use inside a plain BT Sequence (with memory) paired with a
 * NavigateToPose action:
 * @code
 *   <KeepRunningUntilFailure>
 *     <Sequence name="InspectionPipeline">
 *       <SelectWallViewpoint
 *           robot_pose="{robot_pose}"
 *           nav_goal="{nav_goal}"
 *           inspection_done="{inspection_done}"/>
 *       <ForceSuccess name="NavToWall">
 *         <RetryUntilSuccessful num_attempts="3">
 *           <NavigateToPose goal="{nav_goal}"/>
 *         </RetryUntilSuccessful>
 *       </ForceSuccess>
 *     </Sequence>
 *   </KeepRunningUntilFailure>
 * @endcode
 *
 * Because a stateful Sequence skips child[0] while child[1] is RUNNING,
 * this node is ticked exactly once per viewpoint leg (on the tick that starts
 * the leg), so no hysteresis or duplicate-goal logic is needed.
 *
 * Parameters (declared on the parent lifecycle node):
 *   wall_inspection.standoff_m         (double) — distance from wall to place
 *                                                 viewpoint [m] (default 0.6)
 *   wall_inspection.cluster_distance_m (double) — spatial bucket size for
 *                                                 candidate merging [m]
 *                                                 (default 1.0)
 *   wall_inspection.min_segment_cells  (int)    — minimum candidate count per
 *                                                 cluster to keep it (default 3)
 *
 * BT ports:
 *   Input:  "robot_pose"      — geometry_msgs::msg::PoseStamped
 *   Output: "nav_goal"        — geometry_msgs::msg::PoseStamped
 *   Output: "inspection_done" — bool
 *
 * Publishes /hermes/wall_viewpoints (visualization_msgs/MarkerArray) for RViz.
 */
class SelectWallViewpointNode : public BT::ActionNodeBase
{
public:
  SelectWallViewpointNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory as "SelectWallViewpoint".
  static void registerWithFactory(
    BT::BehaviorTreeFactory & factory,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  BT::NodeStatus tick() override;

  void halt() override {}

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  // Costmap subscription (same topic as SearchFrontiersNode)
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  nav2_msgs::msg::Costmap::SharedPtr latest_costmap_;

  // Visualisation publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Parameters
  double standoff_m_{0.6};
  double cluster_distance_m_{1.0};
  int min_segment_cells_{3};

  // Computed viewpoint list (filled once, then consumed sequentially)
  std::vector<geometry_msgs::msg::PoseStamped> viewpoints_;
  std::size_t current_idx_{0};
  bool viewpoints_computed_{false};

  /// @brief Compute and store the ordered wall-viewpoint list from the costmap.
  /// @param costmap    Latest global costmap.
  /// @param robot_pose Current robot pose (used for nearest-neighbour ordering).
  void computeViewpoints(
    const nav2_msgs::msg::Costmap & costmap,
    const geometry_msgs::msg::PoseStamped & robot_pose);

  /// @brief Publish the remaining viewpoints as RViz sphere markers.
  void publishMarkers();
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__SELECT_WALL_VIEWPOINT_NODE_HPP_
