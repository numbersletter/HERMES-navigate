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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_NODE_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace hermes_navigate
{

/**
 * @class ReturnToStartNode
 * @brief BT action node that builds the return-to-start waypoint list from the
 *        robot's breadcrumb trail and writes it onto the blackboard.
 *
 * The node reads the exploration breadcrumbs recorded by HermesNavigateNode,
 * reverses them so the robot retraces its path, then appends the original
 * start pose as the final destination.  The resulting waypoints list is
 * written to the "waypoints" output port for NavigateThroughWaypointsNode to
 * consume via the Nav2 NavigateThroughPoses action server.
 *
 * If no breadcrumbs are available the waypoints list contains only the start
 * pose, allowing Nav2 to plan a direct path home.
 *
 * BT ports:
 *   Input:  "start_pose"  — geometry_msgs::msg::PoseStamped
 *   Input:  "breadcrumbs" — std::vector<geometry_msgs::msg::PoseStamped>
 *   Output: "waypoints"   — std::vector<geometry_msgs::msg::PoseStamped>
 */
class ReturnToStartNode : public BT::SyncActionNode
{
public:
  ReturnToStartNode(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory.
  static void registerWithFactory(BT::BehaviorTreeFactory & factory);

  BT::NodeStatus tick() override;
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__RETURN_TO_START_NODE_HPP_

