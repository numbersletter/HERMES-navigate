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

#ifndef HERMES_NAVIGATE__BT_PLUGINS__NAVIGATE_TO_FRONTIER_NODE_HPP_
#define HERMES_NAVIGATE__BT_PLUGINS__NAVIGATE_TO_FRONTIER_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace hermes_navigate
{

/**
 * @class NavigateToFrontierNode
 * @brief BT action node that sends a navigation goal to the Nav2
 *        NavigateToPose action server.
 *
 * Reads the current frontier goal from the "goal" blackboard entry (set by
 * SelectFrontierNode) and forwards it to the Nav2 stack.
 *
 * Returns RUNNING while navigation is in progress, SUCCESS on arrival, and
 * FAILURE if the action server rejects the goal or navigation fails.
 *
 * BT ports:
 *   Input: "goal"  — geometry_msgs::msg::PoseStamped
 */
class NavigateToFrontierNode : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToFrontierNode(
    const std::string & name,
    const BT::NodeConfig & config,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  static BT::PortsList providedPorts();

  /// @brief Register this node type with the BT factory.
  static void registerWithFactory(
    BT::BehaviorTreeFactory & factory,
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandle::SharedPtr goal_handle_;
  std::string navigate_bt_xml_;

  geometry_msgs::msg::PoseStamped last_sent_goal_;

  enum class State { IDLE, PENDING, RUNNING, DONE_SUCCESS, DONE_FAILURE };
  State state_{State::IDLE};
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__BT_PLUGINS__NAVIGATE_TO_FRONTIER_NODE_HPP_
