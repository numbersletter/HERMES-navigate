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

#include "hermes_navigate/bt_plugins/return_to_start_node.hpp"

#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

ReturnToStartNode::ReturnToStartNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp::Logger logger)
: BT::SyncActionNode(name, config),
  logger_(logger)
{
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList ReturnToStartNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus ReturnToStartNode::tick()
{
  auto start_pose_res = getInput<geometry_msgs::msg::PoseStamped>("start_pose");
  if (!start_pose_res) {
    throw BT::RuntimeError(
      "ReturnToStartNode: missing required port 'start_pose': ", start_pose_res.error());
  }
  // Write the start pose as the navigation goal.  The NavigateToPose BT node
  // in the same branch reads {nav_goal} and calls the navigate_to_pose action
  // server; Nav2 plans the path autonomously.
  const auto & start_pose = start_pose_res.value();
  RCLCPP_INFO(logger_,
    "ReturnToStartNode: setting nav_goal to start pose at (%.2f, %.2f).",
    start_pose.pose.position.x, start_pose.pose.position.y);
  setOutput("goal", start_pose);
  return BT::NodeStatus::SUCCESS;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void ReturnToStartNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp::Logger logger)
{
  factory.registerBuilder<ReturnToStartNode>(
    "ReturnToStart",
    [logger](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ReturnToStartNode>(name, config, logger);
    });
}

}  // namespace hermes_navigate

