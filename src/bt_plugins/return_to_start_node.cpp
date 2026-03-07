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

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

ReturnToStartNode::ReturnToStartNode(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
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
  setOutput("goal", start_pose_res.value());
  return BT::NodeStatus::SUCCESS;
}

// ─── Factory registration ─────────────────────────────────────────────────────

void ReturnToStartNode::registerWithFactory(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<ReturnToStartNode>("ReturnToStart");
}

}  // namespace hermes_navigate

