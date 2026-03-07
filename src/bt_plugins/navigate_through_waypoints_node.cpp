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

#include "hermes_navigate/bt_plugins/navigate_through_waypoints_node.hpp"

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

NavigateThroughWaypointsNode::NavigateThroughWaypointsNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::StatefulActionNode(name, config),
  parent_(parent)
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("NavigateThroughWaypointsNode: parent node expired.");
  }

  action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(
    node, "navigate_through_poses");
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList NavigateThroughWaypointsNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints"),
  };
}

// ─── onStart ──────────────────────────────────────────────────────────────────

BT::NodeStatus NavigateThroughWaypointsNode::onStart()
{
  auto node = parent_.lock();

  auto wp_res = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
  if (!wp_res || wp_res.value().empty()) {
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "NavigateThroughWaypointsNode: no waypoints on blackboard.");
    }
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "NavigateThroughWaypointsNode: navigate_through_poses server not available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  NavigateThroughPoses::Goal goal_msg;
  goal_msg.poses = wp_res.value();

  auto send_goal_options =
    rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();

  send_goal_options.goal_response_callback =
    [this](const GoalHandle::SharedPtr & handle) {
      if (!handle) {
        state_ = State::DONE_FAILURE;
      } else {
        goal_handle_ = handle;
        state_ = State::RUNNING;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandle::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        state_ = State::DONE_SUCCESS;
      } else {
        state_ = State::DONE_FAILURE;
      }
    };

  state_ = State::PENDING;
  action_client_->async_send_goal(goal_msg, send_goal_options);

  if (node) {
    RCLCPP_INFO(node->get_logger(),
      "NavigateThroughWaypointsNode: navigating through %zu waypoints.",
      wp_res.value().size());
  }

  return BT::NodeStatus::RUNNING;
}

// ─── onRunning ────────────────────────────────────────────────────────────────

BT::NodeStatus NavigateThroughWaypointsNode::onRunning()
{
  switch (state_) {
    case State::DONE_SUCCESS:
      return BT::NodeStatus::SUCCESS;
    case State::DONE_FAILURE:
      return BT::NodeStatus::FAILURE;
    default:
      return BT::NodeStatus::RUNNING;
  }
}

// ─── onHalted ────────────────────────────────────────────────────────────────

void NavigateThroughWaypointsNode::onHalted()
{
  if (goal_handle_ && (state_ == State::PENDING || state_ == State::RUNNING)) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  state_ = State::IDLE;
  goal_handle_.reset();
}

// ─── Factory registration ─────────────────────────────────────────────────────

void NavigateThroughWaypointsNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  factory.registerBuilder<NavigateThroughWaypointsNode>(
    "NavigateThroughWaypoints",
    [parent](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateThroughWaypointsNode>(name, config, parent);
    });
}

}  // namespace hermes_navigate
