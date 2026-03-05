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

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

ReturnToStartNode::ReturnToStartNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::StatefulActionNode(name, config),
  parent_(parent)
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("ReturnToStartNode: parent node expired.");
  }

  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node, "navigate_to_pose");
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList ReturnToStartNode::providedPorts()
{
  return {BT::InputPort<geometry_msgs::msg::PoseStamped>("start_pose")};
}

// ─── onStart ─────────────────────────────────────────────────────────────────

BT::NodeStatus ReturnToStartNode::onStart()
{
  auto node = parent_.lock();

  auto start_pose_res = getInput<geometry_msgs::msg::PoseStamped>("start_pose");
  if (!start_pose_res) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(),
        "ReturnToStartNode: 'start_pose' not set on blackboard.");
    }
    return BT::NodeStatus::FAILURE;
  }

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(),
        "ReturnToStartNode: navigate_to_pose action server not available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = start_pose_res.value();
  goal_msg.behavior_tree = "";

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

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
    RCLCPP_INFO(node->get_logger(), "ReturnToStartNode: navigating back to start pose.");
  }

  return BT::NodeStatus::RUNNING;
}

// ─── onRunning ────────────────────────────────────────────────────────────────

BT::NodeStatus ReturnToStartNode::onRunning()
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

void ReturnToStartNode::onHalted()
{
  if (goal_handle_ && (state_ == State::PENDING || state_ == State::RUNNING)) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  state_ = State::IDLE;
  goal_handle_.reset();
}

}  // namespace hermes_navigate
