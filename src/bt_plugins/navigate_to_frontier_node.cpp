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

#include "hermes_navigate/bt_plugins/navigate_to_frontier_node.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

NavigateToFrontierNode::NavigateToFrontierNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::StatefulActionNode(name, config),
  parent_(parent)
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("NavigateToFrontierNode: parent node expired.");
  }

  action_client_ = rclcpp_action::create_client<NavigateToPose>(
    node, "navigate_to_pose");

  // Load navigate_with_replanning.xml so it can be passed as the behavior_tree
  // field in every NavigateToPose goal we send.
  const std::string xml_path =
    ament_index_cpp::get_package_share_directory("hermes_navigate") +
    "/behavior_trees/navigate_with_replanning.xml";
  std::ifstream xml_file(xml_path);
  if (!xml_file.is_open()) {
    throw std::runtime_error(
      "NavigateToFrontierNode: could not open '" + xml_path + "'.");
  }
  std::ostringstream ss;
  ss << xml_file.rdbuf();
  navigate_bt_xml_ = ss.str();
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList NavigateToFrontierNode::providedPorts()
{
  return {BT::InputPort<geometry_msgs::msg::PoseStamped>("goal")};
}

// ─── onStart ─────────────────────────────────────────────────────────────────

BT::NodeStatus NavigateToFrontierNode::onStart()
{
  auto node = parent_.lock();

  auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal_res) {
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "NavigateToFrontierNode: no goal on blackboard.");
    }
    return BT::NodeStatus::FAILURE;
  }

  const auto & goal_pose = goal_res.value();

  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "NavigateToFrontierNode: navigate_to_pose server not available.");
    }
    return BT::NodeStatus::FAILURE;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = goal_pose;
  goal_msg.behavior_tree = navigate_bt_xml_;

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
  last_sent_goal_ = goal_pose;
  action_client_->async_send_goal(goal_msg, send_goal_options);

  if (node) {
    RCLCPP_DEBUG(node->get_logger(),
      "NavigateToFrontierNode: navigating to (%.2f, %.2f).",
      goal_pose.pose.position.x, goal_pose.pose.position.y);
  }

  return BT::NodeStatus::RUNNING;
}

// ─── onRunning ────────────────────────────────────────────────────────────────

BT::NodeStatus NavigateToFrontierNode::onRunning()
{
  // If the goal changed on the blackboard, cancel the current goal and
  // return FAILURE so the pipeline can re-send with the new goal.
  auto goal_res = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (goal_res) {
    const auto & new_goal = goal_res.value();
    if (new_goal.pose.position.x != last_sent_goal_.pose.position.x ||
      new_goal.pose.position.y != last_sent_goal_.pose.position.y)
    {
      onHalted();
      return BT::NodeStatus::FAILURE;
    }
  }

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

void NavigateToFrontierNode::onHalted()
{
  if (goal_handle_ && (state_ == State::PENDING || state_ == State::RUNNING)) {
    action_client_->async_cancel_goal(goal_handle_);
  }
  state_ = State::IDLE;
  goal_handle_.reset();
}

// ─── Factory registration ─────────────────────────────────────────────────────

void NavigateToFrontierNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  factory.registerBuilder<NavigateToFrontierNode>(
    "NavigateToFrontier",
    [parent](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateToFrontierNode>(name, config, parent);
    });
}

}  // namespace hermes_navigate
