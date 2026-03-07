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

#include "hermes_navigate/hermes_navigate_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "hermes_navigate/bt_plugins/return_to_start_condition.hpp"
#include "hermes_navigate/bt_plugins/search_frontiers_node.hpp"
#include "hermes_navigate/bt_plugins/assign_costs_node.hpp"
#include "hermes_navigate/bt_plugins/select_frontier_node.hpp"
#include "hermes_navigate/bt_plugins/return_to_start_node.hpp"
#include "hermes_navigate/bt_plugins/navigate_to_frontier_node.hpp"
#include "hermes_navigate/bt_plugins/navigate_through_waypoints_node.hpp"

namespace hermes_navigate
{

// ─── Constructor ──────────────────────────────────────────────────────────────

HermesNavigateNode::HermesNavigateNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hermes_navigate_node", options)
{
  // Declare parameters with defaults; values are read in on_configure so that
  // they can be overridden by the launch file or a YAML params file before the
  // node is configured.
  declare_parameter("bt_xml_file",            std::string(""));
  declare_parameter("bt_tick_rate_hz",        10.0);
  declare_parameter("base_frame",             std::string("base_link"));
  declare_parameter("map_frame",              std::string("map"));
  declare_parameter("robot_pose_topic",       std::string("/robot_pose"));
  declare_parameter("breadcrumb_spacing_m",   2.0);
  declare_parameter("nav2_server_timeout_s",  60.0);
}

// ─── on_configure ─────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "HermesNavigateNode: configuring…");

  // Read parameters
  bt_tick_rate_hz_        = get_parameter("bt_tick_rate_hz").as_double();
  base_frame_             = get_parameter("base_frame").as_string();
  map_frame_              = get_parameter("map_frame").as_string();
  breadcrumb_spacing_m_   = get_parameter("breadcrumb_spacing_m").as_double();
  nav2_server_timeout_s_  = get_parameter("nav2_server_timeout_s").as_double();

  bt_xml_file_ = get_parameter("bt_xml_file").as_string();
  if (bt_xml_file_.empty()) {
    try {
      bt_xml_file_ = ament_index_cpp::get_package_share_directory("hermes_navigate") +
        "/behavior_trees/hermes_exploration_bt.xml";
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "Could not find hermes_navigate share dir: %s", e.what());
      return CallbackReturn::FAILURE;
    }
  }

  // ── Wait for Nav2 action servers ─────────────────────────────────────────
  // We presume Nav2 is launched before HERMES-navigate.  Block here until
  // the required navigators are ready so that no frontier calculation or BT
  // execution begins before Nav2 is operational.
  const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(nav2_server_timeout_s_));

  RCLCPP_INFO(get_logger(),
    "HermesNavigateNode: waiting for Nav2 action servers (timeout %.0f s)…",
    nav2_server_timeout_s_);

  auto nav_to_pose_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

  auto nav_through_poses_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
      this, "navigate_through_poses");

  if (!nav_to_pose_client->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(get_logger(),
      "HermesNavigateNode: navigate_to_pose server not available after %.0f s. "
      "Ensure Nav2 is running before starting HERMES-navigate.",
      nav2_server_timeout_s_);
    return CallbackReturn::FAILURE;
  }

  if (!nav_through_poses_client->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(get_logger(),
      "HermesNavigateNode: navigate_through_poses server not available after %.0f s. "
      "Ensure Nav2 is running before starting HERMES-navigate.",
      nav2_server_timeout_s_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "HermesNavigateNode: Nav2 action servers are ready.");

  // ── TF listener ──────────────────────────────────────────────────────────
  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── Robot pose subscription ───────────────────────────────────────────────
  const std::string pose_topic = get_parameter("robot_pose_topic").as_string();
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    pose_topic,
    rclcpp::QoS(1),
    [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      latest_pose_ = *msg;
    });

  // ── Capture start pose via TF ─────────────────────────────────────────────
  // Wait up to 5 s for a transform to become available.
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero,
      tf2::durationFromSec(5.0));

    start_pose_.header.frame_id = map_frame_;
    start_pose_.header.stamp    = now();
    start_pose_.pose.position.x = tf_stamped.transform.translation.x;
    start_pose_.pose.position.y = tf_stamped.transform.translation.y;
    start_pose_.pose.position.z = 0.0;
    start_pose_.pose.orientation = tf_stamped.transform.rotation;

    RCLCPP_INFO(get_logger(),
      "HermesNavigateNode: start pose captured at (%.2f, %.2f).",
      start_pose_.pose.position.x, start_pose_.pose.position.y);
  } catch (const tf2::TransformException & ex) {
    // TF not available yet (e.g., SLAM not started) — use map origin as fallback
    RCLCPP_WARN(get_logger(),
      "HermesNavigateNode: TF lookup failed (%s); using map origin as start pose.",
      ex.what());
    start_pose_.header.frame_id = map_frame_;
    start_pose_.header.stamp    = now();
    start_pose_.pose.orientation.w = 1.0;
  }

  // ── Build BT shared blackboard ────────────────────────────────────────────
  blackboard_ = BT::Blackboard::create();
  blackboard_->set("start_pose",        start_pose_);
  blackboard_->set("exploration_done",  false);
  blackboard_->set("return_to_start",   false);
  blackboard_->set("nav_goal",          start_pose_);  // initialise to avoid unset port errors
  blackboard_->set("breadcrumbs",       std::vector<geometry_msgs::msg::PoseStamped>{});
  blackboard_->set("return_waypoints",  std::vector<geometry_msgs::msg::PoseStamped>{});

  // ── Register BT nodes with this node as context ───────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr self = shared_from_this();
  factory_.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
  factory_.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
  factory_.registerNodeType<nav2_behavior_tree::RateController>("RateController");

  ReturnToStartCondition::registerWithFactory(factory_);
  SearchFrontiersNode::registerWithFactory(factory_, self);
  AssignCostsNode::registerWithFactory(factory_, self);
  SelectFrontierNode::registerWithFactory(factory_, self);
  ReturnToStartNode::registerWithFactory(factory_);
  NavigateToFrontierNode::registerWithFactory(factory_, self);
  NavigateThroughWaypointsNode::registerWithFactory(factory_, self);

  // ── Load BT tree from XML ─────────────────────────────────────────────────
  if (!loadBehaviorTree(bt_xml_file_)) {
    return CallbackReturn::FAILURE;
  }

  // ── Stop service ─────────────────────────────────────────────────────────
  // Available in all post-configure states so the operator can trigger a
  // return-to-start whether the robot is actively exploring or paused.
  stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/stop",
    [this](
      const std_srvs::srv::Trigger::Request::SharedPtr req,
      std_srvs::srv::Trigger::Response::SharedPtr res)
    {
      handleStop(req, res);
    });

  RCLCPP_INFO(get_logger(), "HermesNavigateNode configured. Robot is stationary.");
  return CallbackReturn::SUCCESS;
}

// ─── on_activate ──────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "HermesNavigateNode: activating — exploration begins.");

  // Reset exploration state on the blackboard.
  blackboard_->set("return_to_start",  false);
  blackboard_->set("exploration_done", false);

  // Clear the breadcrumb trail for a fresh exploration run.
  breadcrumbs_.clear();
  blackboard_->set("breadcrumbs",      std::vector<geometry_msgs::msg::PoseStamped>{});
  blackboard_->set("return_waypoints", std::vector<geometry_msgs::msg::PoseStamped>{});

  // Start the BT tick timer.
  auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / bt_tick_rate_hz_));
  tick_timer_ = create_wall_timer(period_ns, [this]() {tickTree();});

  return CallbackReturn::SUCCESS;
}

// ─── on_deactivate ────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(),
    "HermesNavigateNode: deactivating — exploration paused. "
    "Call ~/stop to return the robot to its start pose.");

  // Stop the tick timer and halt all running BT nodes (cancels any active
  // navigation goals via the nodes' onHalted() methods).
  tick_timer_.reset();
  tree_.haltTree();

  return CallbackReturn::SUCCESS;
}

// ─── on_cleanup ───────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  tick_timer_.reset();
  stop_srv_.reset();
  tree_ = BT::Tree{};  // destroy tree (halts all running nodes)
  blackboard_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  pose_sub_.reset();
  breadcrumbs_.clear();
  RCLCPP_INFO(get_logger(), "HermesNavigateNode cleaned up.");
  return CallbackReturn::SUCCESS;
}

// ─── on_shutdown ──────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  tick_timer_.reset();
  return CallbackReturn::SUCCESS;
}

// ─── handleStop ──────────────────────────────────────────────────────────────

void HermesNavigateNode::handleStop(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (!blackboard_) {
    response->success = false;
    response->message = "Node not yet configured.";
    return;
  }

  RCLCPP_INFO(get_logger(),
    "HermesNavigateNode: stop requested — robot will retrace %zu breadcrumbs "
    "to return to start.",
    breadcrumbs_.size());

  // Signal the BT to enter the return-to-start branch.
  // The ReturnToStartNode will read {breadcrumbs} from the blackboard,
  // reverse them, append the start pose, and pass the list to
  // NavigateThroughWaypointsNode.
  blackboard_->set("return_to_start", true);

  // (Re)start the tick timer.  on_deactivate resets it to nullptr; tickTree()
  // calls cancel() when the BT finishes.  Both cases are handled by always
  // resetting and creating a fresh timer here.
  tick_timer_.reset();
  auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / bt_tick_rate_hz_));
  tick_timer_ = create_wall_timer(period_ns, [this]() {tickTree();});
  RCLCPP_INFO(get_logger(),
    "HermesNavigateNode: tick timer (re)started for return-to-start journey.");

  response->success = true;
  response->message = "Return to start initiated.";
}

// ─── loadBehaviorTree ─────────────────────────────────────────────────────────

bool HermesNavigateNode::loadBehaviorTree(const std::string & bt_xml_path)
{
  try {
    tree_ = factory_.createTreeFromFile(bt_xml_path, blackboard_);
    RCLCPP_INFO(get_logger(), "HermesNavigateNode: BT loaded from '%s'.",
      bt_xml_path.c_str());
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to load BT from '%s': %s",
      bt_xml_path.c_str(), e.what());
    return false;
  }
}

// ─── tickTree ─────────────────────────────────────────────────────────────────

void HermesNavigateNode::tickTree()
{
  // Update robot pose on the blackboard from the latest subscription message.
  if (!latest_pose_.header.frame_id.empty()) {
    blackboard_->set("robot_pose", latest_pose_);

    // ── Breadcrumb recording ──────────────────────────────────────────────
    // Append a breadcrumb each time the robot has moved at least
    // breadcrumb_spacing_m_ from the previous recorded position.  The trail
    // is used by ReturnToStartNode to build the reverse return path.
    // Use breadcrumbs_.empty() as the initialisation guard so the first pose
    // is always captured regardless of its frame_id value.
    bool need_crumb = breadcrumbs_.empty();
    if (!need_crumb) {
      const double dx =
        latest_pose_.pose.position.x - breadcrumbs_.back().pose.position.x;
      const double dy =
        latest_pose_.pose.position.y - breadcrumbs_.back().pose.position.y;
      need_crumb = std::sqrt(dx * dx + dy * dy) >= breadcrumb_spacing_m_;
    }
    if (need_crumb) {
      breadcrumbs_.push_back(latest_pose_);
      blackboard_->set("breadcrumbs", breadcrumbs_);
    }
  }

  try {
    BT::NodeStatus status = tree_.tickOnce();

    if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
      RCLCPP_INFO(get_logger(),
        "HermesNavigateNode: BT finished with status %s. Stopping tick timer.",
        status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
      tick_timer_->cancel();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "HermesNavigateNode: BT tick threw: %s", e.what());
    tick_timer_->cancel();
  }
}

}  // namespace hermes_navigate

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hermes_navigate::HermesNavigateNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
