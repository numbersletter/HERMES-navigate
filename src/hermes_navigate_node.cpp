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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/utils/shared_library.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "hermes_navigate/bt_plugins/blackboard_check_bool_node.hpp"
#include "hermes_navigate/bt_plugins/search_frontiers_node.hpp"
#include "hermes_navigate/bt_plugins/assign_costs_node.hpp"
#include "hermes_navigate/bt_plugins/select_frontier_node.hpp"
#include "hermes_navigate/bt_plugins/return_to_start_node.hpp"
#include "hermes_navigate/bt_plugins/navigate_to_frontier_node.hpp"

namespace hermes_navigate
{

// ─── Constructor ──────────────────────────────────────────────────────────────

HermesNavigateNode::HermesNavigateNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("hermes_navigate_node", options)
{
  // Declare parameters with defaults; values are read in on_configure so that
  // they can be overridden by the launch file or a YAML params file before the
  // node is configured.
  declare_parameter("bt_xml_file",      std::string(""));
  declare_parameter("bt_tick_rate_hz",  10.0);
  declare_parameter("base_frame",       std::string("base_link"));
  declare_parameter("map_frame",        std::string("map"));
  declare_parameter("robot_pose_topic", std::string("/robot_pose"));

  // Nav2 BT plugin libraries to load into the BehaviorTree factory.
  // This list mirrors the plugin_lib_names used by nav2_bt_navigator so that
  // all standard Nav2 BT condition/action/decorator/control nodes are available
  // inside any behavior-tree XML loaded by this node.
  declare_parameter<std::vector<std::string>>(
    "bt_plugins",
    {
      "nav2_compute_path_to_pose_action_bt_node",
      "nav2_compute_path_through_poses_action_bt_node",
      "nav2_smooth_path_action_bt_node",
      "nav2_follow_path_action_bt_node",
      "nav2_back_up_action_bt_node",
      "nav2_spin_action_bt_node",
      "nav2_wait_action_bt_node",
      "nav2_assisted_teleop_action_bt_node",
      "nav2_clear_costmap_service_bt_node",
      "nav2_is_stuck_condition_bt_node",
      "nav2_goal_reached_condition_bt_node",
      "nav2_goal_updated_condition_bt_node",
      "nav2_globally_updated_goal_condition_bt_node",
      "nav2_is_path_valid_condition_bt_node",
      "nav2_initial_pose_received_condition_bt_node",
      "nav2_reinitialize_global_localization_service_bt_node",
      "nav2_rate_controller_bt_node",
      "nav2_distance_controller_bt_node",
      "nav2_speed_controller_bt_node",
      "nav2_truncate_path_action_bt_node",
      "nav2_truncate_path_local_action_bt_node",
      "nav2_goal_updater_node_bt_node",
      "nav2_recovery_node_bt_node",
      "nav2_pipeline_sequence_bt_node",
      "nav2_round_robin_node_bt_node",
      "nav2_transform_available_condition_bt_node",
      "nav2_time_expired_condition_bt_node",
      "nav2_path_expiring_timer_condition",
      "nav2_distance_traveled_condition_bt_node",
      "nav2_single_trigger_bt_node",
      "nav2_goal_updated_controller_bt_node",
      "nav2_navigate_through_poses_action_bt_node",
      "nav2_navigate_to_pose_action_bt_node",
      "nav2_remove_passed_goals_action_bt_node",
      "nav2_planner_selector_bt_node",
      "nav2_controller_selector_bt_node",
      "nav2_goal_checker_selector_bt_node",
      "nav2_controller_cancel_bt_node",
      "nav2_path_longer_on_approach_bt_node",
      "nav2_wait_cancel_bt_node",
      "nav2_spin_cancel_bt_node",
      "nav2_back_up_cancel_bt_node",
      "nav2_assisted_teleop_cancel_bt_node",
      "nav2_drive_on_heading_cancel_bt_node",
      "nav2_is_battery_low_condition_bt_node",
    });
}

// ─── on_configure ─────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "HermesNavigateNode: configuring…");

  // Read parameters
  bt_tick_rate_hz_ = get_parameter("bt_tick_rate_hz").as_double();
  base_frame_      = get_parameter("base_frame").as_string();
  map_frame_       = get_parameter("map_frame").as_string();

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

  // ── Register BT nodes with this node as context ───────────────────────────
  registerBTNodes();

  // ── Load BT tree from XML ─────────────────────────────────────────────────
  if (!loadBehaviorTree(bt_xml_file_)) {
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "HermesNavigateNode configured. Robot is stationary.");
  return CallbackReturn::SUCCESS;
}

// ─── on_activate ──────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "HermesNavigateNode: activating — exploration begins.");

  blackboard_->set("return_to_start", false);
  blackboard_->set("exploration_done", false);

  // Start the BT tick timer
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
    "HermesNavigateNode: deactivating — robot will return to start pose.");

  // Signal the BT to return to start
  blackboard_->set("return_to_start", true);

  // The tick timer keeps running so the ReturnToStart branch can complete.
  // The launch system / lifecycle manager should call on_cleanup only after
  // checking that the tree is finished, or after a timeout.

  return CallbackReturn::SUCCESS;
}

// ─── on_cleanup ───────────────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
HermesNavigateNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  tick_timer_.reset();
  tree_ = BT::Tree{};  // destroy tree (halts all running nodes)
  blackboard_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  pose_sub_.reset();
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

// ─── registerBTNodes ──────────────────────────────────────────────────────────

void HermesNavigateNode::registerBTNodes()
{
  // ── Load Nav2 BT plugin libraries ─────────────────────────────────────────
  // Each library exposes BT_REGISTER_NODES which registers its node types
  // (e.g. PipelineSequence, RateController) with the factory.  Failures are
  // logged as warnings rather than errors so that a missing optional plugin
  // does not prevent the node from starting.
  const auto bt_plugins =
    get_parameter("bt_plugins").as_string_array();

  for (const auto & plugin : bt_plugins) {
    try {
      factory_.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
      RCLCPP_DEBUG(get_logger(),
        "HermesNavigateNode: registered Nav2 BT plugin '%s'.", plugin.c_str());
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(),
        "HermesNavigateNode: could not load Nav2 BT plugin '%s': %s",
        plugin.c_str(), e.what());
    }
  }

  // ── Register HERMES-specific BT nodes ─────────────────────────────────────
  // Capture a weak_ptr to this lifecycle node; BT nodes use it to access
  // ROS 2 parameters, create publishers, and load pluginlib plugins.
  rclcpp_lifecycle::LifecycleNode::WeakPtr self = shared_from_this();

  BlackboardCheckBool::registerWithFactory(factory_);
  SearchFrontiersNode::registerWithFactory(factory_, self);
  AssignCostsNode::registerWithFactory(factory_, self);
  SelectFrontierNode::registerWithFactory(factory_, self);
  ReturnToStartNode::registerWithFactory(factory_, self);
  NavigateToFrontierNode::registerWithFactory(factory_, self);
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
  // Update robot pose on the blackboard from the latest subscription message
  if (!latest_pose_.header.frame_id.empty()) {
    blackboard_->set("robot_pose", latest_pose_);
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
