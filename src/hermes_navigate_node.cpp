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

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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
  // Capture a weak_ptr to this lifecycle node; BT nodes use it to access
  // ROS 2 parameters, create publishers, and load pluginlib plugins.
  rclcpp_lifecycle::LifecycleNode::WeakPtr self = shared_from_this();

  // ── New structured BT nodes (plugin-aware) ────────────────────────────────
  //
  // SearchFrontiersNode  — comprising plugin: frontier_search.plugin
  //   (default: hermes_navigate::WavefrontFrontierDetector)
  factory_.registerBuilder<SearchFrontiersNode>(
    "SearchFrontiers",
    [self](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SearchFrontiersNode>(name, config, self);
    });

  // AssignCostsNode  — comprising plugins: cost_plugins list
  //   (default: InformationGainPlugin, EuclideanPlugin)
  factory_.registerBuilder<AssignCostsNode>(
    "AssignCosts",
    [self](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<AssignCostsNode>(name, config, self);
    });

  // SelectFrontierNode
  factory_.registerBuilder<SelectFrontierNode>(
    "SelectFrontier",
    [self](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SelectFrontierNode>(name, config, self);
    });

  // ReturnToStartNode
  factory_.registerBuilder<ReturnToStartNode>(
    "ReturnToStart",
    [self](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<ReturnToStartNode>(name, config, self);
    });

  // NavigateToFrontierNode — sends the selected frontier goal to Nav2
  factory_.registerBuilder<NavigateToFrontierNode>(
    "NavigateToFrontier",
    [self](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<NavigateToFrontierNode>(name, config, self);
    });
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
