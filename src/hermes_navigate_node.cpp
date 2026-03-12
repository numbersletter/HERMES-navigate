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
#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_behavior_tree/plugins/action/navigate_to_pose_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "hermes_navigate/bt_plugins/return_to_start_condition.hpp"
#include "hermes_navigate/bt_plugins/search_frontiers_node.hpp"
#include "hermes_navigate/bt_plugins/assign_costs_node.hpp"
#include "hermes_navigate/bt_plugins/select_frontier_node.hpp"
#include "hermes_navigate/bt_plugins/return_to_start_node.hpp"
#include "hermes_navigate/bt_plugins/blacklist_frontier_node.hpp"

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

  // ── Wait for Nav2 action server ──────────────────────────────────────────
  // We presume Nav2 is launched before HERMES-navigate.  Block here until
  // navigate_to_pose is ready so that no frontier calculation or BT execution
  // begins before Nav2 is operational.
  const auto timeout = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(nav2_server_timeout_s_));

  RCLCPP_INFO(get_logger(),
    "HermesNavigateNode: waiting for navigate_to_pose action server (timeout %.0f s)…",
    nav2_server_timeout_s_);

  auto nav_to_pose_client =
    rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, "navigate_to_pose");

  if (!nav_to_pose_client->wait_for_action_server(timeout)) {
    RCLCPP_ERROR(get_logger(),
      "HermesNavigateNode: navigate_to_pose server not available after %.0f s. "
      "Ensure Nav2 is running before starting HERMES-navigate.",
      nav2_server_timeout_s_);
    return CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(get_logger(), "HermesNavigateNode: navigate_to_pose server is ready.");

  // ── Wait for bt_navigator to be fully active ──────────────────────────────
  // wait_for_action_server() returns true as soon as bt_navigator registers
  // its action server (during its own configure phase).  However, the server
  // only accepts goals once bt_navigator has been *activated* by the Nav2
  // lifecycle manager.  We use a dedicated helper node so we can spin its
  // executor independently without deadlocking the HERMES lifecycle executor.
  {
    auto helper_node = std::make_shared<rclcpp::Node>(
      std::string(get_name()) + "_nav2_wait_helper");

    auto get_state_client =
      helper_node->create_client<lifecycle_msgs::srv::GetState>(
        "/bt_navigator/get_state");

    const auto deadline =
      helper_node->now() + rclcpp::Duration::from_seconds(nav2_server_timeout_s_);

    RCLCPP_INFO(get_logger(),
      "HermesNavigateNode: waiting for bt_navigator to become active…");

    bool bt_nav_active = false;
    while (rclcpp::ok() && helper_node->now() < deadline) {
      // Cap each wait to the remaining budget so we don't overshoot the deadline.
      const auto remaining_ns = std::chrono::nanoseconds(
        (deadline - helper_node->now()).nanoseconds());
      const auto cap_1s = std::chrono::nanoseconds(std::chrono::seconds(1));
      const auto wait_timeout = std::min(remaining_ns, cap_1s);

      // Wait up to 1 s (or remaining budget) for the GetState service to appear.
      if (!get_state_client->wait_for_service(wait_timeout)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "HermesNavigateNode: /bt_navigator/get_state service not yet available.");
        continue;
      }

      auto req = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto future = get_state_client->async_send_request(req);

      // Spin helper_node's executor to drive the response — this does NOT
      // involve the HERMES lifecycle executor, so there is no deadlock risk.
      const auto spin_timeout = std::min(
        std::chrono::nanoseconds((deadline - helper_node->now()).nanoseconds()),
        cap_1s);
      if (rclcpp::spin_until_future_complete(helper_node, future, spin_timeout)
          == rclcpp::FutureReturnCode::SUCCESS)
      {
        if (future.get()->current_state.id ==
            lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          bt_nav_active = true;
          break;
        }
      }

      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "HermesNavigateNode: bt_navigator not yet active; retrying…");
      rclcpp::sleep_for(std::chrono::milliseconds(200));
    }

    if (!bt_nav_active) {
      RCLCPP_ERROR(get_logger(),
        "HermesNavigateNode: bt_navigator did not become active within %.0f s. "
        "Ensure Nav2 is fully started before activating HERMES-navigate.",
        nav2_server_timeout_s_);
      return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "HermesNavigateNode: bt_navigator is active.");
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
  blackboard_->set("nav_goal",          start_pose_);  // initialise to avoid unset port errors
  blackboard_->set("blacklisted_goals",
    std::vector<geometry_msgs::msg::PoseStamped>{});  // grows as Nav2 fails

  // Nav2 BT action/service nodes (e.g. NavigateToPoseAction) retrieve these
  // entries from the blackboard in their constructors.  "node" must be a plain
  // rclcpp::Node::SharedPtr — lifecycle nodes are not accepted.
  bt_client_node_ = std::make_shared<rclcpp::Node>(
    std::string(get_name()) + "_bt_client",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  blackboard_->set("node",                    bt_client_node_);
  blackboard_->set("bt_loop_duration",        std::chrono::milliseconds(10));
  blackboard_->set("wait_for_service_timeout", std::chrono::milliseconds(1000));

  // ── Register BT nodes with this node as context ───────────────────────────
  rclcpp_lifecycle::LifecycleNode::WeakPtr self = shared_from_this();
  factory_.registerNodeType<nav2_behavior_tree::PipelineSequence>("PipelineSequence");
  factory_.registerNodeType<nav2_behavior_tree::RecoveryNode>("RecoveryNode");
  factory_.registerNodeType<nav2_behavior_tree::RateController>("RateController");

  // NavigateToPoseAction needs (xml_tag_name, action_name, config), therefore
  // it must be registered with a custom builder instead of registerNodeType.
  BT::NodeBuilder navigate_to_pose_builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<nav2_behavior_tree::NavigateToPoseAction>(
        name, "navigate_to_pose", config);
    };
  factory_.registerBuilder<nav2_behavior_tree::NavigateToPoseAction>(
    "NavigateToPose", navigate_to_pose_builder);

  ReturnToStartCondition::registerWithFactory(factory_, get_logger());
  SearchFrontiersNode::registerWithFactory(factory_, self);
  AssignCostsNode::registerWithFactory(factory_, self);
  SelectFrontierNode::registerWithFactory(factory_, self);
  ReturnToStartNode::registerWithFactory(factory_, get_logger());
  BlacklistFrontierNode::registerWithFactory(factory_, get_logger());

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
  bt_client_node_.reset();
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
    "HermesNavigateNode: stop requested — robot will navigate to start pose.");

  // Signal the BT to enter the return-to-start branch.
  // ReturnToStartNode sets nav_goal = start_pose; NavigateToPose then calls
  // the navigate_to_pose action server and lets Nav2 plan the path home.
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
  }

  try {
    BT::NodeStatus status = tree_.tickOnce();

    if (status == BT::NodeStatus::RUNNING) {
      RCLCPP_DEBUG(get_logger(), "HermesNavigateNode: BT tick → RUNNING.");
    } else if (status == BT::NodeStatus::FAILURE) {
      // FAILURE means SelectFrontier found no viable frontier
      // (exploration_done=true) or an unexpected error occurred.  Either way,
      // stop ticking.  With KeepRunningUntilFailure wrapping the exploration
      // pipeline, successful navigation legs are converted to RUNNING and never
      // reach here — only a true end-of-exploration FAILURE does.
      bool exploration_done = false;
      if (!blackboard_->get("exploration_done", exploration_done)) {
        RCLCPP_WARN(get_logger(),
          "HermesNavigateNode: 'exploration_done' key missing from blackboard; "
          "assuming false.");
      }
      RCLCPP_INFO(get_logger(),
        "HermesNavigateNode: BT finished with status FAILURE "
        "(exploration_done=%s). Stopping tick timer.",
        exploration_done ? "true" : "false");
      tick_timer_->cancel();
    } else if (status == BT::NodeStatus::SUCCESS) {
      // SUCCESS means the return-to-start branch completed: the robot has
      // reached its start pose.  With KeepRunningUntilFailure wrapping the
      // exploration pipeline, exploration legs can never produce SUCCESS here —
      // they are converted to RUNNING by the decorator, so the tree stays alive
      // between legs without any manual restart.
      RCLCPP_INFO(get_logger(),
        "HermesNavigateNode: BT finished with status SUCCESS (returned to start). "
        "Stopping tick timer.");
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
