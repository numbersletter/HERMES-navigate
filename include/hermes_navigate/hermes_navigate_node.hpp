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

#ifndef HERMES_NAVIGATE__HERMES_NAVIGATE_NODE_HPP_
#define HERMES_NAVIGATE__HERMES_NAVIGATE_NODE_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav2_behavior_tree/plugins/control/pipeline_sequence.hpp"
#include "nav2_behavior_tree/plugins/control/recovery_node.hpp"
#include "nav2_behavior_tree/plugins/decorator/rate_controller.hpp"



namespace hermes_navigate
{

/**
 * @class HermesNavigateNode
 * @brief Main lifecycle entry point for the HERMES autonomous exploration suite.
 *
 * Lifecycle behaviour:
 *   on_configure — Waits for Nav2 action servers; records start pose; loads BT
 *                  plugins and builds the HermesExplorationBT tree.
 *                  Robot does not move yet.
 *   on_activate  — Starts the BT tick timer and resets the breadcrumb trail.
 *                  Robot begins exploration.
 *   on_deactivate— Stops the BT tick timer and halts the tree (pause).
 *                  Robot stays where it is; call the ~/stop service to trigger
 *                  a return to the start pose.
 *   on_cleanup   — Destroys the tree and releases all resources.
 *   on_shutdown  — Graceful shutdown.
 *
 * Services:
 *   ~/stop (std_srvs/Trigger) — Signals the BT to navigate the robot back to
 *                               its start pose. Nav2 plans the path autonomously
 *                               via the navigate_to_pose action server.
 *                               Can be called while active or paused.
 *
 * Parameters (all declared in on_configure):
 *   bt_xml_file           (string) — Path to the BT XML; defaults to the
 *                                    installed hermes_exploration_bt.xml.
 *   bt_tick_rate_hz       (double) — BT tick frequency [Hz]  (default 10.0).
 *   base_frame            (string) — Robot base TF frame     (default "base_link").
 *   map_frame             (string) — Map TF frame            (default "map").
 *   robot_pose_topic      (string) — Topic for the current robot pose
 *                                    (default "/robot_pose").
 *   nav2_server_timeout_s (double) — Seconds to wait for Nav2 action server
 *                                    during configure (default 60.0).
 */
class HermesNavigateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit HermesNavigateNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // ── Lifecycle callbacks ────────────────────────────────────────────────────
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // ── BT machinery ──────────────────────────────────────────────────────────
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  /// @brief Load and build the BT tree from the configured XML file.
  bool loadBehaviorTree(const std::string & bt_xml_path);

  // ── Tick timer ────────────────────────────────────────────────────────────
  rclcpp::TimerBase::SharedPtr tick_timer_;

  /// @brief Called by the tick timer; advances the BT by one cycle.
  void tickTree();

  // ── TF (for start-pose capture) ───────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── Robot pose subscription ───────────────────────────────────────────────
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  geometry_msgs::msg::PoseStamped latest_pose_;

  // ── Stop service ──────────────────────────────────────────────────────────
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;

  /// @brief Handler for the ~/stop service.  Signals the BT to return to start.
  void handleStop(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  // ── Parameters ────────────────────────────────────────────────────────────
  std::string bt_xml_file_;
  double bt_tick_rate_hz_{10.0};
  std::string base_frame_{"base_link"};
  std::string map_frame_{"map"};
  double nav2_server_timeout_s_{60.0};

  // ── State ─────────────────────────────────────────────────────────────────
  geometry_msgs::msg::PoseStamped start_pose_;  ///< Captured on configure.
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__HERMES_NAVIGATE_NODE_HPP_
