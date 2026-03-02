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

#ifndef HERMES_NAVIGATE__COVERAGE_TRACKER_NODE_HPP_
#define HERMES_NAVIGATE__COVERAGE_TRACKER_NODE_HPP_

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace hermes_navigate
{

/**
 * @class CoverageTrackerNode
 * @brief Standalone lifecycle node that maintains a camera-coverage grid.
 *
 * The node subscribes to /map and tracks which cells have been observed by
 * the camera.  On each timer tick it looks up the current `map → base_link`
 * transform via TF2, projects the camera frustum (configurable FOV + max
 * range) onto the occupancy-grid frame, and marks covered cells.
 *
 * The resulting coverage grid is published on `/coverage_grid` as an
 * OccupancyGrid (0 = not viewed, 100 = viewed).
 *
 * Parameters (all have defaults):
 *   camera_fov_deg      (double)  — horizontal FOV of the camera [deg] (60°)
 *   camera_max_range_m  (double)  — max detection range [m] (5.0 m)
 *   update_rate_hz      (double)  — TF lookup / grid update rate [Hz] (10.0)
 *   base_frame          (string)  — robot base TF frame ("base_link")
 *   map_frame           (string)  — map TF frame ("map")
 *   camera_yaw_offset   (double)  — camera heading offset from base_link [rad] (0.0)
 */
class CoverageTrackerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit CoverageTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Lifecycle callbacks
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
  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscriptions / publishers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr coverage_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // State
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  std::vector<int8_t> coverage_data_;  ///< Flat grid; 0 = unseen, 100 = seen.

  // Parameters
  double camera_fov_rad_{};
  double camera_max_range_m_{};
  std::string base_frame_{"base_link"};
  std::string map_frame_{"map"};
  double camera_yaw_offset_{0.0};

  void onMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void updateCoverage();

  /// @brief Mark grid cells visible from (robot_x, robot_y) with heading yaw.
  void markFrustum(double robot_x, double robot_y, double yaw);
};

}  // namespace hermes_navigate

#endif  // HERMES_NAVIGATE__COVERAGE_TRACKER_NODE_HPP_
