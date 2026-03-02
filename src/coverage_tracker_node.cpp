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

#include "hermes_navigate/coverage_tracker_node.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

CoverageTrackerNode::CoverageTrackerNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("coverage_tracker_node", options)
{
  declare_parameter("camera_fov_deg",     60.0);
  declare_parameter("camera_max_range_m", 5.0);
  declare_parameter("update_rate_hz",     10.0);
  declare_parameter("base_frame",         std::string("base_link"));
  declare_parameter("map_frame",          std::string("map"));
  declare_parameter("camera_yaw_offset",  0.0);
}

// ─── Lifecycle callbacks ──────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CoverageTrackerNode::on_configure(const rclcpp_lifecycle::State &)
{
  camera_fov_rad_       = get_parameter("camera_fov_deg").as_double() * M_PI / 180.0;
  camera_max_range_m_   = get_parameter("camera_max_range_m").as_double();
  base_frame_           = get_parameter("base_frame").as_string();
  map_frame_            = get_parameter("map_frame").as_string();
  camera_yaw_offset_    = get_parameter("camera_yaw_offset").as_double();
  double update_rate_hz = get_parameter("update_rate_hz").as_double();

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).transient_local(),
    std::bind(&CoverageTrackerNode::onMapReceived, this, std::placeholders::_1));

  coverage_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/coverage_grid", rclcpp::QoS(1).transient_local());

  auto period = std::chrono::duration<double>(1.0 / update_rate_hz);
  update_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&CoverageTrackerNode::updateCoverage, this));

  RCLCPP_INFO(get_logger(), "CoverageTrackerNode configured.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CoverageTrackerNode::on_activate(const rclcpp_lifecycle::State &)
{
  coverage_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "CoverageTrackerNode activated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CoverageTrackerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  coverage_pub_->on_deactivate();
  RCLCPP_INFO(get_logger(), "CoverageTrackerNode deactivated.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CoverageTrackerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  tf_listener_.reset();
  tf_buffer_.reset();
  map_sub_.reset();
  coverage_pub_.reset();
  update_timer_.reset();
  coverage_data_.clear();
  latest_map_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CoverageTrackerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// ─── Callbacks ────────────────────────────────────────────────────────────────

void CoverageTrackerNode::onMapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!latest_map_ ||
    msg->info.width  != latest_map_->info.width ||
    msg->info.height != latest_map_->info.height)
  {
    // Map size changed — reset coverage grid
    std::size_t n = static_cast<std::size_t>(msg->info.width) *
      static_cast<std::size_t>(msg->info.height);
    coverage_data_.assign(n, 0);
    RCLCPP_INFO(get_logger(), "Coverage grid initialised (%u × %u).",
      msg->info.width, msg->info.height);
  }
  latest_map_ = msg;
}

void CoverageTrackerNode::updateCoverage()
{
  if (!latest_map_) {
    return;
  }

  // Look up current robot pose in map frame
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      map_frame_, base_frame_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  double robot_x = tf_stamped.transform.translation.x;
  double robot_y = tf_stamped.transform.translation.y;

  // Extract yaw from quaternion
  tf2::Quaternion q(
    tf_stamped.transform.rotation.x,
    tf_stamped.transform.rotation.y,
    tf_stamped.transform.rotation.z,
    tf_stamped.transform.rotation.w);
  double roll{}, pitch{}, yaw{};
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  markFrustum(robot_x, robot_y, yaw + camera_yaw_offset_);

  // Publish coverage grid
  nav_msgs::msg::OccupancyGrid cov_msg;
  cov_msg.header.stamp    = get_clock()->now();
  cov_msg.header.frame_id = map_frame_;
  cov_msg.info            = latest_map_->info;
  cov_msg.data            = coverage_data_;
  coverage_pub_->publish(cov_msg);
}

// ─── Frustum projection ───────────────────────────────────────────────────────

void CoverageTrackerNode::markFrustum(double robot_x, double robot_y, double yaw)
{
  const double res  = latest_map_->info.resolution;
  const double ox   = latest_map_->info.origin.position.x;
  const double oy   = latest_map_->info.origin.position.y;
  const int width   = static_cast<int>(latest_map_->info.width);
  const int height  = static_cast<int>(latest_map_->info.height);

  const int range_cells = static_cast<int>(camera_max_range_m_ / res);
  const double half_fov = camera_fov_rad_ / 2.0;

  // Robot in grid coords
  int rx = static_cast<int>((robot_x - ox) / res);
  int ry = static_cast<int>((robot_y - oy) / res);

  for (int dy = -range_cells; dy <= range_cells; ++dy) {
    for (int dx = -range_cells; dx <= range_cells; ++dx) {
      int nx = rx + dx;
      int ny = ry + dy;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }

      double dist = std::hypot(static_cast<double>(dx), static_cast<double>(dy)) * res;
      if (dist > camera_max_range_m_) {
        continue;
      }

      // Angle from robot heading to this cell
      double angle_to_cell = std::atan2(
        static_cast<double>(dy), static_cast<double>(dx));
      // Normalise to [-pi, pi] using std::remainder (single-pass, no loop)
      double angle_diff = std::remainder(angle_to_cell - yaw, 2.0 * M_PI);

      if (std::abs(angle_diff) <= half_fov) {
        coverage_data_[static_cast<std::size_t>(ny * width + nx)] = 100;
      }
    }
  }
}

}  // namespace hermes_navigate

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<hermes_navigate::CoverageTrackerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
