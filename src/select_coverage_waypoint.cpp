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

#include "hermes_navigate/select_coverage_waypoint.hpp"

#include <cmath>
#include <utility>
#include <vector>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

SelectCoverageWaypoint::SelectCoverageWaypoint(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared("select_coverage_waypoint_node");
  standoff_distance_m_ = 1.5;  // 1.5 m from wall

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map",
    rclcpp::QoS(1).transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      latest_map_ = msg;
      waypoints_computed_ = false;  // recompute on map update
    });
}

BT::PortsList SelectCoverageWaypoint::providedPorts()
{
  return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal")};
}

// ─── Tick ─────────────────────────────────────────────────────────────────────

BT::NodeStatus SelectCoverageWaypoint::tick()
{
  rclcpp::spin_some(node_);

  if (!latest_map_) {
    RCLCPP_WARN(node_->get_logger(), "SelectCoverageWaypoint: waiting for /map.");
    return BT::NodeStatus::FAILURE;
  }

  if (!waypoints_computed_) {
    computeWaypoints(*latest_map_);
    waypoint_index_    = 0;
    waypoints_computed_ = true;
  }

  if (waypoint_index_ >= waypoints_.size()) {
    RCLCPP_INFO(node_->get_logger(), "SelectCoverageWaypoint: all waypoints visited.");
    return BT::NodeStatus::FAILURE;  // coverage complete
  }

  setOutput("goal", waypoints_[waypoint_index_]);
  ++waypoint_index_;
  return BT::NodeStatus::SUCCESS;
}

// ─── Waypoint computation ─────────────────────────────────────────────────────

void SelectCoverageWaypoint::computeWaypoints(const nav_msgs::msg::OccupancyGrid & map)
{
  waypoints_.clear();

  const int width  = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double res = map.info.resolution;
  const double ox  = map.info.origin.position.x;
  const double oy  = map.info.origin.position.y;

  const int standoff_cells = static_cast<int>(standoff_distance_m_ / res);

  // 8-connectivity directions for normal estimation
  const int dnx[] = {-1,  0,  1, -1, 1, -1,  0,  1};
  const int dny[] = {-1, -1, -1,  0, 0,  1,  1,  1};

  auto idx = [&](int x, int y) { return y * width + x; };
  auto inBounds = [&](int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
  };

  // Step through occupied cells (walls), sub-sample every 5 cells for
  // efficiency on a Pi 5.
  const int sample_stride = 5;

  for (int y = 0; y < height; y += sample_stride) {
    for (int x = 0; x < width; x += sample_stride) {
      if (map.data[static_cast<std::size_t>(idx(x, y))] < 50) {
        continue;  // not occupied
      }

      // Estimate inward normal by averaging directions of free neighbours
      double norm_x = 0.0, norm_y = 0.0;
      for (int d = 0; d < 8; ++d) {
        int nx = x + dnx[d];
        int ny = y + dny[d];
        if (inBounds(nx, ny) &&
          map.data[static_cast<std::size_t>(idx(nx, ny))] < 50)
        {
          norm_x += static_cast<double>(dnx[d]);
          norm_y += static_cast<double>(dny[d]);
        }
      }

      double norm_len = std::hypot(norm_x, norm_y);
      if (norm_len < 1e-6) {
        continue;  // surrounded by walls
      }
      norm_x /= norm_len;
      norm_y /= norm_len;

      // Place viewing pose standoff_cells away along the normal
      int vx = x + static_cast<int>(standoff_cells * norm_x);
      int vy = y + static_cast<int>(standoff_cells * norm_y);

      if (!inBounds(vx, vy)) {
        continue;
      }
      if (map.data[static_cast<std::size_t>(idx(vx, vy))] > 20) {
        continue;  // viewing pose is inside an obstacle
      }

      // Face back toward the wall (opposite of normal)
      double yaw = atan2Yaw(-norm_y, -norm_x);

      geometry_msgs::msg::PoseStamped wp;
      wp.header.frame_id = map.header.frame_id;
      wp.header.stamp    = map.header.stamp;
      wp.pose.position.x = ox + (vx + 0.5) * res;
      wp.pose.position.y = oy + (vy + 0.5) * res;
      wp.pose.position.z = 0.0;

      // Convert yaw to quaternion
      double half_yaw = yaw / 2.0;
      wp.pose.orientation.x = 0.0;
      wp.pose.orientation.y = 0.0;
      wp.pose.orientation.z = std::sin(half_yaw);
      wp.pose.orientation.w = std::cos(half_yaw);

      waypoints_.push_back(wp);
    }
  }

  RCLCPP_INFO(node_->get_logger(),
    "SelectCoverageWaypoint: computed %zu wall-facing waypoints.",
    waypoints_.size());
}

double SelectCoverageWaypoint::atan2Yaw(double dy, double dx)
{
  return std::atan2(dy, dx);
}

}  // namespace hermes_navigate

// ─── Plugin registration ──────────────────────────────────────────────────────
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<hermes_navigate::SelectCoverageWaypoint>("SelectCoverageWaypoint");
}
