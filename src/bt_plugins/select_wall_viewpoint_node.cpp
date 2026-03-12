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

#include "hermes_navigate/bt_plugins/select_wall_viewpoint_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "hermes_navigate/costmap_constants.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

SelectWallViewpointNode::SelectWallViewpointNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::ActionNodeBase(name, config),
  parent_(parent)
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("SelectWallViewpointNode: parent node expired.");
  }

  // Declare and read parameters
  node->declare_parameter("wall_inspection.standoff_m",         rclcpp::ParameterValue(0.6));
  node->declare_parameter("wall_inspection.cluster_distance_m", rclcpp::ParameterValue(1.0));
  node->declare_parameter("wall_inspection.min_segment_cells",  rclcpp::ParameterValue(3));

  standoff_m_         = node->get_parameter("wall_inspection.standoff_m").as_double();
  cluster_distance_m_ = node->get_parameter("wall_inspection.cluster_distance_m").as_double();
  min_segment_cells_  = node->get_parameter("wall_inspection.min_segment_cells").as_int();

  // Subscribe to the global costmap (same topic as SearchFrontiersNode)
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw",
    rclcpp::QoS(1).transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      latest_costmap_ = msg;
    });

  // Visualisation publisher
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/hermes/wall_viewpoints", rclcpp::QoS(1));

  RCLCPP_INFO(node->get_logger(),
    "SelectWallViewpointNode: standoff=%.2fm, cluster_dist=%.2fm, min_cells=%d",
    standoff_m_, cluster_distance_m_, min_segment_cells_);
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList SelectWallViewpointNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("nav_goal"),
    BT::OutputPort<bool>("inspection_done"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus SelectWallViewpointNode::tick()
{
  auto node = parent_.lock();

  // ── Wait for costmap ────────────────────────────────────────────────────
  if (!latest_costmap_) {
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "SelectWallViewpointNode: waiting for global costmap.");
    }
    setOutput("inspection_done", false);
    return BT::NodeStatus::RUNNING;
  }

  // ── Compute viewpoints once (after exploration has mapped the environment) ─
  if (!viewpoints_computed_) {
    geometry_msgs::msg::PoseStamped robot_pose;
    auto pose_res = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");
    if (pose_res) {
      robot_pose = pose_res.value();
    }

    computeViewpoints(*latest_costmap_, robot_pose);
    viewpoints_computed_ = true;

    if (node) {
      RCLCPP_INFO(node->get_logger(),
        "SelectWallViewpointNode: computed %zu wall viewpoint(s) for inspection.",
        viewpoints_.size());
    }

    publishMarkers();
  }

  // ── Check if all viewpoints have been visited ───────────────────────────
  if (current_idx_ >= viewpoints_.size()) {
    if (node) {
      RCLCPP_INFO(node->get_logger(),
        "SelectWallViewpointNode: all %zu wall viewpoint(s) visited — inspection done.",
        viewpoints_.size());
    }
    setOutput("inspection_done", true);
    return BT::NodeStatus::FAILURE;  // signals InspectionPipeline completion
  }

  // ── Select the next viewpoint ───────────────────────────────────────────
  const auto & vp = viewpoints_[current_idx_];

  if (node) {
    RCLCPP_INFO(node->get_logger(),
      "SelectWallViewpointNode: viewpoint %zu/%zu → (%.2f, %.2f) [%.1f°].",
      current_idx_ + 1, viewpoints_.size(),
      vp.pose.position.x, vp.pose.position.y,
      std::atan2(
        2.0 * (vp.pose.orientation.w * vp.pose.orientation.z),
        1.0 - 2.0 * (vp.pose.orientation.z * vp.pose.orientation.z)) * 180.0 / M_PI);
  }

  setOutput("nav_goal", vp);
  setOutput("inspection_done", false);

  // Advance the index NOW so that on the next leg (after successful navigation
  // and Sequence restart) we pick the following viewpoint.  This node is
  // ticked exactly once per leg because the enclosing Sequence has memory and
  // skips child[0] while child[1] (NavigateToPose) is RUNNING.
  ++current_idx_;

  return BT::NodeStatus::SUCCESS;
}

// ─── computeViewpoints ────────────────────────────────────────────────────────

void SelectWallViewpointNode::computeViewpoints(
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  using namespace hermes_navigate::costmap;  // NOLINT(build/namespaces)

  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);
  const double res = costmap.metadata.resolution;
  const double ox  = costmap.metadata.origin.position.x;
  const double oy  = costmap.metadata.origin.position.y;
  const auto & data = costmap.data;

  if (width <= 0 || height <= 0 || res <= 0.0) {
    return;
  }

  auto idx      = [&](int x, int y) { return y * width + x; };
  auto inBounds = [&](int x, int y) {
      return x >= 0 && x < width && y >= 0 && y < height;
    };

  // 4-connectivity offsets (N W S E)
  const int dx4[] = {-1, 0, 1, 0};
  const int dy4[] = {0, -1, 0, 1};

  // ── Step 1: collect candidate viewpoints from wall-boundary cells ─────
  // A wall-boundary cell is a lethal (≥ LETHAL) cell with at least one free
  // neighbour.  For each such cell we compute the outward normal (pointing
  // away from the wall into free space) and place a candidate viewpoint at
  // wall_centre + normal × standoff_m.

  struct Candidate
  {
    double x, y, yaw;
  };
  std::vector<Candidate> candidates;
  candidates.reserve(1024);

  for (int cy = 0; cy < height; ++cy) {
    for (int cx = 0; cx < width; ++cx) {
      uint8_t cost = data[static_cast<std::size_t>(idx(cx, cy))];

      // Only consider lethal (wall/obstacle) cells
      if (cost < LETHAL) {
        continue;
      }

      // Accumulate free-neighbour directions to get the outward normal
      double normal_x = 0.0, normal_y = 0.0;
      int free_count = 0;

      for (int d = 0; d < 4; ++d) {
        int nx = cx + dx4[d];
        int ny = cy + dy4[d];
        if (!inBounds(nx, ny)) {
          continue;
        }
        uint8_t ncost = data[static_cast<std::size_t>(idx(nx, ny))];
        // A neighbour is "free" if it is navigable (below INSCRIBED) and known
        if (ncost != UNKNOWN && ncost < INSCRIBED) {
          normal_x += static_cast<double>(nx - cx);
          normal_y += static_cast<double>(ny - cy);
          ++free_count;
        }
      }

      if (free_count == 0) {
        continue;  // interior obstacle cell — not a wall face
      }

      // Normalise the outward normal
      const double normal_len = std::hypot(normal_x, normal_y);
      if (normal_len < 1e-6) {
        continue;
      }
      normal_x /= normal_len;
      normal_y /= normal_len;

      // World coordinates of the wall cell centre
      const double wall_wx = ox + (static_cast<double>(cx) + 0.5) * res;
      const double wall_wy = oy + (static_cast<double>(cy) + 0.5) * res;

      // Candidate viewpoint: step standoff_m into free space along the normal
      const double vp_x = wall_wx + normal_x * standoff_m_;
      const double vp_y = wall_wy + normal_y * standoff_m_;

      // Validate: the viewpoint must land in a known, navigable cell
      const int vp_gx = static_cast<int>((vp_x - ox) / res);
      const int vp_gy = static_cast<int>((vp_y - oy) / res);
      if (!inBounds(vp_gx, vp_gy)) {
        continue;
      }
      uint8_t vp_cost = data[static_cast<std::size_t>(idx(vp_gx, vp_gy))];
      if (vp_cost == UNKNOWN || vp_cost >= INSCRIBED) {
        continue;
      }

      // Orientation: face the wall (opposite of outward normal)
      const double yaw = std::atan2(-normal_y, -normal_x);

      candidates.push_back({vp_x, vp_y, yaw});
    }
  }

  if (candidates.empty()) {
    auto node = parent_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "SelectWallViewpointNode: no wall-boundary candidates found "
        "— check costmap and standoff_m parameter.");
    }
    return;
  }

  // ── Step 2: spatial grid clustering ──────────────────────────────────────
  // Group candidates into grid buckets of size cluster_distance_m.  Each
  // bucket that contains at least min_segment_cells_ candidates becomes one
  // viewpoint at the centroid of the bucket.  This is O(n) and naturally
  // merges adjacent wall-pixel candidates that would produce identical goals.

  // Use std::map so pair<int,int> works without a custom hash
  using GridKey = std::pair<int, int>;
  std::map<GridKey, std::vector<std::size_t>> grid;

  const double bucket_size = std::max(cluster_distance_m_, res);

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const int gx = static_cast<int>(std::floor(candidates[i].x / bucket_size));
    const int gy = static_cast<int>(std::floor(candidates[i].y / bucket_size));
    grid[{gx, gy}].push_back(i);
  }

  std::vector<geometry_msgs::msg::PoseStamped> viewpoints;
  viewpoints.reserve(grid.size());

  for (const auto & [key, indices] : grid) {
    if (static_cast<int>(indices.size()) < min_segment_cells_) {
      continue;  // too few candidates — likely noise or a tiny obstacle
    }

    // Centroid in world coordinates
    double sum_x = 0.0, sum_y = 0.0;
    double sum_cos = 0.0, sum_sin = 0.0;
    for (std::size_t i : indices) {
      sum_x   += candidates[i].x;
      sum_y   += candidates[i].y;
      sum_cos += std::cos(candidates[i].yaw);
      sum_sin += std::sin(candidates[i].yaw);
    }
    const double n    = static_cast<double>(indices.size());
    const double cx   = sum_x / n;
    const double cy   = sum_y / n;
    const double yaw  = std::atan2(sum_sin, sum_cos);

    // Validate centroid is also in navigable space
    const int cgx = static_cast<int>((cx - ox) / res);
    const int cgy = static_cast<int>((cy - oy) / res);
    if (!inBounds(cgx, cgy)) {
      continue;
    }
    const uint8_t ccost = data[static_cast<std::size_t>(idx(cgx, cgy))];
    if (ccost == UNKNOWN || ccost >= INSCRIBED) {
      continue;
    }

    // Build PoseStamped with orientation facing the wall
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = costmap.header.frame_id;
    pose.header.stamp    = costmap.header.stamp;
    pose.pose.position.x = cx;
    pose.pose.position.y = cy;
    pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation = tf2::toMsg(q);

    viewpoints.push_back(pose);
  }

  if (viewpoints.empty()) {
    auto node = parent_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "SelectWallViewpointNode: %zu candidate(s) found but all clusters "
        "were too small (min_segment_cells=%d) or had invalid centroids.",
        candidates.size(), min_segment_cells_);
    }
    return;
  }

  // ── Step 3: nearest-neighbour tour ordering ───────────────────────────────
  // Greedily order viewpoints to minimise total travel distance, starting from
  // the robot's current position.

  double cur_x = robot_pose.pose.position.x;
  double cur_y = robot_pose.pose.position.y;

  std::vector<bool> visited(viewpoints.size(), false);
  std::vector<geometry_msgs::msg::PoseStamped> ordered;
  ordered.reserve(viewpoints.size());

  while (ordered.size() < viewpoints.size()) {
    std::size_t nearest      = 0;
    double      nearest_dist = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < viewpoints.size(); ++i) {
      if (visited[i]) {
        continue;
      }
      const double dx   = viewpoints[i].pose.position.x - cur_x;
      const double dy   = viewpoints[i].pose.position.y - cur_y;
      const double dist = dx * dx + dy * dy;
      if (dist < nearest_dist) {
        nearest_dist = dist;
        nearest      = i;
      }
    }

    visited[nearest] = true;
    ordered.push_back(viewpoints[nearest]);
    cur_x = viewpoints[nearest].pose.position.x;
    cur_y = viewpoints[nearest].pose.position.y;
  }

  viewpoints_ = std::move(ordered);
}

// ─── Visualisation ────────────────────────────────────────────────────────────

void SelectWallViewpointNode::publishMarkers()
{
  visualization_msgs::msg::MarkerArray msg;

  // Delete-all marker first for clean update
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.push_back(del);

  int id = 0;
  for (const auto & vp : viewpoints_) {
    // Sphere at the viewpoint position
    visualization_msgs::msg::Marker sphere;
    sphere.header = vp.header;
    sphere.ns     = "wall_viewpoints";
    sphere.id     = id++;
    sphere.type   = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose   = vp.pose;
    sphere.scale.x = 0.2;
    sphere.scale.y = 0.2;
    sphere.scale.z = 0.2;
    sphere.color.r = 1.0f;
    sphere.color.g = 0.5f;
    sphere.color.b = 0.0f;
    sphere.color.a = 0.9f;
    msg.markers.push_back(sphere);

    // Arrow showing the heading (pointing toward the wall)
    visualization_msgs::msg::Marker arrow;
    arrow.header = vp.header;
    arrow.ns     = "wall_viewpoint_headings";
    arrow.id     = id++;
    arrow.type   = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.pose   = vp.pose;
    arrow.scale.x = 0.4;   // shaft length
    arrow.scale.y = 0.06;  // shaft diameter
    arrow.scale.z = 0.06;
    arrow.color.r = 1.0f;
    arrow.color.g = 0.2f;
    arrow.color.b = 0.0f;
    arrow.color.a = 0.9f;
    msg.markers.push_back(arrow);
  }

  marker_pub_->publish(msg);
}

// ─── Factory registration ─────────────────────────────────────────────────────

void SelectWallViewpointNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  factory.registerBuilder<SelectWallViewpointNode>(
    "SelectWallViewpoint",
    [parent](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SelectWallViewpointNode>(name, config, parent);
    });
}

}  // namespace hermes_navigate
