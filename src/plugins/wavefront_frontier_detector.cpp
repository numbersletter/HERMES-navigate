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

#include "hermes_navigate/plugins/wavefront_frontier_detector.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <utility>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace hermes_navigate
{

// ─── Initialization ───────────────────────────────────────────────────────────

void WavefrontFrontierDetector::initialize(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
  const std::string & name)
{
  auto node = parent.lock();
  if (!node) {
    return;
  }

  logger_ = node->get_logger();

  // Declare and read runtime parameters from plugin_params.yaml namespace.
  // The parameter name includes the plugin instance name as a prefix.
  const std::string prefix = name + ".";
  node->declare_parameter(prefix + "min_frontier_size",   rclcpp::ParameterValue(3));
  node->declare_parameter(prefix + "clustering_distance", rclcpp::ParameterValue(0.5));
  node->declare_parameter(prefix + "search_distance",     rclcpp::ParameterValue(10.0));

  min_frontier_size_   = node->get_parameter(prefix + "min_frontier_size").as_int();
  clustering_distance_ = node->get_parameter(prefix + "clustering_distance").as_double();
  search_distance_     = node->get_parameter(prefix + "search_distance").as_double();

  RCLCPP_INFO(logger_,
    "WavefrontFrontierDetector initialised [min_size=%d, cluster_dist=%.2f m, "
    "search_dist=%.2f m]",
    min_frontier_size_, clustering_distance_, search_distance_);
}

// ─── Search ───────────────────────────────────────────────────────────────────

std::vector<Frontier> WavefrontFrontierDetector::searchFrontiers(
  const nav2_msgs::msg::Costmap & costmap,
  const geometry_msgs::msg::PoseStamped & robot_pose)
{
  const double res = costmap.metadata.resolution;
  const double ox  = costmap.metadata.origin.position.x;
  const double oy  = costmap.metadata.origin.position.y;
  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);

  // Robot grid coordinates
  int robot_gx = static_cast<int>((robot_pose.pose.position.x - ox) / res);
  int robot_gy = static_cast<int>((robot_pose.pose.position.y - oy) / res);

  // Clamp to map bounds
  robot_gx = std::clamp(robot_gx, 0, width  - 1);
  robot_gy = std::clamp(robot_gy, 0, height - 1);

  // 1. BFS wavefront to find frontier cells reachable from the robot
  auto frontier_cells = detectFrontierCells(costmap, robot_gx, robot_gy);

  if (frontier_cells.empty()) {
    return {};
  }

  // 2. Cluster frontier cells
  auto clusters = clusterFrontiers(frontier_cells, costmap);

  // 3. Convert clusters to Frontier structs (filter small clusters)
  std::vector<Frontier> frontiers;
  frontiers.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    if (static_cast<int>(cluster.size()) < min_frontier_size_) {
      continue;
    }
    frontiers.push_back(clusterToFrontier(cluster, costmap));
  }

  RCLCPP_DEBUG(logger_, "WavefrontFrontierDetector: found %zu frontier clusters.",
    frontiers.size());

  return frontiers;
}

// ─── BFS wavefront frontier detection ────────────────────────────────────────

std::vector<std::pair<int, int>> WavefrontFrontierDetector::detectFrontierCells(
  const nav2_msgs::msg::Costmap & costmap,
  int robot_gx, int robot_gy)
{
  using namespace hermes_navigate::costmap;  // NOLINT(build/namespaces)

  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int height = static_cast<int>(costmap.metadata.size_y);
  const double res = costmap.metadata.resolution;
  const auto & data = costmap.data;

  // Max search radius in cells (0 = unlimited)
  const int max_radius_cells =
    (search_distance_ > 0.0) ? static_cast<int>(search_distance_ / res) : INT_MAX;

  auto idx      = [&](int x, int y) { return y * width + x; };
  auto inBounds = [&](int x, int y) {
      return x >= 0 && x < width && y >= 0 && y < height;
    };

  const int dx4[] = {-1, 0, 1, 0};
  const int dy4[] = {0, -1, 0, 1};

  // BFS from robot position through free cells (Wavefront Frontier Detector)
  std::vector<bool> visited(static_cast<std::size_t>(width * height), false);
  std::queue<std::pair<int, int>> q;

  if (data[static_cast<std::size_t>(idx(robot_gx, robot_gy))] != UNKNOWN &&
    data[static_cast<std::size_t>(idx(robot_gx, robot_gy))] < LETHAL)
  {
    q.push({robot_gx, robot_gy});
    visited[static_cast<std::size_t>(idx(robot_gx, robot_gy))] = true;
  }

  std::vector<std::pair<int, int>> frontier_cells;

  while (!q.empty()) {
    auto [cx, cy] = q.front();
    q.pop();

    // Check search radius
    int dx = cx - robot_gx;
    int dy = cy - robot_gy;
    if (dx * dx + dy * dy > max_radius_cells * max_radius_cells) {
      continue;
    }

    // A free cell is a frontier cell if it is adjacent to an unknown cell
    bool is_frontier = false;
    for (int d = 0; d < 4; ++d) {
      int nx = cx + dx4[d];
      int ny = cy + dy4[d];
      if (!inBounds(nx, ny)) {
        continue;
      }
      uint8_t cost = data[static_cast<std::size_t>(idx(nx, ny))];
      if (cost == UNKNOWN) {
        is_frontier = true;
      } else if (cost < LETHAL && !visited[static_cast<std::size_t>(idx(nx, ny))]) {
        visited[static_cast<std::size_t>(idx(nx, ny))] = true;
        q.push({nx, ny});
      }
    }

    if (is_frontier) {
      frontier_cells.emplace_back(cx, cy);
    }
  }

  return frontier_cells;
}

// ─── Clustering ───────────────────────────────────────────────────────────────

std::vector<std::vector<std::pair<int, int>>> WavefrontFrontierDetector::clusterFrontiers(
  const std::vector<std::pair<int, int>> & cells,
  const nav2_msgs::msg::Costmap & costmap)
{
  const double res = costmap.metadata.resolution;
  const int width  = static_cast<int>(costmap.metadata.size_x);
  const int cluster_radius =
    std::max(1, static_cast<int>(std::ceil(clustering_distance_ / res)));

  std::unordered_set<int> cell_set;
  for (const auto & [x, y] : cells) {
    cell_set.insert(y * width + x);
  }

  std::unordered_set<int> visited_set;
  std::vector<std::vector<std::pair<int, int>>> clusters;

  for (const auto & [sx, sy] : cells) {
    int key = sy * width + sx;
    if (visited_set.count(key)) {
      continue;
    }

    std::vector<std::pair<int, int>> cluster;
    std::queue<std::pair<int, int>> q;
    q.push({sx, sy});
    visited_set.insert(key);

    while (!q.empty()) {
      auto [cx, cy] = q.front();
      q.pop();
      cluster.emplace_back(cx, cy);

      for (int ddy = -cluster_radius; ddy <= cluster_radius; ++ddy) {
        for (int ddx = -cluster_radius; ddx <= cluster_radius; ++ddx) {
          int nx   = cx + ddx;
          int ny   = cy + ddy;
          int nkey = ny * width + nx;
          if (cell_set.count(nkey) && !visited_set.count(nkey)) {
            visited_set.insert(nkey);
            q.push({nx, ny});
          }
        }
      }
    }

    clusters.push_back(std::move(cluster));
  }

  return clusters;
}

// ─── Cluster → Frontier ───────────────────────────────────────────────────────

Frontier WavefrontFrontierDetector::clusterToFrontier(
  const std::vector<std::pair<int, int>> & cluster,
  const nav2_msgs::msg::Costmap & costmap)
{
  const double res = costmap.metadata.resolution;
  const double ox  = costmap.metadata.origin.position.x;
  const double oy  = costmap.metadata.origin.position.y;

  double sum_x = 0.0, sum_y = 0.0;
  for (const auto & [cx, cy] : cluster) {
    sum_x += cx;
    sum_y += cy;
  }
  double mean_cx = sum_x / static_cast<double>(cluster.size());
  double mean_cy = sum_y / static_cast<double>(cluster.size());

  Frontier f;
  f.centroid_x = ox + (mean_cx + 0.5) * res;
  f.centroid_y = oy + (mean_cy + 0.5) * res;
  f.size = static_cast<int>(cluster.size());

  f.goal_pose.header.frame_id = costmap.header.frame_id;
  f.goal_pose.header.stamp    = costmap.header.stamp;
  f.goal_pose.pose.position.x = f.centroid_x;
  f.goal_pose.pose.position.y = f.centroid_y;
  f.goal_pose.pose.position.z = 0.0;
  f.goal_pose.pose.orientation.w = 1.0;

  return f;
}

}  // namespace hermes_navigate

// ─── Plugin export ────────────────────────────────────────────────────────────
PLUGINLIB_EXPORT_CLASS(
  hermes_navigate::WavefrontFrontierDetector,
  hermes_navigate::BaseFrontierSearch)
