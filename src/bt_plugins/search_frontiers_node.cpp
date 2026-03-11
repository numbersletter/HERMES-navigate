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

#include "hermes_navigate/bt_plugins/search_frontiers_node.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace hermes_navigate
{

static const char * kDefaultPlugin = "hermes_navigate::WavefrontFrontierDetector";

// ─── Construction ─────────────────────────────────────────────────────────────

SearchFrontiersNode::SearchFrontiersNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::ActionNodeBase(name, config),
  parent_(parent),
  plugin_loader_("hermes_navigate", "hermes_navigate::BaseFrontierSearch")
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("SearchFrontiersNode: parent node expired.");
  }

  // ── Select and load the frontier search plugin ────────────────────────────
  node->declare_parameter("frontier_search.plugin",
    rclcpp::ParameterValue(std::string(kDefaultPlugin)));
  const std::string plugin_type =
    node->get_parameter("frontier_search.plugin").as_string();

  RCLCPP_INFO(node->get_logger(),
    "SearchFrontiersNode: loading plugin '%s'", plugin_type.c_str());

  frontier_plugin_ = plugin_loader_.createSharedInstance(plugin_type);
  frontier_plugin_->initialize(parent_, "wavefront_frontier_detector");

  // ── Costmap subscription ──────────────────────────────────────────────────
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw",
    rclcpp::QoS(1).transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      latest_costmap_ = msg;
    });

  // ── Visualisation publisher ───────────────────────────────────────────────
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/hermes/frontiers", rclcpp::QoS(1));
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList SearchFrontiersNode::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
    BT::OutputPort<std::vector<Frontier>>("frontiers"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus SearchFrontiersNode::tick()
{
  auto node = parent_.lock();

  if (!latest_costmap_) {
    if (node) {
      RCLCPP_WARN(node->get_logger(),
        "SearchFrontiersNode: waiting for global costmap.");
    }
    return BT::NodeStatus::RUNNING;
  }

  // Get robot pose (optional — plugin has fallback behaviour)
  geometry_msgs::msg::PoseStamped robot_pose;
  auto pose_result = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");
  if (pose_result) {
    robot_pose = pose_result.value();
  }

  std::vector<Frontier> frontiers =
    frontier_plugin_->searchFrontiers(*latest_costmap_, robot_pose);

  if (node) {
    RCLCPP_DEBUG(node->get_logger(),
      "SearchFrontiersNode: found %zu frontier(s).", frontiers.size());
  }

  setOutput("frontiers", frontiers);

  publishMarkers(frontiers);

  return BT::NodeStatus::SUCCESS;
}

// ─── Visualisation ────────────────────────────────────────────────────────────

void SearchFrontiersNode::publishMarkers(const std::vector<Frontier> & frontiers)
{
  visualization_msgs::msg::MarkerArray msg;

  // Delete-all marker first for clean update
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.push_back(del);

  int id = 0;
  for (const auto & f : frontiers) {
    visualization_msgs::msg::Marker m;
    m.header = f.goal_pose.header;
    m.ns     = "frontiers";
    m.id     = id++;
    m.type   = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose   = f.goal_pose.pose;
    m.scale.x = 0.3;
    m.scale.y = 0.3;
    m.scale.z = 0.3;
    m.color.r = 0.0f;
    m.color.g = 0.8f;
    m.color.b = 0.2f;
    m.color.a = 0.9f;
    msg.markers.push_back(m);
  }

  marker_pub_->publish(msg);
}

// ─── Factory registration ─────────────────────────────────────────────────────

void SearchFrontiersNode::registerWithFactory(
  BT::BehaviorTreeFactory & factory,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
{
  factory.registerBuilder<SearchFrontiersNode>(
    "SearchFrontiers",
    [parent](const std::string & name, const BT::NodeConfig & config) {
      return std::make_unique<SearchFrontiersNode>(name, config, parent);
    });
}

}  // namespace hermes_navigate
