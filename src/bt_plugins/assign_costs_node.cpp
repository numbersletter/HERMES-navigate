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

#include "hermes_navigate/bt_plugins/assign_costs_node.hpp"

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace hermes_navigate
{

// ─── Construction ─────────────────────────────────────────────────────────────

AssignCostsNode::AssignCostsNode(
  const std::string & name,
  const BT::NodeConfig & config,
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent)
: BT::SyncActionNode(name, config),
  parent_(parent),
  plugin_loader_("hermes_navigate", "hermes_navigate::BaseCostPlugin")
{
  auto node = parent_.lock();
  if (!node) {
    throw std::runtime_error("AssignCostsNode: parent node expired.");
  }

  // ── Select and load cost plugins ─────────────────────────────────────────
  const std::vector<std::string> default_plugins = {
    "hermes_navigate::InformationGainPlugin",
    "hermes_navigate::EuclideanPlugin",
  };
  node->declare_parameter("cost_plugins", rclcpp::ParameterValue(default_plugins));
  const auto plugin_types = node->get_parameter("cost_plugins").as_string_array();

  // Canonical parameter-namespace names for each plugin type
  // (matches keys in plugin_params.yaml)
  const std::vector<std::string> default_param_names = {
    "information_gain",
    "euclidean",
  };
  node->declare_parameter("cost_plugin_names",
    rclcpp::ParameterValue(default_param_names));
  const auto param_names = node->get_parameter("cost_plugin_names").as_string_array();

  for (std::size_t i = 0; i < plugin_types.size(); ++i) {
    const std::string & type = plugin_types[i];
    const std::string & pname = (i < param_names.size()) ? param_names[i] : type;

    RCLCPP_INFO(node->get_logger(),
      "AssignCostsNode: loading cost plugin '%s' (params namespace: '%s')",
      type.c_str(), pname.c_str());

    auto plugin = plugin_loader_.createSharedInstance(type);
    plugin->initialize(parent_, pname);
    cost_plugins_.push_back(plugin);
    plugin_names_.push_back(pname);
  }

  // ── Costmap subscription ──────────────────────────────────────────────────
  costmap_sub_ = node->create_subscription<nav2_msgs::msg::Costmap>(
    "/global_costmap/costmap_raw",
    rclcpp::QoS(1).transient_local(),
    [this](nav2_msgs::msg::Costmap::SharedPtr msg) {
      latest_costmap_ = msg;
    });

  // ── Visualisation publisher ───────────────────────────────────────────────
  marker_pub_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/hermes/scored_frontiers", rclcpp::QoS(1));
}

// ─── BT ports ─────────────────────────────────────────────────────────────────

BT::PortsList AssignCostsNode::providedPorts()
{
  return {
    BT::InputPort<std::vector<Frontier>>("frontiers"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>("robot_pose"),
    BT::OutputPort<std::vector<ScoredFrontier>>("scored_frontiers"),
  };
}

// ─── Tick ────────────────────────────────────────────────────────────────────

BT::NodeStatus AssignCostsNode::tick()
{
  auto frontiers_res = getInput<std::vector<Frontier>>("frontiers");
  if (!frontiers_res) {
    return BT::NodeStatus::FAILURE;
  }
  const auto & frontiers = frontiers_res.value();

  if (frontiers.empty()) {
    setOutput("scored_frontiers", std::vector<ScoredFrontier>{});
    return BT::NodeStatus::SUCCESS;
  }

  if (!latest_costmap_) {
    auto node = parent_.lock();
    if (node) {
      RCLCPP_WARN(node->get_logger(), "AssignCostsNode: waiting for costmap.");
    }
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped robot_pose;
  auto pose_res = getInput<geometry_msgs::msg::PoseStamped>("robot_pose");
  if (pose_res) {
    robot_pose = pose_res.value();
  }

  std::vector<ScoredFrontier> scored;
  scored.reserve(frontiers.size());

  for (const auto & f : frontiers) {
    ScoredFrontier sf;
    sf.frontier = f;
    sf.score    = 0.0;
    for (const auto & plugin : cost_plugins_) {
      sf.score += plugin->scoreFrontier(f, *latest_costmap_, robot_pose);
    }
    scored.push_back(sf);
  }

  setOutput("scored_frontiers", scored);

  publishMarkers(scored);

  return BT::NodeStatus::SUCCESS;
}

// ─── Visualisation ────────────────────────────────────────────────────────────

void AssignCostsNode::publishMarkers(const std::vector<ScoredFrontier> & scored)
{
  visualization_msgs::msg::MarkerArray msg;

  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  msg.markers.push_back(del);

  int id = 0;
  for (const auto & sf : scored) {
    visualization_msgs::msg::Marker m;
    m.header = sf.frontier.goal_pose.header;
    m.ns     = "scored_frontiers";
    m.id     = id++;
    m.type   = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose   = sf.frontier.goal_pose.pose;
    m.pose.position.z += 0.5;  // raise text above sphere
    m.scale.z = 0.25;
    m.color.r = 1.0f;
    m.color.g = 1.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0f;
    m.text    = std::to_string(static_cast<int>(sf.score));
    msg.markers.push_back(m);
  }

  marker_pub_->publish(msg);
}

}  // namespace hermes_navigate
