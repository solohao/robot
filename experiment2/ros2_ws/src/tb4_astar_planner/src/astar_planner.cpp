// Copyright 2026 Rodriguez Sage
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

#include "tb4_astar_planner/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace tb4_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw std::runtime_error("Unable to lock planner lifecycle node");
  }

  name_ = std::move(name);
  tf_ = std::move(tf);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".use_diagonal", rclcpp::ParameterValue(true));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".cost_penalty", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".lethal_cost", rclcpp::ParameterValue(253));

  node_->get_parameter(name_ + ".allow_unknown", options_.allow_unknown);
  node_->get_parameter(name_ + ".use_diagonal", options_.use_diagonal);
  node_->get_parameter(name_ + ".cost_penalty", options_.cost_penalty);
  int lethal_cost = options_.lethal_cost;
  node_->get_parameter(name_ + ".lethal_cost", lethal_cost);
  options_.lethal_cost = static_cast<unsigned char>(
    std::clamp(lethal_cost, 1, 255));

  RCLCPP_INFO(
    node_->get_logger(),
    "Configured %s: diagonal=%s, allow_unknown=%s, cost_penalty=%.2f",
    name_.c_str(),
    options_.use_diagonal ? "true" : "false",
    options_.allow_unknown ? "true" : "false",
    options_.cost_penalty);
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up %s", name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating %s", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating %s", name_.c_str());
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_->now();
  path.header.frame_id = global_frame_;

  if (
    start.header.frame_id != global_frame_ ||
    goal.header.frame_id != global_frame_)
  {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Start and goal must use the %s frame",
      global_frame_.c_str());
    return path;
  }

  unsigned int start_x;
  unsigned int start_y;
  unsigned int goal_x;
  unsigned int goal_y;
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(
    *costmap_->getMutex());

  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y)) {
    RCLCPP_ERROR(node_->get_logger(), "Start pose is outside the global costmap");
    return path;
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
    RCLCPP_ERROR(node_->get_logger(), "Goal pose is outside the global costmap");
    return path;
  }

  const auto width = costmap_->getSizeInCellsX();
  const auto start_cell = start_y * width + start_x;
  const auto goal_cell = goal_y * width + goal_x;
  const auto result = search_.findPath(
    start_cell,
    goal_cell,
    costmap_->getCharMap(),
    width,
    costmap_->getSizeInCellsY(),
    options_);

  if (!result.success) {
    RCLCPP_WARN(node_->get_logger(), "No collision-free A* path was found");
    return path;
  }

  std::vector<std::pair<double, double>> points;
  points.reserve(result.cells.size());
  for (const auto cell : result.cells) {
    const auto map_x = cell % width;
    const auto map_y = cell / width;
    double world_x;
    double world_y;
    costmap_->mapToWorld(map_x, map_y, world_x, world_y);
    points.emplace_back(world_x, world_y);
  }
  lock.unlock();

  points.front() = {start.pose.position.x, start.pose.position.y};
  points.back() = {goal.pose.position.x, goal.pose.position.y};
  path.poses.reserve(points.size());

  for (std::size_t index = 0; index < points.size(); ++index) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = points[index].first;
    pose.pose.position.y = points[index].second;

    if (index + 1U == points.size()) {
      pose.pose.orientation = goal.pose.orientation;
    } else {
      const auto yaw = std::atan2(
        points[index + 1U].second - points[index].second,
        points[index + 1U].first - points[index].first);
      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, yaw);
      pose.pose.orientation = tf2::toMsg(orientation);
    }
    path.poses.push_back(std::move(pose));
  }

  RCLCPP_DEBUG(
    node_->get_logger(),
    "A* expanded %zu cells and returned %zu poses",
    result.expanded_nodes,
    path.poses.size());
  return path;
}

}  // namespace tb4_astar_planner

PLUGINLIB_EXPORT_CLASS(tb4_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
