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

#ifndef TB4_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
#define TB4_ASTAR_PLANNER__ASTAR_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tb4_astar_planner/astar_search.hpp"
#include "tf2_ros/buffer.h"

namespace tb4_astar_planner
{

class AStarPlanner : public nav2_core::GlobalPlanner
{
public:
  AStarPlanner() = default;
  ~AStarPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string name_;
  std::string global_frame_;
  SearchOptions options_;
  AStarSearch search_;
};

}  // namespace tb4_astar_planner

#endif  // TB4_ASTAR_PLANNER__ASTAR_PLANNER_HPP_
