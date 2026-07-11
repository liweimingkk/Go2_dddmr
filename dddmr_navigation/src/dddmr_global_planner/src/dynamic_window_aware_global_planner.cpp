/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <global_planner/dynamic_window_aware_global_planner.h>
#include <global_planner/planner_safety.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

namespace global_planner
{

DWA_GlobalPlanner::DWA_GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse DWA_GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DWA_GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void DWA_GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&DWA_GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void DWA_GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d, 
                                    const std::shared_ptr<global_planner::GlobalPlanner>& global_planner){

  perception_3d_ros_ = perception_3d;
  global_planner_ = global_planner;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  has_current_goal_ = false;
  pcl_global_path_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  declare_parameter("look_ahead_distance", rclcpp::ParameterValue(5.0));
  this->get_parameter("look_ahead_distance", look_ahead_distance_);
  RCLCPP_INFO(this->get_logger(), "look_ahead_distance: %.2f", look_ahead_distance_);    
  declare_parameter("recompute_frequency", rclcpp::ParameterValue(10.0));
  this->get_parameter("recompute_frequency", recompute_frequency_);
  RCLCPP_INFO(this->get_logger(), "recompute_frequency: %.2f", recompute_frequency_);
  declare_parameter("maximum_splice_segment_length", rclcpp::ParameterValue(0.25));
  this->get_parameter("maximum_splice_segment_length", maximum_splice_segment_length_);
  RCLCPP_INFO(
    this->get_logger(), "maximum_splice_segment_length: %.2f",
    maximum_splice_segment_length_);
  declare_parameter("terrain_transition_mapping_tolerance", rclcpp::ParameterValue(0.001));
  this->get_parameter(
    "terrain_transition_mapping_tolerance", terrain_transition_mapping_tolerance_);
  RCLCPP_INFO(
    this->get_logger(), "terrain_transition_mapping_tolerance: %.4f",
    terrain_transition_mapping_tolerance_);
  if (!std::isfinite(look_ahead_distance_) || !std::isfinite(recompute_frequency_) ||
    !std::isfinite(maximum_splice_segment_length_) ||
    !std::isfinite(terrain_transition_mapping_tolerance_) || look_ahead_distance_ <= 0.0 ||
    recompute_frequency_ <= 0.0 ||
    maximum_splice_segment_length_ <= 0.0 ||
    terrain_transition_mapping_tolerance_ <= 0.0 ||
    terrain_transition_mapping_tolerance_ > 0.01)
  {
    throw std::invalid_argument(
            "DWA planner distances/frequency must be positive and terrain waypoint "
            "mapping tolerance must not exceed 0.01 m");
  }
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("awared_global_path", 1);

  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;

  auto loop_time = std::chrono::milliseconds(
    std::max(1, static_cast<int>(1000.0 / recompute_frequency_)));
  threading_timer_ = this->create_wall_timer(loop_time, std::bind(&DWA_GlobalPlanner::determineDWAPlan, this), action_server_group_);
  threading_timer_->cancel();
  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_dwa_plan",
    std::bind(&DWA_GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&DWA_GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&DWA_GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

DWA_GlobalPlanner::~DWA_GlobalPlanner(){
  action_server_global_planner_.reset();
}

void DWA_GlobalPlanner::failClosedDWAPlan(const std::string& reason)
{
  const std::unique_lock<std::recursive_mutex> state_lock(path_state_mutex_);
  global_dwa_path_ = nav_msgs::msg::Path();
  global_dwa_path_binding_ = planner_safety::PlanningDataBinding{};
  global_dwa_path_terrain_statistics_ = TerrainEdgeRejectionStatistics{};
  last_dwa_rejection_reason_ = reason;
  global_dwa_path_.header.frame_id = global_frame_;
  global_dwa_path_.header.stamp = clock_->now();
  RCLCPP_ERROR_THROTTLE(
    this->get_logger(), *clock_, 1000, "DWA replanning rejected: %s", reason.c_str());
  pub_path_->publish(global_dwa_path_);
}

bool DWA_GlobalPlanner::isNewGoal(){
  if (!has_current_goal_) {
    return true;
  }
  if(new_goal_.pose.position.x==current_goal_.pose.position.x && 
      new_goal_.pose.position.y==current_goal_.pose.position.y && 
        new_goal_.pose.position.z==current_goal_.pose.position.z && 
          new_goal_.pose.orientation.x==current_goal_.pose.orientation.x && 
            new_goal_.pose.orientation.y==current_goal_.pose.orientation.y && 
              new_goal_.pose.orientation.z==current_goal_.pose.orientation.z && 
                new_goal_.pose.orientation.w==current_goal_.pose.orientation.w)
     {
      
      return false;
     }
  
  RCLCPP_INFO(this->get_logger(), "Received new goal at: %.2f, %.2f, %.2f", new_goal_.pose.position.x, new_goal_.pose.position.y, new_goal_.pose.position.z);
  return true;
}

void DWA_GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  const auto planning_started = clock_->now();
  const auto make_result = [this, &planning_started](const BoundGlobalPlan & plan) {
      auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
      populateGetPlanResult(plan, *result);
      result->planning_time = static_cast<builtin_interfaces::msg::Duration>(
        clock_->now() - planning_started);
      return result;
    };
  const std::unique_lock<std::recursive_mutex> state_lock(path_state_mutex_);
  new_goal_ = goal_handle->get_goal()->goal;

  if(!goal_handle->get_goal()->activate_threading){
    threading_timer_->cancel();
    has_current_goal_ = false;
    global_path_.poses.clear();
    global_dwa_path_.poses.clear();
    global_path_binding_ = planner_safety::PlanningDataBinding{};
    global_dwa_path_binding_ = planner_safety::PlanningDataBinding{};
    global_path_terrain_statistics_ = TerrainEdgeRejectionStatistics{};
    global_dwa_path_terrain_statistics_ = TerrainEdgeRejectionStatistics{};
    last_dwa_rejection_reason_.clear();
    pcl_global_path_->clear();
    kdtree_global_path_.reset();
    BoundGlobalPlan inactive;
    inactive.status_code = PlanStatusCode::DEACTIVATED;
    inactive.status_reason = "DEACTIVATED";
    goal_handle->succeed(make_result(inactive));
    return;
  }
  
  if(isNewGoal()){
    
    geometry_msgs::msg::PoseStamped start;
    perception_3d_ros_->getGlobalPose(start);

    auto bound_plan = global_planner_->makeROSPlanWithBinding(
      start, goal_handle->get_goal()->goal);
    global_path_ = std::move(bound_plan.path);
    global_path_binding_ = std::move(bound_plan.binding);
    global_path_terrain_statistics_ = bound_plan.terrain_statistics;
    bound_plan.path = global_path_;
    bound_plan.binding = global_path_binding_;

    std::string binding_rejection;
    if(global_path_.poses.empty() ||
      !global_planner_->isPlanningBindingCurrent(
        global_path_binding_, &binding_rejection))
    {
      if (!global_path_.poses.empty()) {
        RCLCPP_WARN(
          this->get_logger(), "Initial DWA path binding is stale: %s",
          binding_rejection.c_str());
      }
      threading_timer_->cancel();
      has_current_goal_ = false;
      pcl_global_path_->clear();
      kdtree_global_path_.reset();
      global_path_.poses.clear();
      global_dwa_path_.poses.clear();
      global_path_binding_ = planner_safety::PlanningDataBinding{};
      global_dwa_path_binding_ = planner_safety::PlanningDataBinding{};
      global_path_terrain_statistics_ = TerrainEdgeRejectionStatistics{};
      global_dwa_path_terrain_statistics_ = TerrainEdgeRejectionStatistics{};
      bound_plan.path = nav_msgs::msg::Path();
      if (bound_plan.status_code == PlanStatusCode::SUCCESS) {
        bound_plan.status_code = PlanStatusCode::INPUT_CHANGED;
        bound_plan.status_reason = "INITIAL_PATH_BINDING_STALE:" + binding_rejection;
      }
      goal_handle->abort(make_result(bound_plan));
      return;
    }

    //@move global plan to pcl
    pcl_global_path_->points.clear();
    for(auto it=global_path_.poses.begin(); it!=global_path_.poses.end(); it++){
      pcl::PointXYZ pt;
      pt.x = (*it).pose.position.x; pt.y = (*it).pose.position.y; pt.z = (*it).pose.position.z;
      pcl_global_path_->push_back(pt);
    }
    //@ generate kd-tree
    kdtree_global_path_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    kdtree_global_path_->setInputCloud(pcl_global_path_);

    if (!global_planner_->isPlanningBindingCurrent(
        global_path_binding_, &binding_rejection))
    {
      threading_timer_->cancel();
      has_current_goal_ = false;
      global_path_.poses.clear();
      global_path_binding_ = planner_safety::PlanningDataBinding{};
      pcl_global_path_->clear();
      kdtree_global_path_.reset();
      bound_plan.path = nav_msgs::msg::Path();
      bound_plan.status_code = PlanStatusCode::INPUT_CHANGED;
      bound_plan.status_reason = "INITIAL_PATH_CHANGED_BEFORE_PUBLICATION:" +
        binding_rejection;
      failClosedDWAPlan("INITIAL_PATH_CHANGED_BEFORE_PUBLICATION");
      goal_handle->abort(make_result(bound_plan));
      return;
    }

    current_goal_ = new_goal_;
    has_current_goal_ = true;
    bound_plan.path = global_path_;
    bound_plan.binding = global_path_binding_;
    bound_plan.status_code = PlanStatusCode::SUCCESS;
    bound_plan.status_reason = "SUCCESS";
    pub_path_->publish(global_path_);
    goal_handle->succeed(make_result(bound_plan));
    global_dwa_path_.poses.clear();
    global_dwa_path_binding_ = planner_safety::PlanningDataBinding{};
    global_dwa_path_terrain_statistics_ = TerrainEdgeRejectionStatistics{};
    last_dwa_rejection_reason_.clear();
    threading_timer_->reset();
  }
  else{
    BoundGlobalPlan dwa_plan;
    dwa_plan.path = global_dwa_path_;
    dwa_plan.binding = global_dwa_path_binding_;
    dwa_plan.terrain_statistics = global_dwa_path_terrain_statistics_;
    dwa_plan.status_code = PlanStatusCode::SUCCESS;
    dwa_plan.status_reason = "SUCCESS";
    std::string binding_rejection;
    if (global_dwa_path_.poses.empty() ||
      !global_planner_->isPlanningBindingCurrent(
        global_dwa_path_binding_, &binding_rejection))
    {
      if (!global_dwa_path_.poses.empty()) {
        RCLCPP_WARN(
          this->get_logger(), "DWA sub-path binding is stale: %s",
          binding_rejection.c_str());
      }
      dwa_plan.path = nav_msgs::msg::Path();
      dwa_plan.status_code = PlanStatusCode::DWA_NOT_READY;
      dwa_plan.status_reason = last_dwa_rejection_reason_.empty() ?
        "DWA_PATH_NOT_READY" : last_dwa_rejection_reason_;
      RCLCPP_WARN(this->get_logger(), "No valid DWA sub-path is available");
      goal_handle->abort(make_result(dwa_plan));
    } else {
      goal_handle->succeed(make_result(dwa_plan));
      pub_path_->publish(global_dwa_path_);
    }
  }
  
}

void DWA_GlobalPlanner::determineDWAPlan(){
  const std::unique_lock<std::recursive_mutex> state_lock(path_state_mutex_);
  if (!pcl_global_path_ || pcl_global_path_->empty() || global_path_.poses.empty() ||
    pcl_global_path_->points.size() != global_path_.poses.size() || !kdtree_global_path_)
  {
    failClosedDWAPlan("GLOBAL_PATH_NOT_READY");
    return;
  }

  std::string binding_rejection;
  if (!global_planner_->isPlanningBindingCurrent(
      global_path_binding_, &binding_rejection))
  {
    failClosedDWAPlan("ORIGINAL_PATH_BINDING_STALE:" + binding_rejection);
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);
  if (!planner_safety::isFinitePoint(start.pose.position)) {
    failClosedDWAPlan("INVALID_START_POSE");
    return;
  }

  pcl::PointXYZ start_pt;
  start_pt.x = start.pose.position.x;
  start_pt.y = start.pose.position.y;
  start_pt.z = start.pose.position.z;
  std::vector<float> resultant_distances;
  std::vector<int> indices;
  const int nearest_count = kdtree_global_path_->nearestKSearch(
    start_pt, 1, indices, resultant_distances);
  if (nearest_count < 1 || indices.empty() || indices.front() < 0 ||
    static_cast<std::size_t>(indices.front()) >= pcl_global_path_->points.size())
  {
    failClosedDWAPlan("GLOBAL_PATH_NEAREST_SEARCH_FAILED");
    return;
  }

  std::unique_lock<std::recursive_mutex> lock(
    perception_3d_ros_->getSharedDataPtr()->ground_kdtree_cb_mutex_);
  const auto shared_data = perception_3d_ros_->getSharedDataPtr();
  if (!shared_data->kdtree_ground_ || !shared_data->pcl_ground_ ||
    shared_data->pcl_ground_->empty())
  {
    failClosedDWAPlan("GROUND_GRAPH_NOT_READY");
    return;
  }
  const auto observed_binding = capturePlanningDataBinding(
    shared_data, global_path_binding_.terrain_enabled);
  if (!planner_safety::planningBindingsMatch(
      global_path_binding_, observed_binding, &binding_rejection))
  {
    failClosedDWAPlan("ORIGINAL_PATH_CHANGED_DURING_PIVOT:" + binding_rejection);
    return;
  }

  const std::size_t nearest_index = static_cast<std::size_t>(indices.front());
  const std::size_t last_index = pcl_global_path_->points.size() - 1;
  const double inscribed_radius = perception_3d_ros_->getGlobalUtils()->getInscribedRadius();
  auto pivotIsClear = [&](std::size_t pivot) {
      std::vector<int> ground_indices;
      std::vector<float> ground_distances;
      pcl::PointXYZI query;
      query.x = pcl_global_path_->points[pivot].x;
      query.y = pcl_global_path_->points[pivot].y;
      query.z = pcl_global_path_->points[pivot].z;
      shared_data->kdtree_ground_->radiusSearch(
        query, 0.25, ground_indices, ground_distances);
      if (ground_indices.empty()) {
        return false;
      }
      for (const int ground_index : ground_indices) {
        if (ground_index < 0 ||
          static_cast<std::size_t>(ground_index) >= shared_data->pcl_ground_->points.size())
        {
          return false;
        }
        const double clearance = perception_3d_ros_->get_min_dGraphValue(ground_index);
        if (!std::isfinite(clearance) || clearance < inscribed_radius) {
          return false;
        }
      }
      return true;
    };

  bool found_clear_pivot = false;
  std::size_t dwa_pivot = nearest_index;
  for (double requested_distance = look_ahead_distance_; requested_distance <= 100.0;
    requested_distance += 1.0)
  {
    double accumulated_distance = 0.0;
    std::size_t pivot = nearest_index;
    while (pivot < last_index && accumulated_distance < requested_distance) {
      const auto & current = pcl_global_path_->points[pivot];
      const auto & next = pcl_global_path_->points[pivot + 1];
      accumulated_distance += std::sqrt(
        std::pow(static_cast<double>(next.x) - current.x, 2.0) +
        std::pow(static_cast<double>(next.y) - current.y, 2.0) +
        std::pow(static_cast<double>(next.z) - current.z, 2.0));
      ++pivot;
    }

    if (pivotIsClear(pivot)) {
      dwa_pivot = pivot;
      found_clear_pivot = true;
      break;
    }
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "DWA pivot is unsupported or blocked at: %.2f, %.2f, %.2f",
      pcl_global_path_->points[pivot].x, pcl_global_path_->points[pivot].y,
      pcl_global_path_->points[pivot].z);
    if (pivot == last_index) {
      break;
    }
  }

  if (!found_clear_pivot) {
    failClosedDWAPlan("NO_CLEAR_SUPPORTED_PIVOT");
    return;
  }

  // makeROSPlan acquires the planner mutex before the same perception mutex inside A*.
  // Release here to preserve that lock order and avoid a cross-node deadlock.
  lock.unlock();

  geometry_msgs::msg::PoseStamped dwa_goal = global_path_.poses[dwa_pivot];
  dwa_goal.header.frame_id = global_frame_;
  RCLCPP_DEBUG(
    this->get_logger(),
    "DWA pivot: %zu is at %.2f, %.2f, %.2f with start: %.2f, %.2f, %.2f",
    dwa_pivot, dwa_goal.pose.position.x, dwa_goal.pose.position.y, dwa_goal.pose.position.z,
    start.pose.position.x, start.pose.position.y, start.pose.position.z);

  auto replanned_prefix = global_planner_->makeROSPlanWithBinding(start, dwa_goal);
  if (replanned_prefix.path.poses.empty()) {
    failClosedDWAPlan("EMPTY_REPLANNED_PREFIX");
    return;
  }
  if (!planner_safety::planningBindingsMatch(
      global_path_binding_, replanned_prefix.binding, &binding_rejection))
  {
    failClosedDWAPlan("PREFIX_TAIL_BINDING_MISMATCH:" + binding_rejection);
    return;
  }

  nav_msgs::msg::Path spliced_path;
  std::string splice_rejection;
  bool splice_succeeded = false;
  if (global_path_binding_.terrain_enabled) {
    // Reacquire the perception side only after makeROSPlan has released the
    // planner mutex. The current binding proves that this cloud and immutable
    // snapshot are the same generation used by both prefix and tail.
    lock.lock();
    const auto splice_binding = capturePlanningDataBinding(shared_data, true);
    if (!planner_safety::planningBindingsMatch(
        global_path_binding_, splice_binding, &binding_rejection))
    {
      lock.unlock();
      failClosedDWAPlan("SPLICE_TERRAIN_BINDING_STALE:" + binding_rejection);
      return;
    }
    const auto terrain_snapshot = shared_data->getTerrainSnapshot();
    if (!shared_data->pcl_ground_) {
      lock.unlock();
      failClosedDWAPlan("SPLICE_TERRAIN_GROUND_MISSING");
      return;
    }
    splice_succeeded = planner_safety::spliceTerrainPathsFailClosed(
      replanned_prefix.path, global_path_, dwa_pivot, maximum_splice_segment_length_,
      *shared_data->pcl_ground_, terrain_snapshot, splice_binding,
      terrain_transition_mapping_tolerance_, spliced_path, &splice_rejection);
    lock.unlock();
  } else {
    // Preserve the legacy strict segment threshold when terrain validation is
    // disabled; no snapshot-based exception is available in this mode.
    splice_succeeded = planner_safety::splicePathsFailClosed(
      replanned_prefix.path, global_path_, dwa_pivot, maximum_splice_segment_length_,
      spliced_path, &splice_rejection);
  }
  if (!splice_succeeded)
  {
    failClosedDWAPlan(splice_rejection);
    return;
  }
  if (!global_planner_->isPlanningBindingCurrent(
      replanned_prefix.binding, &binding_rejection))
  {
    failClosedDWAPlan("SPLICED_PATH_BINDING_STALE:" + binding_rejection);
    return;
  }
  global_dwa_path_ = std::move(spliced_path);
  global_dwa_path_binding_ = std::move(replanned_prefix.binding);
  global_dwa_path_terrain_statistics_ = replanned_prefix.terrain_statistics;
  last_dwa_rejection_reason_.clear();
}

}
