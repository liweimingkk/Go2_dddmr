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
#include <global_planner/global_planner.h>
#include <global_planner/planner_safety.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>
#include <utility>

using namespace std::chrono_literals;

namespace global_planner
{

void populateGetPlanResult(
  const BoundGlobalPlan & plan,
  dddmr_sys_core::action::GetPlan::Result & result)
{
  result.path = plan.path;
  result.status_code = static_cast<std::uint16_t>(plan.status_code);
  result.status_reason = plan.status_reason;
  result.terrain_enabled = plan.binding.terrain_enabled;
  result.static_ground_generation = plan.binding.static_ground_generation;
  result.map_hash = plan.binding.map_hash;
  result.terrain_snapshot_version = plan.binding.terrain_snapshot_version;
  result.evaluated_edge_count = plan.terrain_statistics.evaluated;
  result.accepted_edge_count = plan.terrain_statistics.accepted;
  result.rejected_edge_count = plan.terrain_statistics.rejected();
  result.rejection_statistics = plan.terrain_statistics.toStructuredString();
}

GlobalPlanner::GlobalPlanner(const std::string& name)
    : Node(name) 
{
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse GlobalPlanner::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::GetPlan::Goal> goal)
{
  (void)uuid;
  (void)goal;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GlobalPlanner::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GlobalPlanner::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle)
{
  rclcpp::Rate r(20);
  while (is_active(current_handle_)) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Wait for current handle to join");
    r.sleep();
  }
  current_handle_.reset();
  current_handle_ = goal_handle;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&GlobalPlanner::makePlan, this, std::placeholders::_1), goal_handle}.detach();
}
  
void GlobalPlanner::initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d){
  
  static_ground_generation_ = 0U;
  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>);
  kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>);
  
  declare_parameter("turning_weight", rclcpp::ParameterValue(0.1));
  this->get_parameter("turning_weight", turning_weight_);
  RCLCPP_INFO(this->get_logger(), "turning_weight: %.2f", turning_weight_);    

  declare_parameter("enable_detail_log", rclcpp::ParameterValue(false));
  this->get_parameter("enable_detail_log", enable_detail_log_);
  RCLCPP_INFO(this->get_logger(), "enable_detail_log: %d", enable_detail_log_);    

  declare_parameter("a_star_expanding_radius", rclcpp::ParameterValue(0.5));
  this->get_parameter("a_star_expanding_radius", a_star_expanding_radius_);
  RCLCPP_INFO(this->get_logger(), "a_star_expanding_radius: %.2f", a_star_expanding_radius_);    

  declare_parameter("use_pre_graph", rclcpp::ParameterValue(false));
  this->get_parameter("use_pre_graph", use_pre_graph_);
  RCLCPP_INFO(this->get_logger(), "use_pre_graph: %d", use_pre_graph_);    

  declare_parameter("find_start_tolerance", rclcpp::ParameterValue(0.5));
  this->get_parameter("find_start_tolerance", find_start_tolerance_);
  RCLCPP_INFO(this->get_logger(), "find_start_tolerance: %.2f", find_start_tolerance_);    

  declare_parameter("find_goal_tolerance", rclcpp::ParameterValue(0.5));
  this->get_parameter("find_goal_tolerance", find_goal_tolerance_);
  RCLCPP_INFO(this->get_logger(), "find_goal_tolerance: %.2f", find_goal_tolerance_);

  declare_parameter("max_vertical_search_distance", rclcpp::ParameterValue(0.5));
  this->get_parameter("max_vertical_search_distance", max_vertical_search_distance_);
  RCLCPP_INFO(
    this->get_logger(), "max_vertical_search_distance: %.2f", max_vertical_search_distance_);

  declare_parameter("ground_layer_separation", rclcpp::ParameterValue(0.35));
  this->get_parameter("ground_layer_separation", ground_layer_separation_);
  RCLCPP_INFO(this->get_logger(), "ground_layer_separation: %.2f", ground_layer_separation_);

  declare_parameter("layer_ambiguity_xy_distance", rclcpp::ParameterValue(0.25));
  this->get_parameter("layer_ambiguity_xy_distance", layer_ambiguity_xy_distance_);
  RCLCPP_INFO(
    this->get_logger(), "layer_ambiguity_xy_distance: %.2f", layer_ambiguity_xy_distance_);

  declare_parameter("robot_ground_z_offset", rclcpp::ParameterValue(0.24));
  this->get_parameter("robot_ground_z_offset", robot_ground_z_offset_);
  RCLCPP_INFO(this->get_logger(), "robot_ground_z_offset: %.2f", robot_ground_z_offset_);

  declare_parameter("raw_goal_support_radius", rclcpp::ParameterValue(0.4));
  this->get_parameter("raw_goal_support_radius", raw_goal_support_radius_);
  RCLCPP_INFO(this->get_logger(), "raw_goal_support_radius: %.2f", raw_goal_support_radius_);

  declare_parameter("raw_goal_support_z_tolerance", rclcpp::ParameterValue(0.2));
  this->get_parameter("raw_goal_support_z_tolerance", raw_goal_support_z_tolerance_);
  RCLCPP_INFO(
    this->get_logger(), "raw_goal_support_z_tolerance: %.2f",
    raw_goal_support_z_tolerance_);

  declare_parameter("raw_goal_minimum_support_points", rclcpp::ParameterValue(3));
  this->get_parameter("raw_goal_minimum_support_points", raw_goal_minimum_support_points_);
  RCLCPP_INFO(
    this->get_logger(), "raw_goal_minimum_support_points: %d",
    raw_goal_minimum_support_points_);

  declare_parameter("raw_goal_minimum_support_sectors", rclcpp::ParameterValue(3));
  this->get_parameter("raw_goal_minimum_support_sectors", raw_goal_minimum_support_sectors_);
  RCLCPP_INFO(
    this->get_logger(), "raw_goal_minimum_support_sectors: %d",
    raw_goal_minimum_support_sectors_);

  declare_parameter("path_interpolation_resolution", rclcpp::ParameterValue(0.1));
  this->get_parameter("path_interpolation_resolution", path_interpolation_resolution_);
  RCLCPP_INFO(
    this->get_logger(), "path_interpolation_resolution: %.2f", path_interpolation_resolution_);

  declareTerrainParameters();

  if (!std::isfinite(find_start_tolerance_) || !std::isfinite(find_goal_tolerance_) ||
    !std::isfinite(max_vertical_search_distance_) ||
    !std::isfinite(ground_layer_separation_) ||
    !std::isfinite(layer_ambiguity_xy_distance_) ||
    !std::isfinite(robot_ground_z_offset_) ||
    !std::isfinite(raw_goal_support_radius_) ||
    !std::isfinite(raw_goal_support_z_tolerance_) ||
    !std::isfinite(path_interpolation_resolution_) ||
    !std::isfinite(a_star_expanding_radius_) ||
    !std::isfinite(turning_weight_) ||
    find_start_tolerance_ <= 0.0 || find_goal_tolerance_ <= 0.0 ||
    max_vertical_search_distance_ < 0.0 || ground_layer_separation_ <= 0.0 ||
    layer_ambiguity_xy_distance_ < 0.0 || robot_ground_z_offset_ < 0.0 ||
    raw_goal_support_radius_ <= 0.0 ||
    raw_goal_support_z_tolerance_ < 0.0 || raw_goal_minimum_support_points_ <= 0 ||
    raw_goal_minimum_support_sectors_ <= 0 || raw_goal_minimum_support_sectors_ > 4 ||
    path_interpolation_resolution_ <= 0.0 || a_star_expanding_radius_ <= 0.0 ||
    turning_weight_ < 0.0)
  {
    throw std::invalid_argument("Global planner fail-closed tolerances must be positive");
  }

  

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = action_server_group_;
  
  perception_3d_check_timer_ = this->create_wall_timer(500ms, std::bind(&GlobalPlanner::checkPerception3DThread, this), action_server_group_);
  
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "clicked_point", 1, 
      std::bind(&GlobalPlanner::cbClickedPoint, this, std::placeholders::_1), sub_options);
  
  pub_path_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 1);
  pub_static_graph_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("static_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_weighted_pc_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("weighted_ground", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  //@Create action server
  this->action_server_global_planner_ = rclcpp_action::create_server<dddmr_sys_core::action::GetPlan>(
    this,
    "/get_plan",
    std::bind(&GlobalPlanner::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GlobalPlanner::handle_cancel, this, std::placeholders::_1),
    std::bind(&GlobalPlanner::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
  
}

GlobalPlanner::~GlobalPlanner(){

  //perception_3d_ros_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  a_star_planner_.reset();
  a_star_planner_pre_graph_.reset();
  action_server_global_planner_.reset();
  kdtree_ground_.reset();
  kdtree_map_.reset();
  pcl_ground_.reset();
  pcl_map_.reset();
}

planner_safety::PlanningDataBinding
GlobalPlanner::captureCurrentPlanningBindingLocked() const
{
  planner_safety::PlanningDataBinding binding;
  binding.terrain_enabled = terrain_edge_config_.policy.enabled;
  if (!perception_3d_ros_ || !perception_3d_ros_->getSharedDataPtr() ||
    !graph_ready_ || !has_initialized_ || !pcl_ground_ || pcl_ground_->empty() ||
    !kdtree_ground_ || static_ground_generation_ == 0U)
  {
    return binding;
  }

  const auto shared_data = perception_3d_ros_->getSharedDataPtr();
  binding = capturePlanningDataBinding(shared_data, terrain_edge_config_.policy.enabled);
  if (!binding.valid ||
    binding.static_ground_generation != static_ground_generation_)
  {
    binding.valid = false;
    return binding;
  }
  if (binding.terrain_enabled &&
    (binding.map_hash != terrain_edge_config_.expected_map_hash ||
    (terrain_edge_config_.required_snapshot_version != 0U &&
    binding.terrain_snapshot_version != terrain_edge_config_.required_snapshot_version)))
  {
    binding.valid = false;
  }
  return binding;
}

bool GlobalPlanner::isPlanningBindingCurrentLocked(
  const planner_safety::PlanningDataBinding & binding,
  std::string * rejection_reason) const
{
  const auto observed = captureCurrentPlanningBindingLocked();
  return planner_safety::planningBindingsMatch(binding, observed, rejection_reason);
}

bool GlobalPlanner::isPlanningBindingCurrent(
  const planner_safety::PlanningDataBinding & binding,
  std::string * rejection_reason) const
{
  const std::unique_lock<std::mutex> planner_lock(protect_kdtree_ground_);
  return isPlanningBindingCurrentLocked(binding, rejection_reason);
}

void GlobalPlanner::declareTerrainParameters()
{
  auto get_float_parameter = [this](const std::string & name, float default_value) {
      const double value = this->declare_parameter<double>(name, default_value);
      return static_cast<float>(value);
    };
  auto get_bool_parameter = [this](const std::string & name, bool default_value) {
      return this->declare_parameter<bool>(name, default_value);
    };

  auto & policy = terrain_edge_config_.policy;
  policy.enabled = get_bool_parameter("terrain.enabled", policy.enabled);
  policy.fail_closed = get_bool_parameter("terrain.fail_closed", policy.fail_closed);
  terrain_edge_config_.expected_map_hash = this->declare_parameter<std::string>(
    "terrain.map_hash", terrain_edge_config_.expected_map_hash);
  const auto required_version = this->declare_parameter<std::int64_t>(
    "terrain.required_snapshot_version",
    static_cast<std::int64_t>(terrain_edge_config_.required_snapshot_version));
  if (required_version < 0) {
    throw std::invalid_argument("terrain.required_snapshot_version cannot be negative");
  }
  terrain_edge_config_.required_snapshot_version =
    static_cast<std::uint64_t>(required_version);

  terrain_edge_config_.support_sample_spacing_m = get_float_parameter(
    "terrain.support_sample_spacing_m", terrain_edge_config_.support_sample_spacing_m);
  terrain_edge_config_.support_search_radius_m = get_float_parameter(
    "terrain.support_search_radius_m", terrain_edge_config_.support_search_radius_m);
  terrain_edge_config_.support_vertical_tolerance_m = get_float_parameter(
    "terrain.support_vertical_tolerance_m",
    terrain_edge_config_.support_vertical_tolerance_m);
  terrain_edge_config_.continuous_height_residual_m = get_float_parameter(
    "terrain.continuous_height_residual_m",
    terrain_edge_config_.continuous_height_residual_m);
  terrain_edge_config_.ground_index_alignment_tolerance_m = get_float_parameter(
    "terrain.ground_index_alignment_tolerance_m",
    terrain_edge_config_.ground_index_alignment_tolerance_m);

  policy.max_up_slope_rad = get_float_parameter(
    "terrain.max_up_slope_rad", policy.max_up_slope_rad);
  policy.max_down_slope_rad = get_float_parameter(
    "terrain.max_down_slope_rad", policy.max_down_slope_rad);
  policy.max_cross_slope_rad = get_float_parameter(
    "terrain.max_cross_slope_rad", policy.max_cross_slope_rad);
  policy.max_roughness_m = get_float_parameter(
    "terrain.max_roughness_m", policy.max_roughness_m);
  policy.max_normal_change_rad = get_float_parameter(
    "terrain.max_normal_change_rad", policy.max_normal_change_rad);
  policy.max_step_up_m = get_float_parameter("terrain.max_step_up_m", policy.max_step_up_m);
  policy.max_step_down_m = get_float_parameter(
    "terrain.max_step_down_m", policy.max_step_down_m);
  policy.min_support_ratio = get_float_parameter(
    "terrain.min_support_ratio", policy.min_support_ratio);
  policy.max_unknown_ratio = get_float_parameter(
    "terrain.max_unknown_ratio", policy.max_unknown_ratio);
  policy.min_confidence = get_float_parameter(
    "terrain.min_confidence", policy.min_confidence);
  policy.max_support_sample_spacing_m = get_float_parameter(
    "terrain.max_support_sample_spacing_m", policy.max_support_sample_spacing_m);

  policy.stair_enabled = get_bool_parameter("terrain.stair_enabled", policy.stair_enabled);
  policy.allow_stair_up = get_bool_parameter(
    "terrain.allow_stair_up", policy.allow_stair_up);
  policy.allow_stair_down = get_bool_parameter(
    "terrain.allow_stair_down", policy.allow_stair_down);
  policy.require_manual_corridor = get_bool_parameter(
    "terrain.require_manual_corridor", policy.require_manual_corridor);
  policy.require_online_confirmation = get_bool_parameter(
    "terrain.require_online_confirmation", policy.require_online_confirmation);
  const auto max_step_index_delta = this->declare_parameter<std::int64_t>(
    "terrain.max_step_index_delta", policy.max_step_index_delta);
  if (max_step_index_delta < std::numeric_limits<std::int32_t>::min() ||
    max_step_index_delta > std::numeric_limits<std::int32_t>::max())
  {
    throw std::invalid_argument("terrain.max_step_index_delta is outside int32 range");
  }
  policy.max_step_index_delta = static_cast<std::int32_t>(max_step_index_delta);
  policy.max_stair_riser_height_m = get_float_parameter(
    "terrain.max_stair_riser_height_m", policy.max_stair_riser_height_m);
  policy.min_stair_tread_depth_m = get_float_parameter(
    "terrain.min_stair_tread_depth_m", policy.min_stair_tread_depth_m);
  policy.max_stair_tread_depth_m = get_float_parameter(
    "terrain.max_stair_tread_depth_m", policy.max_stair_tread_depth_m);
  policy.max_stair_riser_deviation_m = get_float_parameter(
    "terrain.max_stair_riser_deviation_m", policy.max_stair_riser_deviation_m);
  policy.max_stair_tread_deviation_m = get_float_parameter(
    "terrain.max_stair_tread_deviation_m", policy.max_stair_tread_deviation_m);
  policy.max_stair_heading_error_rad = get_float_parameter(
    "terrain.max_stair_heading_error_rad", policy.max_stair_heading_error_rad);

  policy.distance_cost_weight = get_float_parameter(
    "terrain.distance_cost_weight", policy.distance_cost_weight);
  policy.slope_cost_weight = get_float_parameter(
    "terrain.slope_cost_weight", policy.slope_cost_weight);
  policy.cross_slope_cost_weight = get_float_parameter(
    "terrain.cross_slope_cost_weight", policy.cross_slope_cost_weight);
  policy.roughness_cost_weight = get_float_parameter(
    "terrain.roughness_cost_weight", policy.roughness_cost_weight);
  policy.risk_cost_weight = get_float_parameter(
    "terrain.risk_cost_weight", policy.risk_cost_weight);
  policy.stair_transition_cost = get_float_parameter(
    "terrain.stair_transition_cost", policy.stair_transition_cost);

  std::string terrain_error;
  if (!global_planner::TerrainEdgeValidator::validateConfiguration(
      terrain_edge_config_, &terrain_error))
  {
    throw std::invalid_argument("Invalid terrain configuration: " + terrain_error);
  }
  RCLCPP_INFO(
    this->get_logger(),
    "terrain.enabled=%d fail_closed=%d map_hash=%s required_snapshot_version=%llu",
    policy.enabled, policy.fail_closed,
    terrain_edge_config_.expected_map_hash.empty() ? "none" :
    terrain_edge_config_.expected_map_hash.c_str(),
    static_cast<unsigned long long>(terrain_edge_config_.required_snapshot_version));
}

void GlobalPlanner::checkPerception3DThread(){
  if (!perception_3d_ros_ || !perception_3d_ros_->getSharedDataPtr()) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *clock_, 1000, "Shared perception data is unavailable");
    return;
  }

  const auto shared_data = perception_3d_ros_->getSharedDataPtr();
  if (shared_data->getStaticGroundGeneration() == static_ground_generation_) {
    return;
  }

  // Global lock order is always planner state first, then perception data.
  // A* follows the same order through makeROSPlanWithBinding().
  std::unique_lock<std::mutex> planner_lock(protect_kdtree_ground_);
  std::unique_lock<std::recursive_mutex> perception_lock(
    shared_data->ground_kdtree_cb_mutex_);
  const std::uint64_t incoming_generation = shared_data->getStaticGroundGeneration();
  if (incoming_generation == static_ground_generation_) {
    return;
  }
  graph_ready_ = false;
  if (!shared_data->is_static_layer_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  if (incoming_generation == 0U || !shared_data->pcl_ground_ ||
    shared_data->pcl_ground_->empty() || !shared_data->kdtree_ground_ ||
    !shared_data->pcl_map_ || !shared_data->kdtree_map_ ||
    !shared_data->sGraph_ptr_)
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "Static layer generation %llu has incomplete map/graph inputs",
      static_cast<unsigned long long>(incoming_generation));
    return;
  }
  const auto shared_ground_input = shared_data->kdtree_ground_->getInputCloud();
  const auto shared_map_input = shared_data->kdtree_map_->getInputCloud();
  const auto shared_node_weight_size = shared_data->sGraph_ptr_->getNodeWeightSize();
  if (!shared_ground_input || shared_ground_input.get() != shared_data->pcl_ground_.get() ||
    !shared_map_input || shared_map_input.get() != shared_data->pcl_map_.get() ||
    shared_data->sGraph_ptr_->getSize() != shared_data->pcl_ground_->points.size() ||
    (shared_node_weight_size != 0U &&
    shared_node_weight_size != shared_data->pcl_ground_->points.size()))
  {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "Static layer generation %llu is internally inconsistent",
      static_cast<unsigned long long>(incoming_generation));
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr copied_ground(
    new pcl::PointCloud<pcl::PointXYZI>(*shared_data->pcl_ground_));
  pcl::PointCloud<pcl::PointXYZI>::Ptr copied_map(
    new pcl::PointCloud<pcl::PointXYZI>(*shared_data->pcl_map_));
  perception_3d::StaticGraph copied_static_graph = *shared_data->sGraph_ptr_;
  perception_lock.unlock();

  pcl_ground_ = std::move(copied_ground);
  pcl_map_ = std::move(copied_map);
  kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  kdtree_ground_->setInputCloud(pcl_ground_);
  kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  kdtree_map_->setInputCloud(pcl_map_);
  static_graph_ = std::move(copied_static_graph);
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();

  RCLCPP_INFO(
    this->get_logger(),
    "Ground, map, kd-trees, and static graph copied from generation %llu",
    static_cast<unsigned long long>(incoming_generation));
  getStaticGraphFromPerception3D();
  if (!graph_ready_) {
    return;
  }

  perception_lock.lock();
  if (!shared_data->is_static_layer_ready_ ||
    shared_data->getStaticGroundGeneration() != incoming_generation)
  {
    graph_ready_ = false;
    RCLCPP_WARN(
      this->get_logger(),
      "Static ground changed while generation %llu was being installed; retrying",
      static_cast<unsigned long long>(incoming_generation));
    return;
  }
  static_ground_generation_ = incoming_generation;
}

void GlobalPlanner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  geometry_msgs::msg::PoseStamped start, goal;

  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;
  goal.pose.orientation.w = 1.0;
  goal.header.frame_id = global_frame_;

  geometry_msgs::msg::TransformStamped transformStamped;

  try
  {
    transformStamped = tf2Buffer_->lookupTransform(
        global_frame_, robot_frame_, tf2::TimePointZero);
    start.pose.position.x = transformStamped.transform.translation.x;
    start.pose.position.y = transformStamped.transform.translation.y;
    start.pose.position.z = transformStamped.transform.translation.z;
  }
  catch (tf2::TransformException& e)
  {
    RCLCPP_INFO(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
    return;
  }

  auto bound_plan = makeROSPlanWithBinding(start, goal);
  if (bound_plan.path.poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "No bound path found for the clicked goal");
    return;
  }
  std::string rejection;
  if (!isPlanningBindingCurrent(bound_plan.binding, &rejection)) {
    RCLCPP_WARN(
      this->get_logger(), "Clicked-goal path became stale before publication: %s",
      rejection.c_str());
    return;
  }
  pub_path_->publish(bound_plan.path);
}

void GlobalPlanner::postSmoothPath(std::vector<unsigned int>& path_id, std::vector<unsigned int>& smoothed_path_id){
  
  smoothed_path_id.clear();
  if (path_id.empty()) {
    return;
  }
  if (path_id.size() == 1U) {
    smoothed_path_id.push_back(path_id.front());
    return;
  }
  geometry_msgs::msg::PoseStamped current_pst;
  current_pst.pose.position.x = pcl_ground_->points[path_id[0]].x;
  current_pst.pose.position.y = pcl_ground_->points[path_id[0]].y;
  current_pst.pose.position.z = pcl_ground_->points[path_id[0]].z;
  
  smoothed_path_id.push_back(path_id[0]);

  for(std::size_t it = 1U; it + 1U < path_id.size(); ++it){

    geometry_msgs::msg::PoseStamped next_pst;
    next_pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    next_pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    next_pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    double vx,vy,vz;
    vx = next_pst.pose.position.x - current_pst.pose.position.x;
    vy = next_pst.pose.position.y - current_pst.pose.position.y;
    vz = next_pst.pose.position.z - current_pst.pose.position.z;
    double unit = sqrt(vx*vx + vy*vy + vz*vz);
    
    tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    current_pst.pose.orientation.x = q.getX();
    current_pst.pose.orientation.y = q.getY();
    current_pst.pose.orientation.z = q.getZ();
    current_pst.pose.orientation.w = q.getW();     

    //@Interpolation to make global plan smoother and better resolution for local planner
    for(double step=0.05;step<0.99;step+=0.05){
      pcl::PointXYZI pst_inter_polate_pc;
      pst_inter_polate_pc.x = current_pst.pose.position.x + vx*step;
      pst_inter_polate_pc.y = current_pst.pose.position.y + vy*step;
      pst_inter_polate_pc.z = current_pst.pose.position.z + vz*step;
      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;
      if(kdtree_map_->radiusSearch(pst_inter_polate_pc, perception_3d_ros_->getGlobalUtils()->getInscribedRadius(), pointIdxRadiusSearch, pointRadiusSquaredDistance)>1){
        //@ no line of sight, keep this pose
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      if(kdtree_ground_->radiusSearch(pst_inter_polate_pc, 1.0, pointIdxRadiusSearch, pointRadiusSquaredDistance)<2){
        //@ not on the ground
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;
      }
      double dx = vx*step;
      double dy = vy*step;
      double dr = sqrt(dx*dx+dy*dy);
      double dz = fabs(vz*step);
      float vertical_angle = std::asin(dz / dr);
      if(dr>0.5 && vertical_angle>0.349){
        //@ z jump
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
      if(dr>20.0){
        //@ longer than 10 meter
        smoothed_path_id.push_back(path_id[it]);
        current_pst = next_pst;
        break;        
      }
    }
  }
  smoothed_path_id.push_back(path_id[path_id.size()-1]);
}

bool GlobalPlanner::getROSPath(
  const std::vector<unsigned int>& path_id,
  const planner_safety::PlanningDataBinding & binding,
  nav_msgs::msg::Path& ros_path)
{
  std::string rejection;
  if (!pcl_ground_ || !isPlanningBindingCurrentLocked(binding, &rejection)) {
    RCLCPP_WARN(
      this->get_logger(), "Path conversion rejected stale planning inputs: %s",
      rejection.c_str());
    ros_path = nav_msgs::msg::Path();
    return false;
  }
  std_msgs::msg::Header header;
  header.frame_id = global_frame_;
  header.stamp = clock_->now();
  if (terrain_edge_config_.policy.enabled) {
    const auto shared_data = perception_3d_ros_->getSharedDataPtr();
    const auto terrain_snapshot = shared_data ? shared_data->getTerrainSnapshot() : nullptr;
    std::string conversion_rejection;
    if (!planner_safety::buildTerrainAwarePath(
        *pcl_ground_, path_id, terrain_snapshot, binding, header,
        path_interpolation_resolution_, ros_path, &conversion_rejection))
    {
      RCLCPP_WARN(
        this->get_logger(), "Terrain-aware path conversion failed closed: %s",
        conversion_rejection.c_str());
      return false;
    }
  } else if (!planner_safety::buildInterpolatedPath(
      *pcl_ground_, path_id, header, path_interpolation_resolution_, ros_path))
  {
    return false;
  }
  if (!isPlanningBindingCurrentLocked(binding, &rejection)) {
    RCLCPP_WARN(
      this->get_logger(), "Planning inputs changed during path conversion: %s",
      rejection.c_str());
    ros_path = nav_msgs::msg::Path();
    return false;
  }
  return true;
}

bool GlobalPlanner::getStartGoalID(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal,
                                    unsigned int& start_id, unsigned int& goal_id){
  start_id = std::numeric_limits<unsigned int>::max();
  goal_id = std::numeric_limits<unsigned int>::max();
  if (!graph_ready_ || !has_initialized_ || !pcl_ground_ || pcl_ground_->empty() ||
    !kdtree_ground_ || !planner_safety::isFinitePoint(start.pose.position) ||
    !planner_safety::isFinitePoint(goal.pose.position))
  {
    RCLCPP_WARN(this->get_logger(), "Ground graph or start/goal pose is not valid");
    return false;
  }

  pcl::PointXYZI pcl_goal;
  pcl_goal.x = goal.pose.position.x;
  pcl_goal.y = goal.pose.position.y;
  pcl_goal.z = goal.pose.position.z;

  std::vector<int> pointIdxRadiusSearch_goal;
  std::vector<float> pointRadiusSquaredDistance_goal;
  kdtree_ground_->radiusSearch(
    pcl_goal, find_goal_tolerance_, pointIdxRadiusSearch_goal,
    pointRadiusSquaredDistance_goal);

  planner_safety::GroundCandidateCriteria goal_criteria;
  goal_criteria.max_horizontal_distance = find_goal_tolerance_;
  goal_criteria.max_vertical_distance = find_goal_tolerance_;
  goal_criteria.layer_separation = ground_layer_separation_;
  goal_criteria.ambiguity_xy_distance = layer_ambiguity_xy_distance_;
  goal_criteria.max_above_query = find_goal_tolerance_;
  auto goal_selection = planner_safety::selectGroundCandidate(
    *pcl_ground_, pcl_goal, pointIdxRadiusSearch_goal, goal_criteria);

  if (goal_selection.status == planner_safety::GroundCandidateStatus::AMBIGUOUS_LAYER) {
    RCLCPP_WARN(this->get_logger(), "Goal overlaps multiple ground layers; refusing to guess");
    return false;
  }

  if (goal_selection.status != planner_safety::GroundCandidateStatus::SUCCESS) {
    RCLCPP_WARN(
      this->get_logger(),
      "Goal has no nearby ground; trying a bounded downward projection of %.2f m",
      max_vertical_search_distance_);

    std::unordered_set<int> unique_vertical_candidates;
    constexpr double vertical_step = 0.1;
    for (double drop = 0.0; drop <= max_vertical_search_distance_ + 1e-6;
      drop += vertical_step)
    {
      pcl::PointXYZI vertical_query = pcl_goal;
      vertical_query.z = pcl_goal.z - drop;
      std::vector<int> slice_indices;
      std::vector<float> slice_distances;
      kdtree_ground_->radiusSearch(
        vertical_query, 0.3, slice_indices, slice_distances);
      unique_vertical_candidates.insert(slice_indices.begin(), slice_indices.end());
    }

    pointIdxRadiusSearch_goal.assign(
      unique_vertical_candidates.begin(), unique_vertical_candidates.end());
    goal_criteria.max_horizontal_distance = 0.3;
    goal_criteria.max_vertical_distance = max_vertical_search_distance_;
    goal_criteria.max_above_query = 0.05;
    goal_selection = planner_safety::selectGroundCandidate(
      *pcl_ground_, pcl_goal, pointIdxRadiusSearch_goal, goal_criteria);

    if (goal_selection.status == planner_safety::GroundCandidateStatus::AMBIGUOUS_LAYER) {
      RCLCPP_WARN(
        this->get_logger(), "Bounded goal projection found multiple ground layers; rejecting goal");
      return false;
    }
    if (goal_selection.status != planner_safety::GroundCandidateStatus::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Bounded goal projection found no ground");
      return false;
    }
  }

  goal_id = goal_selection.index;
  
  if(enable_detail_log_){
    RCLCPP_WARN(this->get_logger(), "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id,
      pcl_ground_->points[goal_id].x, pcl_ground_->points[goal_id].y, pcl_ground_->points[goal_id].z);
  }
  else{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Selected goal: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z, goal_id,
      pcl_ground_->points[goal_id].x, pcl_ground_->points[goal_id].y, pcl_ground_->points[goal_id].z);
  }
  
  //--------------------------------------------------------------------------------------
  //@Get start ID
  std::vector<int> pointIdxRadiusSearch_start;
  std::vector<float> pointRadiusSquaredDistance_start;
  pcl::PointXYZI pcl_start;
  pcl_start.x = start.pose.position.x;
  pcl_start.y = start.pose.position.y;
  pcl_start.z = start.pose.position.z - robot_ground_z_offset_;

  kdtree_ground_->radiusSearch(
    pcl_start, find_start_tolerance_, pointIdxRadiusSearch_start,
    pointRadiusSquaredDistance_start);
  planner_safety::GroundCandidateCriteria start_criteria;
  start_criteria.max_horizontal_distance = find_start_tolerance_;
  start_criteria.max_vertical_distance = find_start_tolerance_;
  start_criteria.layer_separation = ground_layer_separation_;
  start_criteria.ambiguity_xy_distance = layer_ambiguity_xy_distance_;
  start_criteria.max_above_query = find_start_tolerance_;
  const auto start_selection = planner_safety::selectGroundCandidate(
    *pcl_ground_, pcl_start, pointIdxRadiusSearch_start, start_criteria);
  if (start_selection.status == planner_safety::GroundCandidateStatus::AMBIGUOUS_LAYER) {
    RCLCPP_WARN(this->get_logger(), "Start overlaps multiple ground layers; refusing to guess");
    return false;
  }
  if(start_selection.status != planner_safety::GroundCandidateStatus::SUCCESS){
    RCLCPP_WARN(this->get_logger(), "Start is not found.");
    return false;
  }
  start_id = start_selection.index;
  
  if(enable_detail_log_){
    RCLCPP_WARN(this->get_logger(), "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id,
      pcl_ground_->points[start_id].x, pcl_ground_->points[start_id].y, pcl_ground_->points[start_id].z);
  }
  else{
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Selected start: %.2f, %.2f, %.2f, Nearest-> id: %u, x: %.2f, y: %.2f, z: %.2f", 
      start.pose.position.x, start.pose.position.y, start.pose.position.z, start_id,
      pcl_ground_->points[start_id].x, pcl_ground_->points[start_id].y, pcl_ground_->points[start_id].z);

  }
  return true;

}

bool GlobalPlanner::isRawGoalSupported(
  const geometry_msgs::msg::PoseStamped& goal, unsigned int goal_id,
  const planner_safety::PlanningDataBinding & binding)
{
  if (!perception_3d_ros_ || !perception_3d_ros_->getSharedDataPtr() || !pcl_ground_ ||
    goal_id >= pcl_ground_->points.size() ||
    !planner_safety::isFinitePoint(goal.pose.position))
  {
    return false;
  }

  std::unique_lock<std::recursive_mutex> perception_lock(
    perception_3d_ros_->getSharedDataPtr()->ground_kdtree_cb_mutex_);

  std::string binding_rejection;
  if (!isPlanningBindingCurrentLocked(binding, &binding_rejection)) {
    RCLCPP_WARN(
      this->get_logger(), "Raw-goal validation rejected stale inputs: %s",
      binding_rejection.c_str());
    return false;
  }

  const auto & selected_ground = pcl_ground_->points[goal_id];
  if (!std::isfinite(selected_ground.x) || !std::isfinite(selected_ground.y) ||
    !std::isfinite(selected_ground.z))
  {
    return false;
  }
  const double horizontal_offset = std::hypot(
    goal.pose.position.x - selected_ground.x,
    goal.pose.position.y - selected_ground.y);
  const double maximum_vertical_offset = std::max(
    find_goal_tolerance_, max_vertical_search_distance_);
  if (horizontal_offset > find_goal_tolerance_ ||
    std::abs(goal.pose.position.z - selected_ground.z) > maximum_vertical_offset)
  {
    RCLCPP_WARN(this->get_logger(), "Raw goal is too far from its selected ground node");
    return false;
  }

  const double clearance = perception_3d_ros_->get_min_dGraphValue(goal_id);
  const double inscribed_radius = perception_3d_ros_->getGlobalUtils()->getInscribedRadius();
  if (!std::isfinite(clearance) || clearance < inscribed_radius) {
    RCLCPP_WARN(this->get_logger(), "Raw goal ground node is blocked or has invalid clearance");
    return false;
  }

  pcl::PointXYZI support_query;
  support_query.x = goal.pose.position.x;
  support_query.y = goal.pose.position.y;
  support_query.z = selected_ground.z;
  std::vector<int> support_indices;
  std::vector<float> support_distances;
  const double support_search_radius = std::hypot(
    raw_goal_support_radius_, raw_goal_support_z_tolerance_);
  kdtree_ground_->radiusSearch(
    support_query, support_search_radius, support_indices, support_distances);

  planner_safety::GoalSupportCriteria support_criteria;
  support_criteria.radius = raw_goal_support_radius_;
  support_criteria.layer_z_tolerance = raw_goal_support_z_tolerance_;
  support_criteria.minimum_points = static_cast<std::size_t>(raw_goal_minimum_support_points_);
  support_criteria.minimum_sectors = static_cast<std::size_t>(raw_goal_minimum_support_sectors_);
  if (!planner_safety::hasGoalSupport(
      *pcl_ground_, support_indices, goal.pose.position, selected_ground.z, support_criteria))
  {
    RCLCPP_WARN(this->get_logger(), "Raw goal does not have enough same-layer ground support");
    return false;
  }
  if (!isPlanningBindingCurrentLocked(binding, &binding_rejection)) {
    RCLCPP_WARN(
      this->get_logger(), "Planning inputs changed during raw-goal validation: %s",
      binding_rejection.c_str());
    return false;
  }
  return true;
}

void GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  const auto planning_started = clock_->now();
  const auto make_result = [this, &planning_started](const BoundGlobalPlan & plan) {
      auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
      populateGetPlanResult(plan, *result);
      result->planning_time = static_cast<builtin_interfaces::msg::Duration>(
        clock_->now() - planning_started);
      return result;
    };

  //@get goal and start
  const auto goal = goal_handle->get_goal();

  if(!goal_handle->get_goal()->activate_threading){
    BoundGlobalPlan inactive;
    inactive.status_code = PlanStatusCode::DEACTIVATED;
    inactive.status_reason = "DEACTIVATED";
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Deactivate thread");
    goal_handle->succeed(make_result(inactive));
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);

  auto bound_plan = makeROSPlanWithBinding(start, goal->goal);
  auto & ros_path = bound_plan.path;

  if(ros_path.poses.empty()){
    goal_handle->abort(make_result(bound_plan));
  }
  else{
    std::string rejection;
    if (!isPlanningBindingCurrent(bound_plan.binding, &rejection)) {
      RCLCPP_WARN(
        this->get_logger(), "Global path became stale before publication: %s",
        rejection.c_str());
      bound_plan.path = nav_msgs::msg::Path();
      bound_plan.status_code = PlanStatusCode::INPUT_CHANGED;
      bound_plan.status_reason = "PLANNING_INPUT_CHANGED_BEFORE_PUBLICATION:" + rejection;
      goal_handle->abort(make_result(bound_plan));
      return;
    }
    pub_path_->publish(ros_path);
    goal_handle->succeed(make_result(bound_plan));
  }
  
}

nav_msgs::msg::Path GlobalPlanner::makeROSPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal){
  return makeROSPlanWithBinding(start, goal).path;
}

BoundGlobalPlan GlobalPlanner::makeROSPlanWithBinding(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal)
{
  BoundGlobalPlan result;
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
  result.binding = captureCurrentPlanningBindingLocked();
  if (!result.binding.valid) {
    RCLCPP_WARN(this->get_logger(), "No current static/terrain planning binding is available");
    result.status_code = PlanStatusCode::INPUT_NOT_READY;
    result.status_reason = "PLANNING_INPUT_NOT_READY";
    return result;
  }
  unsigned int start_id = std::numeric_limits<unsigned int>::max();
  unsigned int goal_id = std::numeric_limits<unsigned int>::max();
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  auto & ros_path = result.path;

  if(getStartGoalID(start, goal, start_id, goal_id)){
    std::string rejection;
    if (!isPlanningBindingCurrentLocked(result.binding, &rejection)) {
      RCLCPP_WARN(
        this->get_logger(), "Planning inputs changed before A* search: %s",
        rejection.c_str());
      result.status_code = PlanStatusCode::INPUT_CHANGED;
      result.status_reason = "PLANNING_INPUT_CHANGED_BEFORE_SEARCH:" + rejection;
      return result;
    }
    if(!use_pre_graph_ && a_star_planner_)
      a_star_planner_->getPath(
        start_id, goal_id, path, &result.binding, &result.terrain_statistics);
    else if(use_pre_graph_ && a_star_planner_pre_graph_)
      a_star_planner_pre_graph_->getPath(
        start_id, goal_id, path, &result.binding, &result.terrain_statistics);
    else {
      result.status_code = PlanStatusCode::INPUT_NOT_READY;
      result.status_reason = "A_STAR_PLANNER_NOT_READY";
      return result;
    }
  } else {
    result.status_code = PlanStatusCode::START_GOAL_UNSUPPORTED;
    result.status_reason = "START_OR_GOAL_UNSUPPORTED";
    return result;
  }

  if(path.empty()){
    result.status_code = PlanStatusCode::NO_PATH;
    result.status_reason = "NO_PATH";
    if(enable_detail_log_)
      RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "No path found from: %u to %u", start_id, goal_id);
    return result;
  }
  else{
    if(enable_detail_log_)
      RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "Path found from: %u to %u", start_id, goal_id);
    if (!getROSPath(path, result.binding, ros_path)) {
      RCLCPP_WARN(this->get_logger(), "Path conversion failed; returning no path");
      result.path = nav_msgs::msg::Path();
      result.status_code = PlanStatusCode::PATH_INVALID;
      result.status_reason = "PATH_CONVERSION_FAILED";
      return result;
    }
    if (!isRawGoalSupported(goal, goal_id, result.binding)) {
      RCLCPP_WARN(this->get_logger(), "Raw goal support validation failed; returning no path");
      result.path = nav_msgs::msg::Path();
      result.status_code = PlanStatusCode::START_GOAL_UNSUPPORTED;
      result.status_reason = "RAW_GOAL_UNSUPPORTED";
      return result;
    }

    geometry_msgs::msg::PoseStamped validated_goal = goal;
    validated_goal.header = ros_path.header;
    const auto & input_orientation = validated_goal.pose.orientation;
    const double orientation_norm = std::sqrt(
      input_orientation.x * input_orientation.x + input_orientation.y * input_orientation.y +
      input_orientation.z * input_orientation.z + input_orientation.w * input_orientation.w);
    if (!std::isfinite(orientation_norm) || orientation_norm <= 1e-6) {
      validated_goal.pose.orientation = ros_path.poses.back().pose.orientation;
    } else {
      validated_goal.pose.orientation.x /= orientation_norm;
      validated_goal.pose.orientation.y /= orientation_norm;
      validated_goal.pose.orientation.z /= orientation_norm;
      validated_goal.pose.orientation.w /= orientation_norm;
    }

    const auto path_end = ros_path.poses.back().pose.position;
    const double raw_goal_gap = std::sqrt(
      std::pow(validated_goal.pose.position.x - path_end.x, 2.0) +
      std::pow(validated_goal.pose.position.y - path_end.y, 2.0) +
      std::pow(validated_goal.pose.position.z - path_end.z, 2.0));
    if (raw_goal_gap <= 1e-6) {
      ros_path.poses.back().pose.orientation = validated_goal.pose.orientation;
    } else if (terrain_edge_config_.policy.enabled) {
      // The A* search context intentionally owns one immutable terrain
      // snapshot.  Appending a raw-goal segment after that context is gone
      // would bypass edge policy, support sampling, and snapshot binding.
      // Stop at the selected, validated mapground node until the raw segment
      // can be evaluated as part of the same search transaction.
      ros_path.poses.back().pose.orientation = validated_goal.pose.orientation;
      RCLCPP_WARN(
        this->get_logger(),
        "Terrain validation is enabled; stopping %.3f m before the raw goal at its "
        "validated mapground node",
        raw_goal_gap);
    } else {
      const auto safe_end = ros_path.poses.back();
      for (double travelled = path_interpolation_resolution_; travelled < raw_goal_gap;
        travelled += path_interpolation_resolution_)
      {
        const double ratio = travelled / raw_goal_gap;
        geometry_msgs::msg::PoseStamped interpolated = safe_end;
        interpolated.pose.position.x = path_end.x +
          (validated_goal.pose.position.x - path_end.x) * ratio;
        interpolated.pose.position.y = path_end.y +
          (validated_goal.pose.position.y - path_end.y) * ratio;
        interpolated.pose.position.z = path_end.z +
          (validated_goal.pose.position.z - path_end.z) * ratio;
        ros_path.poses.push_back(interpolated);
      }
      ros_path.poses.push_back(validated_goal);
    }
    std::string rejection;
    if (!isPlanningBindingCurrentLocked(result.binding, &rejection)) {
      RCLCPP_WARN(
        this->get_logger(), "Planning inputs changed before returning the path: %s",
        rejection.c_str());
      result.path = nav_msgs::msg::Path();
      result.status_code = PlanStatusCode::INPUT_CHANGED;
      result.status_reason = "PLANNING_INPUT_CHANGED_BEFORE_RETURN:" + rejection;
      return result;
    }
    result.status_code = PlanStatusCode::SUCCESS;
    result.status_reason = "SUCCESS";
    return result;
  }
}

void GlobalPlanner::getStaticGraphFromPerception3D(){
  
  //@Calculate node weight
  graph_ready_ = false;
  if (!pcl_ground_ || pcl_ground_->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot initialize an empty ground graph");
    return;
  }

  if(!has_initialized_){
    if(a_star_expanding_radius_ >= perception_3d_ros_->getGlobalUtils()->getInscribedRadius()*2){
      RCLCPP_WARN(this->get_logger(), "Expanding radius is much larger than InscribedRadius, the planning time will be increased.");
    }
    if(!use_pre_graph_){
      a_star_planner_ = std::make_shared<A_Star_on_Graph>(
        pcl_ground_, perception_3d_ros_, a_star_expanding_radius_, terrain_edge_config_);
      a_star_planner_->setupTurningWeight(turning_weight_);
    }
    else{
      a_star_planner_pre_graph_ = std::make_shared<A_Star_on_PreGraph>(
        pcl_ground_, static_graph_, perception_3d_ros_, a_star_expanding_radius_,
        terrain_edge_config_);
      a_star_planner_pre_graph_->setupTurningWeight(turning_weight_);
    }
    has_initialized_ = true;
  }
  else{
    if (!use_pre_graph_ && a_star_planner_) {
      a_star_planner_->updateGraph(pcl_ground_);
    } else if (use_pre_graph_ && a_star_planner_pre_graph_) {
      a_star_planner_pre_graph_->updateGraph(pcl_ground_, static_graph_);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Configured A* planner is missing during graph update");
      has_initialized_ = false;
      return;
    }
  }

  pubWeight();

  RCLCPP_INFO(this->get_logger(), "Publish weighted ground point cloud.");
  graph_ready_ = true;
}


void GlobalPlanner::pubWeight(){

  /*
  When generate_static_graph is true, the static_layer will generate sGraph_ptr_.
  And there will be wo condition:
  1. enable_edge_detection = false; This arg will push zero element to node_weight_ 
  */

  pcl::PointCloud<pcl::PointXYZI>::Ptr weighted_pc (new pcl::PointCloud<pcl::PointXYZI>);
  
  unsigned long node_weight_size = static_graph_.getNodeWeightSize();
  bool is_node_weight = true;
  if(node_weight_size<=0){
    node_weight_size = static_graph_.getSize();
    is_node_weight = false;
  }
  if (!pcl_ground_ || node_weight_size != pcl_ground_->points.size()) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Refusing to publish weighted ground with graph/cloud size mismatch: %lu/%zu",
      node_weight_size, pcl_ground_ ? pcl_ground_->points.size() : 0U);
    return;
  }

  for(std::size_t it = 0U; it < node_weight_size; ++it){

    pcl::PointXYZI ipt;

    ipt.x = pcl_ground_->points[it].x;
    ipt.y = pcl_ground_->points[it].y;
    ipt.z = pcl_ground_->points[it].z;
    if(is_node_weight)
      ipt.intensity = static_graph_.getNodeWeight(it);
    else
      ipt.intensity = 0;
    weighted_pc->push_back(ipt);

  }
  weighted_pc->header.frame_id = global_frame_;
  sensor_msgs::msg::PointCloud2 ros_msg_weighted_pc;
  ros_msg_weighted_pc.header.stamp = clock_->now();
  pcl::toROSMsg(*weighted_pc, ros_msg_weighted_pc);
  pub_weighted_pc_->publish(ros_msg_weighted_pc);
}

void GlobalPlanner::pubStaticGraph(){
 
  //@edge visualization 
  //@This is just for visulization, therefore reduce edges to let rviz less lag
  
  std::set<std::pair<unsigned int, unsigned int>> duplicate_check;
  visualization_msgs::msg::MarkerArray markerArray;
  visualization_msgs::msg::Marker markerEdge;
  markerEdge.header.frame_id = global_frame_;
  markerEdge.header.stamp = clock_->now();
  markerEdge.action = visualization_msgs::msg::Marker::ADD;
  markerEdge.type = visualization_msgs::msg::Marker::LINE_LIST;
  markerEdge.pose.orientation.w = 1.0;
  markerEdge.ns = "edges";
  markerEdge.id = 3;
  markerEdge.scale.x = 0.03;
  markerEdge.color.r = 0.9; markerEdge.color.g = 1; markerEdge.color.b = 0;
  markerEdge.color.a = 0.2;

  graph_t* static_graph; //std::unordered_map<unsigned int, std::set<edge_t>> typedef in static_graph.h
  static_graph = static_graph_.getGraphPtr();

  int cnt = 0;
  for(auto it = (*static_graph).begin();it!=(*static_graph).end();it++){
    geometry_msgs::msg::Point p;
    p.x = pcl_ground_->points[(*it).first].x;
    p.y = pcl_ground_->points[(*it).first].y;
    p.z = pcl_ground_->points[(*it).first].z;
    for(auto it_set = (*it).second.begin();it_set != (*it).second.end();it_set++){

      std::pair<unsigned int, unsigned int> edge_marker, edge_marker_inverse;
      edge_marker.first = (*it).first;
      edge_marker.second = (*it_set).first;
      edge_marker_inverse.first = (*it_set).first;
      edge_marker_inverse.second = (*it).first;
      if( !duplicate_check.insert(edge_marker).second )
      {   
        continue;
      }
      if( !duplicate_check.insert(edge_marker_inverse).second )
      {   
        continue;
      }
      markerEdge.points.push_back(p);
      p.x = pcl_ground_->points[(*it_set).first].x;
      p.y = pcl_ground_->points[(*it_set).first].y;
      p.z = pcl_ground_->points[(*it_set).first].z;     
      markerEdge.points.push_back(p);
      markerEdge.id = cnt;
      cnt++;
    }
  }
  markerArray.markers.push_back(markerEdge);
  pub_static_graph_->publish(markerArray);
}

}
