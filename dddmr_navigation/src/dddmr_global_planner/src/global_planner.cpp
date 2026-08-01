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
#include <global_planner/endpoint_clearance_policy.h>
#include <global_planner/endpoint_projection_policy.h>
#include <global_planner/global_planner.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;

namespace global_planner
{

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
  
  static_ground_size_ = 0;
  perception_3d_ros_ = perception_3d;
  graph_ready_ = false;
  has_initialized_ = false;
  robot_frame_ = perception_3d_ros_->getGlobalUtils()->getRobotFrame();
  global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
  global_plan_result_ = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
  
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

  declare_parameter("maximum_ground_connection_z", rclcpp::ParameterValue(0.26));
  this->get_parameter("maximum_ground_connection_z", maximum_ground_connection_z_);
  RCLCPP_INFO(this->get_logger(), "maximum_ground_connection_z: %.2f", maximum_ground_connection_z_);

  declare_parameter("use_pre_graph", rclcpp::ParameterValue(false));
  this->get_parameter("use_pre_graph", use_pre_graph_);
  RCLCPP_INFO(this->get_logger(), "use_pre_graph: %d", use_pre_graph_);    

  declare_parameter("find_start_tolerance", rclcpp::ParameterValue(0.5));
  this->get_parameter("find_start_tolerance", find_start_tolerance_);
  RCLCPP_INFO(this->get_logger(), "find_start_tolerance: %.2f", find_start_tolerance_);    

  declare_parameter("find_goal_tolerance", rclcpp::ParameterValue(0.5));
  this->get_parameter("find_goal_tolerance", find_goal_tolerance_);
  RCLCPP_INFO(this->get_logger(), "find_goal_tolerance: %.2f", find_goal_tolerance_);

  declare_parameter(
    "project_start_goal_to_traversable_ground", rclcpp::ParameterValue(true));
  this->get_parameter(
    "project_start_goal_to_traversable_ground",
    project_start_goal_to_traversable_ground_);
  RCLCPP_INFO(
    this->get_logger(), "project_start_goal_to_traversable_ground: %d",
    project_start_goal_to_traversable_ground_);

  declare_parameter("max_endpoint_projection_xy", rclcpp::ParameterValue(0.35));
  this->get_parameter("max_endpoint_projection_xy", max_endpoint_projection_xy_);
  RCLCPP_INFO(
    this->get_logger(), "max_endpoint_projection_xy: %.2f",
    max_endpoint_projection_xy_);

  declare_parameter("max_endpoint_projection_z", rclcpp::ParameterValue(0.35));
  this->get_parameter("max_endpoint_projection_z", max_endpoint_projection_z_);
  RCLCPP_INFO(
    this->get_logger(), "max_endpoint_projection_z: %.2f",
    max_endpoint_projection_z_);

  // Keep the endpoint parameters as backwards-compatible defaults while
  // allowing an RViz ground goal and the robot's base-frame start pose to use
  // different vertical projection limits.
  declare_parameter(
    "max_start_projection_xy",
    rclcpp::ParameterValue(max_endpoint_projection_xy_));
  this->get_parameter("max_start_projection_xy", max_start_projection_xy_);
  declare_parameter(
    "max_start_projection_z",
    rclcpp::ParameterValue(max_endpoint_projection_z_));
  this->get_parameter("max_start_projection_z", max_start_projection_z_);
  declare_parameter(
    "max_goal_projection_xy",
    rclcpp::ParameterValue(max_endpoint_projection_xy_));
  this->get_parameter("max_goal_projection_xy", max_goal_projection_xy_);
  declare_parameter(
    "max_goal_projection_z",
    rclcpp::ParameterValue(max_endpoint_projection_z_));
  this->get_parameter("max_goal_projection_z", max_goal_projection_z_);
  RCLCPP_INFO(
    this->get_logger(),
    "endpoint projection limits: start XY %.2f m / Z %.2f m, goal XY %.2f m / Z %.2f m",
    max_start_projection_xy_, max_start_projection_z_,
    max_goal_projection_xy_, max_goal_projection_z_);

  declare_parameter(
    "goal_projection_surface_max_z_gap",
    rclcpp::ParameterValue(maximum_ground_connection_z_));
  this->get_parameter(
    "goal_projection_surface_max_z_gap",
    goal_projection_surface_max_z_gap_);
  declare_parameter(
    "goal_projection_min_surface_points", rclcpp::ParameterValue(3));
  this->get_parameter(
    "goal_projection_min_surface_points",
    goal_projection_min_surface_points_);
  RCLCPP_INFO(
    this->get_logger(),
    "2-D goal projection: surface max Z gap %.2f m, minimum surface points %d",
    goal_projection_surface_max_z_gap_,
    goal_projection_min_surface_points_);

  declare_parameter(
    "allow_start_in_dynamic_inflation", rclcpp::ParameterValue(false));
  this->get_parameter(
    "allow_start_in_dynamic_inflation",
    allow_start_in_dynamic_inflation_);
  declare_parameter(
    "start_static_clearance_layer", rclcpp::ParameterValue("map"));
  this->get_parameter(
    "start_static_clearance_layer", start_static_clearance_layer_);
  RCLCPP_INFO(
    this->get_logger(),
    "dynamic start escape: enabled=%d static_layer=%s",
    allow_start_in_dynamic_inflation_,
    start_static_clearance_layer_.c_str());

  if (
    a_star_expanding_radius_ <= 0.0 || maximum_ground_connection_z_ <= 0.0 ||
    find_start_tolerance_ <= 0.0 || find_goal_tolerance_ <= 0.0 ||
    max_endpoint_projection_xy_ <= 0.0 || max_endpoint_projection_z_ <= 0.0 ||
    max_start_projection_xy_ <= 0.0 || max_start_projection_z_ <= 0.0 ||
    max_goal_projection_xy_ <= 0.0 || max_goal_projection_z_ <= 0.0 ||
    goal_projection_surface_max_z_gap_ <= 0.0 ||
    goal_projection_min_surface_points_ <= 0)
  {
    throw std::invalid_argument("global planner endpoint tolerances must be positive");
  }
  if (
    allow_start_in_dynamic_inflation_ &&
    start_static_clearance_layer_.empty())
  {
    throw std::invalid_argument(
      "start_static_clearance_layer must not be empty when dynamic start escape is enabled");
  }

  

  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@Initialize transform listener and broadcaster
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
#if __has_include(<tf2_ros/create_timer_ros.hpp>)
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
#else
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
#endif
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

void GlobalPlanner::checkPerception3DThread(){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Waiting for static layer");
    return;
  }
  
  if(static_ground_size_!=perception_3d_ros_->getSharedDataPtr()->static_ground_size_){
    std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
    *pcl_ground_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_ground_);
    global_frame_ = perception_3d_ros_->getGlobalUtils()->getGblFrame();
    *kdtree_ground_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_ground_);
    *kdtree_map_ = *(perception_3d_ros_->getSharedDataPtr()->kdtree_map_);
    *pcl_map_ = *(perception_3d_ros_->getSharedDataPtr()->pcl_map_);
    static_graph_ = *perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_; //@ node weight
    RCLCPP_INFO(this->get_logger(), "Ground and Kd-tree ground have been received from perception_3d.");
    getStaticGraphFromPerception3D();
    static_ground_size_ = perception_3d_ros_->getSharedDataPtr()->static_ground_size_;
  }

}

void GlobalPlanner::cbClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr clicked_goal){
  
  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received clicked goal before static layer is ready");
    return;
  }

  geometry_msgs::msg::PoseStamped start, goal;

  goal.pose.position.x = clicked_goal->point.x;
  goal.pose.position.y = clicked_goal->point.y;
  goal.pose.position.z = clicked_goal->point.z;

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
  }
  
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
  unsigned int start_id = 0;
  unsigned int goal_id = 0;
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;

  const bool endpoints_found = getStartGoalID(
    start, goal, false, start_id, goal_id);
  if(endpoints_found){
    if(!use_pre_graph_)
      a_star_planner_->getPath(start_id, goal_id, path);
    else
      a_star_planner_pre_graph_->getPath(start_id, goal_id, path);
  }


  if(!endpoints_found){
    RCLCPP_WARN(this->get_logger(), "No path: a traversable start or goal was not found.");
  }
  else if(path.empty()){
    RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
  }
  else{
    //postSmoothPath(path, smoothed_path);
    getROSPath(path, ros_path);
    pub_path_->publish(ros_path);
    RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
  }

}

void GlobalPlanner::postSmoothPath(std::vector<unsigned int>& path_id, std::vector<unsigned int>& smoothed_path_id){
  
  smoothed_path_id.clear();
  geometry_msgs::msg::PoseStamped current_pst;
  current_pst.pose.position.x = pcl_ground_->points[path_id[0]].x;
  current_pst.pose.position.y = pcl_ground_->points[path_id[0]].y;
  current_pst.pose.position.z = pcl_ground_->points[path_id[0]].z;
  
  smoothed_path_id.push_back(path_id[0]);

  for(auto it=1;it<path_id.size()-1;it++){

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

void GlobalPlanner::getROSPath(std::vector<unsigned int>& path_id, nav_msgs::msg::Path& ros_path){

  ros_path.header.frame_id = global_frame_;
  ros_path.header.stamp = clock_->now();


  for(auto it=0;it<path_id.size();it++){
    geometry_msgs::msg::PoseStamped pst;
    pst.header = ros_path.header;
    pst.pose.position.x = pcl_ground_->points[path_id[it]].x;
    pst.pose.position.y = pcl_ground_->points[path_id[it]].y;
    pst.pose.position.z = pcl_ground_->points[path_id[it]].z;

    geometry_msgs::msg::PoseStamped next_pst;
    if(it<path_id.size()-1){
      next_pst.pose.position.x = pcl_ground_->points[path_id[it+1]].x;
      next_pst.pose.position.y = pcl_ground_->points[path_id[it+1]].y;
      next_pst.pose.position.z = pcl_ground_->points[path_id[it+1]].z;
    }


    double vx,vy,vz;
    vx = next_pst.pose.position.x - pst.pose.position.x;
    vy = next_pst.pose.position.y - pst.pose.position.y;
    vz = next_pst.pose.position.z - pst.pose.position.z;

    if(vz!=0){
      double unit = sqrt(vx*vx + vy*vy + vz*vz);
      
      tf2::Vector3 axis_vector(vx/unit, vy/unit, vz/unit);

      tf2::Vector3 up_vector(1.0, 0.0, 0.0);
      tf2::Vector3 right_vector = axis_vector.cross(up_vector);
      right_vector.normalized();
      tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
      q.normalize();
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }
    else{
      //@ handle with 2D
      double yaw = atan2(vy, vx);
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      pst.pose.orientation.x = q.getX();
      pst.pose.orientation.y = q.getY();
      pst.pose.orientation.z = q.getZ();
      pst.pose.orientation.w = q.getW();
    }

    //RCLCPP_INFO(this->get_logger(), "%.2f, %.2f, %.2f,%.2f, %.2f, %.2f, %.2f", vx, vy, vz, q.getX(), q.getY(), q.getZ(), q.getW());
    //@Interpolation to make global plan smoother and better resolution for local planner
    geometry_msgs::msg::PoseStamped pst_inter_polate = pst;
    if(it<path_id.size()-1){
      ros_path.poses.push_back(pst);
      geometry_msgs::msg::PoseStamped last_pst = pst;
      for(double step=0.05;step<0.99;step+=0.05){

        pst_inter_polate.pose.position.x = pst.pose.position.x + vx*step;
        pst_inter_polate.pose.position.y = pst.pose.position.y + vy*step;
        pst_inter_polate.pose.position.z = pst.pose.position.z + vz*step;
        double dx = pst_inter_polate.pose.position.x-last_pst.pose.position.x;
        double dy = pst_inter_polate.pose.position.y-last_pst.pose.position.y;
        double dz = pst_inter_polate.pose.position.z-last_pst.pose.position.z;
        if(sqrt(dx*dx+dy*dy+dz*dz)>0.1){
          ros_path.poses.push_back(pst_inter_polate);
          last_pst = pst_inter_polate;
        }
        
      }
    }
    else{
      ros_path.poses.push_back(pst);
    }
    
  }
}

bool GlobalPlanner::selectTraversableGround(
  const pcl::PointXYZI & requested, double search_radius,
  double max_projection_xy, double max_projection_z,
  const char * endpoint_name, bool is_start_endpoint,
  bool project_by_xy,
  unsigned int & selected_id)
{
  const double inscribed_radius =
    perception_3d_ros_->getGlobalUtils()->getInscribedRadius();
  double best_xy_distance = std::numeric_limits<double>::infinity();
  double best_z_distance = std::numeric_limits<double>::infinity();
  double best_squared_distance = std::numeric_limits<double>::infinity();
  int best_id = -1;
  int dynamic_escape_id = -1;
  double dynamic_escape_xy_distance = std::numeric_limits<double>::infinity();
  double dynamic_escape_z_distance = std::numeric_limits<double>::infinity();
  double dynamic_escape_squared_distance =
    std::numeric_limits<double>::infinity();
  double dynamic_escape_aggregate_clearance = 0.0;
  double dynamic_escape_static_clearance = 0.0;
  int nearest_id = -1;
  double nearest_squared_distance = std::numeric_limits<double>::infinity();
  double nearest_clearance = 0.0;
  std::size_t bounded_candidate_count = 0U;
  int best_clearance_id = -1;
  double best_bounded_clearance = -std::numeric_limits<double>::infinity();

  std::vector<int> candidate_ids;
  VerticalSurfaceBand projected_surface_band{};
  if (project_by_xy) {
    std::vector<double> candidate_z_values;
    candidate_ids.reserve(pcl_ground_->points.size());
    candidate_z_values.reserve(pcl_ground_->points.size());
    for (std::size_t index = 0U; index < pcl_ground_->points.size(); ++index) {
      const auto & candidate = pcl_ground_->points[index];
      const double xy_distance = std::hypot(
        static_cast<double>(candidate.x - requested.x),
        static_cast<double>(candidate.y - requested.y));
      if (xy_distance <= max_projection_xy) {
        candidate_ids.push_back(static_cast<int>(index));
        candidate_z_values.push_back(static_cast<double>(candidate.z));
      }
    }

    const auto supported_surfaces = findSupportedVerticalSurfaceBands(
      std::move(candidate_z_values),
      goal_projection_surface_max_z_gap_,
      static_cast<std::size_t>(goal_projection_min_surface_points_));
    if (supported_surfaces.empty()) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s 2-D projection found no supported ground surface within XY %.2f m "
        "(%zu raw candidates; minimum %d points).",
        endpoint_name, max_projection_xy, candidate_ids.size(),
        goal_projection_min_surface_points_);
      return false;
    }
    if (supported_surfaces.size() != 1U) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s 2-D projection is ambiguous: %zu ground surfaces lie within XY "
        "%.2f m (Z spans %.3f to %.3f m); send an explicit 3-D goal instead.",
        endpoint_name, supported_surfaces.size(), max_projection_xy,
        supported_surfaces.front().min_z,
        supported_surfaces.back().max_z);
      return false;
    }
    projected_surface_band = supported_surfaces.front();
  } else {
    const double projection_search_radius =
      std::hypot(max_projection_xy, max_projection_z);
    const double effective_search_radius =
      std::max(search_radius, projection_search_radius);
    std::vector<float> candidate_squared_distances;
    kdtree_ground_->radiusSearch(
      requested, effective_search_radius,
      candidate_ids, candidate_squared_distances);
  }

  for (const int candidate_id : candidate_ids) {
    const auto & candidate = pcl_ground_->points[candidate_id];
    if (
      project_by_xy &&
      !isInsideVerticalSurfaceBand(candidate.z, projected_surface_band))
    {
      continue;
    }
    const double dx = static_cast<double>(candidate.x - requested.x);
    const double dy = static_cast<double>(candidate.y - requested.y);
    const double dz = static_cast<double>(candidate.z - requested.z);
    const double xy_distance = std::hypot(dx, dy);
    const double z_distance = std::abs(dz);
    const double squared_distance = dx * dx + dy * dy + dz * dz;
    const double selection_squared_distance =
      project_by_xy ? dx * dx + dy * dy : squared_distance;
    const double clearance =
      perception_3d_ros_->get_min_dGraphValue(candidate_id);
    double static_clearance = 0.0;
    bool static_clearance_available = false;
    if (
      project_start_goal_to_traversable_ground_ &&
      is_start_endpoint &&
      allow_start_in_dynamic_inflation_ &&
      clearance < inscribed_radius)
    {
      static_clearance_available =
        perception_3d_ros_->get_layer_dGraphValue(
        start_static_clearance_layer_,
        static_cast<unsigned int>(candidate_id),
        static_clearance);
    }
    const EndpointClearanceDecision clearance_decision =
      evaluateEndpointClearance(
      project_start_goal_to_traversable_ground_,
      is_start_endpoint,
      allow_start_in_dynamic_inflation_,
      clearance,
      inscribed_radius,
      static_clearance_available,
      static_clearance);

    if (selection_squared_distance < nearest_squared_distance) {
      nearest_squared_distance = selection_squared_distance;
      nearest_id = candidate_id;
      nearest_clearance = clearance;
    }

    if (
      xy_distance > max_projection_xy ||
      (!project_by_xy && z_distance > max_projection_z))
    {
      continue;
    }
    ++bounded_candidate_count;
    if (clearance > best_bounded_clearance) {
      best_bounded_clearance = clearance;
      best_clearance_id = candidate_id;
    }
    if (!clearance_decision.traversable) {
      continue;
    }

    // A navigation goal's Z commonly comes from RViz's fixed plane rather
    // than the mapped floor. Prefer the closest horizontal ground sample;
    // use vertical distance only as the tie-breaker.
    if (clearance_decision.uses_dynamic_start_exception) {
      if (
        xy_distance < dynamic_escape_xy_distance ||
        (xy_distance == dynamic_escape_xy_distance &&
        z_distance < dynamic_escape_z_distance))
      {
        dynamic_escape_xy_distance = xy_distance;
        dynamic_escape_z_distance = z_distance;
        dynamic_escape_squared_distance = selection_squared_distance;
        dynamic_escape_aggregate_clearance = clearance;
        dynamic_escape_static_clearance = static_clearance;
        dynamic_escape_id = candidate_id;
      }
      continue;
    }

    if (
      xy_distance < best_xy_distance ||
      (xy_distance == best_xy_distance && z_distance < best_z_distance))
    {
      best_xy_distance = xy_distance;
      best_z_distance = z_distance;
      best_squared_distance = selection_squared_distance;
      best_id = candidate_id;
    }
  }

  bool used_dynamic_start_exception = false;
  if (best_id < 0 && dynamic_escape_id >= 0) {
    best_id = dynamic_escape_id;
    best_xy_distance = dynamic_escape_xy_distance;
    best_z_distance = dynamic_escape_z_distance;
    best_squared_distance = dynamic_escape_squared_distance;
    used_dynamic_start_exception = true;
  }

  if (nearest_id < 0) {
    if (project_by_xy) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s has no mapground candidate on the selected surface within XY "
        "%.2f m.", endpoint_name, max_projection_xy);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "%s has no mapground candidate within XY %.2f m / Z %.2f m.",
        endpoint_name, max_projection_xy, max_projection_z);
    }
    return false;
  }

  if (best_id < 0) {
    const auto & nearest = pcl_ground_->points[nearest_id];
    const double nearest_xy = std::hypot(
      static_cast<double>(nearest.x - requested.x),
      static_cast<double>(nearest.y - requested.y));
    const double nearest_z =
      std::abs(static_cast<double>(nearest.z - requested.z));
    if (project_by_xy) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s has no traversable mapground candidate within XY %.2f m on "
        "surface Z [%.3f, %.3f] m; %zu candidates, best id=%d clearance="
        "%.3f m (required %.3f m).",
        endpoint_name, max_projection_xy,
        projected_surface_band.min_z, projected_surface_band.max_z,
        bounded_candidate_count, best_clearance_id,
        best_bounded_clearance, inscribed_radius);
    } else if (bounded_candidate_count == 0U) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s has no mapground candidate within XY %.2f m / Z %.2f m; "
        "closest searched id=%d is at XY %.3f m / Z %.3f m.",
        endpoint_name, max_projection_xy, max_projection_z,
        nearest_id, nearest_xy, nearest_z);
    } else {
      RCLCPP_WARN(
        this->get_logger(),
        "%s has no traversable mapground candidate within XY %.2f m / Z %.2f m; "
        "%zu bounded candidates, best id=%d clearance=%.3f m (required %.3f m).",
        endpoint_name, max_projection_xy, max_projection_z,
        bounded_candidate_count, best_clearance_id,
        best_bounded_clearance, inscribed_radius);
    }
    return false;
  }

  selected_id = static_cast<unsigned int>(best_id);
  const auto & selected = pcl_ground_->points[selected_id];
  const double projection_xy = std::hypot(
    static_cast<double>(selected.x - requested.x),
    static_cast<double>(selected.y - requested.y));
  const double projection_z =
    std::abs(static_cast<double>(selected.z - requested.z));
  const double selected_clearance =
    perception_3d_ros_->get_min_dGraphValue(selected_id);
  const bool used_projection_search =
    std::sqrt(best_squared_distance) > search_radius;
  if (project_by_xy) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *clock_, 5000,
      "%s 2-D XY projection ignored requested Z %.3f m and selected id=%u "
      "at (%.3f, %.3f, %.3f), XY offset %.3f m, clearance %.3f m.",
      endpoint_name, requested.z, selected_id,
      selected.x, selected.y, selected.z,
      projection_xy, selected_clearance);
  } else if (used_dynamic_start_exception) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "%s is inside live-obstacle inflation (aggregate clearance %.3f m), "
      "but static layer '%s' is clear at %.3f m; allowing only this tightly "
      "bounded start node so A* and the local collision critics can plan an "
      "escape.",
      endpoint_name, dynamic_escape_aggregate_clearance,
      start_static_clearance_layer_.c_str(),
      dynamic_escape_static_clearance);
  } else if (best_id != nearest_id) {
    const auto & nearest = pcl_ground_->points[nearest_id];
    const double nearest_xy = std::hypot(
      static_cast<double>(nearest.x - requested.x),
      static_cast<double>(nearest.y - requested.y));
    const double nearest_z =
      std::abs(static_cast<double>(nearest.z - requested.z));
    if (nearest_xy > max_projection_xy || nearest_z > max_projection_z) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *clock_, 5000,
        "%s nearest 3-D ground id=%d is outside projection bounds at XY %.3f m / "
        "Z %.3f m; selected id=%u at XY %.3f m / Z %.3f m (clearance %.3f m).",
        endpoint_name, nearest_id, nearest_xy, nearest_z, selected_id,
        projection_xy, projection_z, selected_clearance);
    } else if (
      project_start_goal_to_traversable_ground_ &&
      nearest_clearance < inscribed_radius)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "%s nearest ground id=%d is not traversable (clearance %.3f m); "
        "projected to id=%u by XY %.3f m / Z %.3f m (clearance %.3f m).",
        endpoint_name, nearest_id, nearest_clearance, selected_id,
        projection_xy, projection_z, selected_clearance);
    } else if (enable_detail_log_) {
      RCLCPP_INFO(
        this->get_logger(),
        "%s selected closest-horizontal ground id=%u at XY %.3f m / Z %.3f m "
        "instead of nearest-3D id=%d (clearance %.3f m).",
        endpoint_name, selected_id, projection_xy, projection_z,
        nearest_id, selected_clearance);
    }
  } else if (used_projection_search) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *clock_, 5000,
      "%s projected to traversable mapground by XY %.3f m / Z %.3f m "
      "(clearance %.3f m).",
      endpoint_name, projection_xy, projection_z, selected_clearance);
  } else if (enable_detail_log_) {
    RCLCPP_INFO(
      this->get_logger(),
      "%s selected id=%u at (%.3f, %.3f, %.3f), projection XY %.3f m / "
      "Z %.3f m, clearance %.3f m.",
      endpoint_name, selected_id, selected.x, selected.y, selected.z,
      projection_xy, projection_z, selected_clearance);
  }
  return true;
}

bool GlobalPlanner::getStartGoalID(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool project_goal_to_ground,
  unsigned int & start_id, unsigned int & goal_id)
{
  pcl::PointXYZI pcl_start;
  pcl_start.x = start.pose.position.x;
  pcl_start.y = start.pose.position.y;
  pcl_start.z = start.pose.position.z;

  pcl::PointXYZI pcl_goal;
  pcl_goal.x = goal.pose.position.x;
  pcl_goal.y = goal.pose.position.y;
  pcl_goal.z = goal.pose.position.z;

  return
    selectTraversableGround(
      pcl_start, find_start_tolerance_,
      max_start_projection_xy_, max_start_projection_z_,
      "Start", true, false, start_id) &&
    selectTraversableGround(
      pcl_goal, find_goal_tolerance_,
      max_goal_projection_xy_, max_goal_projection_z_,
      "Goal", false, project_goal_to_ground, goal_id);
}

void GlobalPlanner::makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::GetPlan>> goal_handle){
  
  //@get goal and start
  const auto goal = goal_handle->get_goal();

  if(!perception_3d_ros_->getSharedDataPtr()->is_static_layer_ready_){
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Received the request before static layer is ready");
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    goal_handle->abort(result);
    return;
  }

  if(!goal_handle->get_goal()->activate_threading){
    auto result = std::make_shared<dddmr_sys_core::action::GetPlan::Result>();
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, "Deactivate thread");
    goal_handle->succeed(result);
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  perception_3d_ros_->getGlobalPose(start);

  auto ros_path = makeROSPlan(
    start, goal->goal, goal->project_goal_to_ground);

  if(ros_path.poses.empty()){
    global_plan_result_->path = ros_path;
    goal_handle->abort(global_plan_result_);
  }
  else{
    //postSmoothPath(path, smoothed_path);
    pub_path_->publish(ros_path);
    global_plan_result_->path = ros_path;
    goal_handle->succeed(global_plan_result_);
  }
  
}

nav_msgs::msg::Path GlobalPlanner::makeROSPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  bool project_goal_to_ground)
{
  
  std::unique_lock<std::mutex> lock(protect_kdtree_ground_);
  unsigned int start_id = 0;
  unsigned int goal_id = 0;
  std::vector<unsigned int> path;
  std::vector<unsigned int> smoothed_path;
  std::vector<unsigned int> smoothed_path_2nd;
  nav_msgs::msg::Path ros_path;

  const bool endpoints_found = getStartGoalID(
    start, goal, project_goal_to_ground, start_id, goal_id);
  if(endpoints_found){
    if(!use_pre_graph_)
      a_star_planner_->getPath(start_id, goal_id, path);
    else
      a_star_planner_pre_graph_->getPath(start_id, goal_id, path);  
  }

  if(!endpoints_found){
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *clock_, 5000,
      "No path: a traversable start or goal was not found.");
    return ros_path;
  }

  if(path.empty()){
    if(enable_detail_log_)
      RCLCPP_WARN(this->get_logger(), "No path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "No path found from: %u to %u", start_id, goal_id);
    return ros_path;
  }
  else{
    if(enable_detail_log_)
      RCLCPP_INFO(this->get_logger(), "Path found from: %u to %u", start_id, goal_id);
    else
      RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 5000, "Path found from: %u to %u", start_id, goal_id);
    getROSPath(path, ros_path);
    const auto & requested_orientation = goal.pose.orientation;
    const double orientation_norm = std::sqrt(
      requested_orientation.x * requested_orientation.x +
      requested_orientation.y * requested_orientation.y +
      requested_orientation.z * requested_orientation.z +
      requested_orientation.w * requested_orientation.w);
    if (!ros_path.poses.empty() && orientation_norm > 1e-6) {
      // Keep the selected traversable ground position. Appending the raw goal
      // can recreate a final segment into the obstacle that caused projection.
      ros_path.poses.back().pose.orientation = requested_orientation;
    }
    return ros_path;
  }
}

void GlobalPlanner::getStaticGraphFromPerception3D(){
  
  //@Calculate node weight
  
  if(!has_initialized_){
    has_initialized_ = true;
    if(a_star_expanding_radius_ >= perception_3d_ros_->getGlobalUtils()->getInscribedRadius()*2){
      RCLCPP_WARN(this->get_logger(), "Expanding radius is much larger than InscribedRadius, the planning time will be increased.");
    }
    if(!use_pre_graph_){
      a_star_planner_ = std::make_shared<A_Star_on_Graph>(pcl_ground_, perception_3d_ros_, a_star_expanding_radius_);
      a_star_planner_->setupTurningWeight(turning_weight_);
      a_star_planner_->setupMaximumGroundConnectionZ(maximum_ground_connection_z_);
    }
    else{
      a_star_planner_pre_graph_ = std::make_shared<A_Star_on_PreGraph>(pcl_ground_, static_graph_, perception_3d_ros_, a_star_expanding_radius_);
      a_star_planner_pre_graph_->setupTurningWeight(turning_weight_);
      a_star_planner_pre_graph_->setupMaximumGroundConnectionZ(maximum_ground_connection_z_);
    }
  }
  else{
    if (!use_pre_graph_) {
      a_star_planner_->updateGraph(pcl_ground_);
    } else {
      a_star_planner_pre_graph_->updateGraph(pcl_ground_, static_graph_);
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
  
  unsigned long node_weight_size = perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_->getNodeWeightSize();
  bool is_node_weight = true;
  if(node_weight_size<=0){
    node_weight_size = perception_3d_ros_->getSharedDataPtr()->sGraph_ptr_->getSize();
    is_node_weight = false;
  }

  for(auto it=0; it<node_weight_size; it++){

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
