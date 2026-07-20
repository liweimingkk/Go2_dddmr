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
#include <perception_3d/static_layer.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

PLUGINLIB_EXPORT_CLASS(perception_3d::StaticLayer, perception_3d::Sensor)

namespace perception_3d
{

namespace
{

std::string normalizeFrameId(std::string frame_id)
{
  while (!frame_id.empty() && frame_id.front() == '/') {
    frame_id.erase(frame_id.begin());
  }
  return frame_id;
}

}  // namespace

StaticLayer::StaticLayer(){
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

StaticLayer::~StaticLayer(){

}

void StaticLayer::onInitialize()
{ 
  
  ptrInitial();

  cbs_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;

  node_->declare_parameter(name_ + ".radius_of_ground_connection", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".radius_of_ground_connection", radius_of_ground_connection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "radius_of_ground_connection: %.2f", radius_of_ground_connection_);

  node_->declare_parameter(name_ + ".use_adaptive_connection", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".use_adaptive_connection", use_adaptive_connection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "use_adaptive_connection: %d", use_adaptive_connection_);  
  
  node_->declare_parameter(name_ + ".adaptive_connection_number", rclcpp::ParameterValue(20));
  node_->get_parameter(name_ + ".adaptive_connection_number", adaptive_connection_number_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "adaptive_connection_number: %d", adaptive_connection_number_);  
  
  node_->declare_parameter(name_ + ".turning_weight", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".turning_weight", turning_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "turning_weight: %.2f", turning_weight_);    

  node_->declare_parameter(name_ + ".intensity_search_radius", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".intensity_search_radius", intensity_search_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "intensity_search_radius: %.2f", intensity_search_radius_);    

  node_->declare_parameter(name_ + ".intensity_search_punish_weight", rclcpp::ParameterValue(0.1));
  node_->get_parameter(name_ + ".intensity_search_punish_weight", intensity_search_punish_weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "intensity_search_punish_weight: %.2f", intensity_search_punish_weight_);    

  node_->declare_parameter(name_ + ".static_imposing_radius", rclcpp::ParameterValue(0.25));
  node_->get_parameter(name_ + ".static_imposing_radius", static_imposing_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "static_imposing_radius: %.2f", static_imposing_radius_);    

  node_->declare_parameter(name_ + ".static_obstacle_min_points", rclcpp::ParameterValue(11));
  node_->get_parameter(name_ + ".static_obstacle_min_points", static_obstacle_min_points_);
  if(static_obstacle_min_points_ < 1){
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "static_obstacle_min_points must be positive: %d", static_obstacle_min_points_);
    throw std::invalid_argument("static_obstacle_min_points must be positive");
  }
  RCLCPP_INFO(node_->get_logger().get_child(name_), "static_obstacle_min_points: %d", static_obstacle_min_points_);

  node_->declare_parameter(name_ + ".static_obstacle_xy_radius", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name_ + ".static_obstacle_xy_radius", static_obstacle_xy_radius_);
  if(static_obstacle_xy_radius_ <= 0.0){
    RCLCPP_FATAL(node_->get_logger().get_child(name_), "static_obstacle_xy_radius must be positive: %.3f", static_obstacle_xy_radius_);
    throw std::invalid_argument("static_obstacle_xy_radius must be positive");
  }
  RCLCPP_INFO(node_->get_logger().get_child(name_), "static_obstacle_xy_radius: %.2f", static_obstacle_xy_radius_);

  const std::string stair_param = name_ + ".traversed_stair_clearance.";
  node_->declare_parameter(stair_param + "enabled", rclcpp::ParameterValue(false));
  node_->get_parameter(stair_param + "enabled", traversed_stair_clearance_enabled_);

  node_->declare_parameter(
    stair_param + "trajectory_topic", rclcpp::ParameterValue("map1/key_poses"));
  node_->get_parameter(
    stair_param + "trajectory_topic", traversed_stair_trajectory_topic_);

  node_->declare_parameter(
    stair_param + "minimum_pose_count", rclcpp::ParameterValue(6));
  node_->get_parameter(
    stair_param + "minimum_pose_count", traversed_stair_minimum_pose_count_);

  node_->declare_parameter(
    stair_param + "minimum_total_height_change", rclcpp::ParameterValue(0.5));
  node_->get_parameter(
    stair_param + "minimum_total_height_change",
    traversed_stair_minimum_total_height_change_);

  node_->declare_parameter(
    stair_param + "xy_radius", rclcpp::ParameterValue(0.10));
  node_->get_parameter(stair_param + "xy_radius", traversed_stair_xy_radius_);

  node_->declare_parameter(
    stair_param + "minimum_relative_z", rclcpp::ParameterValue(-0.50));
  node_->get_parameter(
    stair_param + "minimum_relative_z", traversed_stair_minimum_relative_z_);

  node_->declare_parameter(
    stair_param + "maximum_relative_z", rclcpp::ParameterValue(0.05));
  node_->get_parameter(
    stair_param + "maximum_relative_z", traversed_stair_maximum_relative_z_);

  if(traversed_stair_minimum_pose_count_ < 2){
    RCLCPP_FATAL(
      node_->get_logger().get_child(name_),
      "traversed_stair_clearance.minimum_pose_count must be at least 2: %d",
      traversed_stair_minimum_pose_count_);
    throw std::invalid_argument(
      "traversed_stair_clearance.minimum_pose_count must be at least 2");
  }
  if(!std::isfinite(traversed_stair_minimum_total_height_change_) ||
    traversed_stair_minimum_total_height_change_ <= 0.0){
    RCLCPP_FATAL(
      node_->get_logger().get_child(name_),
      "traversed_stair_clearance.minimum_total_height_change must be finite and positive: %.3f",
      traversed_stair_minimum_total_height_change_);
    throw std::invalid_argument(
      "traversed_stair_clearance.minimum_total_height_change must be finite and positive");
  }
  if(!std::isfinite(traversed_stair_xy_radius_) ||
    traversed_stair_xy_radius_ <= 0.0 || traversed_stair_xy_radius_ > 0.10){
    RCLCPP_FATAL(
      node_->get_logger().get_child(name_),
      "traversed_stair_clearance.xy_radius must be finite and in (0.0, 0.10]: %.3f",
      traversed_stair_xy_radius_);
    throw std::invalid_argument(
      "traversed_stair_clearance.xy_radius must be finite and no greater than 0.10 m");
  }
  if(!std::isfinite(traversed_stair_minimum_relative_z_) ||
    !std::isfinite(traversed_stair_maximum_relative_z_) ||
    traversed_stair_minimum_relative_z_ < -0.50 ||
    traversed_stair_maximum_relative_z_ > 0.05 ||
    traversed_stair_minimum_relative_z_ >= traversed_stair_maximum_relative_z_){
    RCLCPP_FATAL(
      node_->get_logger().get_child(name_),
      "traversed_stair_clearance relative-z limits must be finite, ordered, and "
      "inside [-0.50, 0.05]: [%.3f, %.3f]",
      traversed_stair_minimum_relative_z_, traversed_stair_maximum_relative_z_);
    throw std::invalid_argument(
      "traversed_stair_clearance relative-z limits are outside the safety envelope");
  }
  if(traversed_stair_clearance_enabled_ &&
    traversed_stair_trajectory_topic_.empty()){
    RCLCPP_FATAL(
      node_->get_logger().get_child(name_),
      "traversed_stair_clearance.trajectory_topic cannot be empty when enabled");
    throw std::invalid_argument(
      "traversed_stair_clearance.trajectory_topic cannot be empty when enabled");
  }
  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "traversed_stair_clearance: %s, trajectory_topic: %s, minimum poses: %d, "
    "minimum height change: %.2f m, XY radius: %.2f m, relative z: "
    "[%.2f, %.2f] m",
    traversed_stair_clearance_enabled_ ? "enabled" : "disabled",
    traversed_stair_trajectory_topic_.c_str(),
    traversed_stair_minimum_pose_count_,
    traversed_stair_minimum_total_height_change_, traversed_stair_xy_radius_,
    traversed_stair_minimum_relative_z_, traversed_stair_maximum_relative_z_);

  node_->declare_parameter(name_ + ".is_local_planner", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".is_local_planner", is_local_planner_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "is_local_planner: %d", is_local_planner_);      

  node_->declare_parameter(name_ + ".map_topic", rclcpp::ParameterValue("mapcloud"));
  node_->get_parameter(name_ + ".map_topic", map_topic_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "map_topic: %s", map_topic_.c_str());     

  node_->declare_parameter(name_ + ".ground_topic", rclcpp::ParameterValue("mapground"));
  node_->get_parameter(name_ + ".ground_topic", ground_topic_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "ground_topic: %s", ground_topic_.c_str());     

  node_->declare_parameter(name_ + ".support.mapping_mode", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".support.mapping_mode", mapping_mode_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "mapping_mode: %d", mapping_mode_);     
  
  node_->declare_parameter(name_ + ".support.enable_edge_detection", rclcpp::ParameterValue(true));
  node_->get_parameter(name_ + ".support.enable_edge_detection", enable_edge_detection_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "enable_edge_detection: %d", enable_edge_detection_);     

  node_->declare_parameter(name_ + ".support.generate_static_graph", rclcpp::ParameterValue(false));
  node_->get_parameter(name_ + ".support.generate_static_graph", generate_static_graph_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "generate_static_graph: %d", generate_static_graph_);     

  
  shared_data_->mapping_mode_ = mapping_mode_;
  
  const std::string dgraph_topic = std::string{node_->get_name()} + "/" + name_ + "/dGraph";
  pub_dGraph_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(dgraph_topic, 2);
  
  if(mapping_mode_){
    pcl_map_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(), 
      std::bind(&StaticLayer::cbMap, this, std::placeholders::_1), sub_options);

    pcl_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort(), 
      std::bind(&StaticLayer::cbGround, this, std::placeholders::_1), sub_options);
  }
  else{
    pcl_map_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&StaticLayer::cbMap, this, std::placeholders::_1), sub_options);

    pcl_ground_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
      ground_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(), 
      std::bind(&StaticLayer::cbGround, this, std::placeholders::_1), sub_options);
  }

  if(traversed_stair_clearance_enabled_){
    traversed_stair_trajectory_sub_ =
      node_->create_subscription<geometry_msgs::msg::PoseArray>(
        traversed_stair_trajectory_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(
          &StaticLayer::cbTraversedStairTrajectory, this,
          std::placeholders::_1),
        sub_options);
  }


}

void StaticLayer::ptrInitial(){
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->sGraph_ptr_ = std::make_shared<perception_3d::StaticGraph>();
  new_map_ = new_ground_ = is_local_planner_ = false;
  is_ground_and_map_being_initialized_once_ = false;
  traversed_stair_clearance_enabled_ = false;
  traversed_stair_trajectory_valid_ = false;
  new_traversed_stair_trajectory_ = false;
  traversed_stair_trajectory_height_change_ = 0.0;
  traversed_stair_trajectory_.clear();
  traversed_stair_ground_mask_.clear();
}

void StaticLayer::cbMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  /*transform to point cloud library format first so we can leverage PCL*/
  pcl::fromROSMsg(*msg, *pcl_map_);
  // remove NaN
  std::vector<int> indices;
  pcl_map_->is_dense = false;
  pcl::removeNaNFromPointCloud(*pcl_map_, *pcl_map_, indices);

  if(shared_data_->static_map_size_!=pcl_map_->points.size()){
    new_map_ = true;
    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s receive new \033[1;32mMap\033[0m with size: %lu", name_.c_str(), pcl_map_->points.size());
  }
  shared_data_->static_map_size_ = pcl_map_->points.size();
  shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared_data_->kdtree_map_->setInputCloud(pcl_map_);
  shared_data_->pcl_map_ = pcl_map_;
}

void StaticLayer::cbGround(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  pcl::fromROSMsg(*msg, *pcl_ground_);  
  // remove NaN
  std::vector<int> indices;
  pcl_ground_->is_dense = false;
  pcl::removeNaNFromPointCloud(*pcl_ground_, *pcl_ground_, indices);

  if(shared_data_->static_ground_size_!=pcl_ground_->points.size()){
    new_ground_ = true;
    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s receive new \033[1;32mGround\033[0m with size: %lu", name_.c_str(), pcl_ground_->points.size());
  }  
  shared_data_->static_ground_size_ = pcl_ground_->points.size();
  shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared_data_->kdtree_ground_->setInputCloud(pcl_ground_);
  shared_data_->pcl_ground_ = pcl_ground_;
 
}

void StaticLayer::cbTraversedStairTrajectory(
  const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
  std::unique_lock<std::recursive_mutex> lock(
    shared_data_->ground_kdtree_cb_mutex_);

  const auto reject_trajectory = [this](const std::string & reason) {
      traversed_stair_trajectory_.clear();
      traversed_stair_ground_mask_.clear();
      traversed_stair_trajectory_valid_ = false;
      new_traversed_stair_trajectory_ = true;
      traversed_stair_trajectory_height_change_ = 0.0;
      is_ground_and_map_being_initialized_once_ = false;
      shared_data_->is_static_layer_ready_ = false;
      RCLCPP_WARN(
        node_->get_logger().get_child(name_),
        "Rejected traversed stair trajectory; static layer remains not ready: %s",
        reason.c_str());
    };

  const std::string trajectory_frame = normalizeFrameId(msg->header.frame_id);
  const std::string global_frame = normalizeFrameId(gbl_utils_->getGblFrame());
  if(trajectory_frame.empty() || trajectory_frame != global_frame){
    reject_trajectory(
      "PoseArray frame '" + msg->header.frame_id +
      "' does not match global frame '" + gbl_utils_->getGblFrame() + "'");
    return;
  }
  if(msg->poses.size() <
    static_cast<std::size_t>(traversed_stair_minimum_pose_count_)){
    reject_trajectory(
      "pose count " + std::to_string(msg->poses.size()) +
      " is below minimum " +
      std::to_string(traversed_stair_minimum_pose_count_));
    return;
  }

  double minimum_z = std::numeric_limits<double>::infinity();
  double maximum_z = -std::numeric_limits<double>::infinity();
  for(std::size_t index = 0; index < msg->poses.size(); ++index){
    const auto & position = msg->poses[index].position;
    if(!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z)){
      reject_trajectory(
        "pose " + std::to_string(index) + " contains a non-finite position");
      return;
    }
    if(position.z < minimum_z){
      minimum_z = position.z;
    }
    if(position.z > maximum_z){
      maximum_z = position.z;
    }
  }

  const double height_change = maximum_z - minimum_z;
  if(!std::isfinite(height_change) ||
    height_change < traversed_stair_minimum_total_height_change_){
    reject_trajectory(
      "height change " + std::to_string(height_change) +
      " m is below minimum " +
      std::to_string(traversed_stair_minimum_total_height_change_) + " m");
    return;
  }

  // A PoseArray that passes the height-change check is a recording of the
  // route the robot actually traversed. Keep its landing poses as well as the
  // stair flight: extrema-only trimming discarded valid approach nodes and
  // made the trusted corridor discontinuous.
  traversed_stair_trajectory_.clear();
  traversed_stair_trajectory_.reserve(msg->poses.size());
  for(const auto & pose : msg->poses){
    traversed_stair_trajectory_.push_back(pose.position);
  }
  traversed_stair_trajectory_height_change_ = height_change;
  traversed_stair_trajectory_valid_ = true;
  new_traversed_stair_trajectory_ = true;
  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "Accepted traversed stair trajectory: %zu trusted poses, height change %.3f m",
    traversed_stair_trajectory_.size(),
    traversed_stair_trajectory_height_change_);
}

bool StaticLayer::buildTraversedStairGroundMask()
{
  if(!traversed_stair_clearance_enabled_){
    traversed_stair_ground_mask_.clear();
    return true;
  }
  if(!traversed_stair_trajectory_valid_ ||
    traversed_stair_trajectory_.size() <
    static_cast<std::size_t>(traversed_stair_minimum_pose_count_) ||
    traversed_stair_trajectory_height_change_ <
    traversed_stair_minimum_total_height_change_){
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_),
      "Cannot build traversed stair mask from an untrusted trajectory");
    traversed_stair_ground_mask_.clear();
    return false;
  }

  // Exempt the narrow set of ground graph nodes physically traversed by the
  // robot. Masking mapcloud points was both geometrically mismatched (the map
  // stores obstacle surfaces, not support nodes) and ineffective: the saved
  // stair map produced 0/4303 matches. Dynamic lidar layers are unaffected.
  traversed_stair_ground_mask_.assign(pcl_ground_->points.size(), 0U);
  const double squared_xy_radius =
    traversed_stair_xy_radius_ * traversed_stair_xy_radius_;
  std::size_t masked_ground_count = 0;
  for(std::size_t ground_index = 0;
    ground_index < pcl_ground_->points.size(); ++ground_index){
    const auto & ground_point = pcl_ground_->points[ground_index];
    if(!std::isfinite(ground_point.x) || !std::isfinite(ground_point.y) ||
      !std::isfinite(ground_point.z)){
      continue;
    }
    for(const auto & trajectory_position : traversed_stair_trajectory_){
      const double relative_z =
        static_cast<double>(ground_point.z) - trajectory_position.z;
      if(relative_z < traversed_stair_minimum_relative_z_ ||
        relative_z > traversed_stair_maximum_relative_z_){
        continue;
      }
      const double dx =
        static_cast<double>(ground_point.x) - trajectory_position.x;
      const double dy =
        static_cast<double>(ground_point.y) - trajectory_position.y;
      if(dx * dx + dy * dy <= squared_xy_radius){
        traversed_stair_ground_mask_[ground_index] = 1U;
        ++masked_ground_count;
        break;
      }
    }
  }

  if(masked_ground_count == 0U){
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_),
      "Traversed stair trusted-ground mask matched 0/%zu ground nodes; "
      "refusing to mark the static layer ready",
      pcl_ground_->points.size());
    traversed_stair_ground_mask_.clear();
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger().get_child(name_),
    "Traversed stair trusted-ground mask: %zu/%zu ground nodes exempted from "
    "static-map voting only; %zu poses, height change %.3f m, XY radius %.3f m, "
    "relative z [%.3f, %.3f] m",
    masked_ground_count, pcl_ground_->points.size(),
    traversed_stair_trajectory_.size(),
    traversed_stair_trajectory_height_change_, traversed_stair_xy_radius_,
    traversed_stair_minimum_relative_z_, traversed_stair_maximum_relative_z_);
  return true;
}

void StaticLayer::updateLethalPointCloud(){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  current_lethal_->clear();
  current_lethal_->header.frame_id = gbl_utils_->getGblFrame();

  if(!is_ground_and_map_being_initialized_once_ || is_local_planner_){
    return;
  }

  const double inscribed_radius = gbl_utils_->getInscribedRadius();
  for(unsigned int index = 0; index < pcl_ground_->points.size(); index++){
    if(dGraph_.getValue(index) < inscribed_radius){
      current_lethal_->push_back(pcl_ground_->points[index]);
    }
  }
}

void StaticLayer::selfMark(){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if(pub_dGraph_->get_subscription_count()>0){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg2 (new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t index=0;index<shared_data_->static_ground_size_;index++){
      pcl::PointXYZI ipt;
      ipt.x = shared_data_->pcl_ground_->points[index].x;
      ipt.y = shared_data_->pcl_ground_->points[index].y;
      ipt.z = shared_data_->pcl_ground_->points[index].z;   
      ipt.intensity = get_dGraphValue(index);
      pcl_msg2->push_back(ipt);
    }
    sensor_msgs::msg::PointCloud2 ros_pc2_msg2;
    pcl_msg2->header.frame_id = gbl_utils_->getGblFrame();
    pcl::toROSMsg(*pcl_msg2, ros_pc2_msg2);
    pub_dGraph_->publish(ros_pc2_msg2);
  }
}

void StaticLayer::selfClear(){

  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  const bool trajectory_ready =
    !traversed_stair_clearance_enabled_ ||
    traversed_stair_trajectory_valid_;
  const bool map_and_ground_updated = new_ground_ && new_map_;
  const bool trajectory_updated_with_map =
    traversed_stair_clearance_enabled_ &&
    new_traversed_stair_trajectory_ &&
    !pcl_ground_->empty() && !pcl_map_->empty();
  if((map_and_ground_updated || trajectory_updated_with_map) && trajectory_ready){

    if(!buildTraversedStairGroundMask()){
      is_ground_and_map_being_initialized_once_ = false;
      shared_data_->is_static_layer_ready_ = false;
      return;
    }

    RCLCPP_WARN(node_->get_logger().get_child(name_), "%s has already received two msg.", name_.c_str());
    shared_data_->requestAllLayersToResetDGraph();

    shared_data_->dgraph_update_request_[name_] = false;
    new_ground_ = false;
    new_map_ = false;
    new_traversed_stair_trajectory_ = false;

    //@ radius search connection to generate sGraph
    resetdGraph();
    if(!is_local_planner_){
      //@ it is static layer of a global planner
      shared_data_->sGraph_ptr_->allocateGraph(pcl_ground_->points.size());
      if(!mapping_mode_ && enable_edge_detection_){
        radiusSearchConnection();
        RCLCPP_INFO(node_->get_logger().get_child(name_), "Computation of Edge from ground point cloud is done.");
      }
      if(generate_static_graph_){
        generateStaticGraph();
        RCLCPP_INFO(node_->get_logger().get_child(name_), "Static graph is generated with graph size: %lu", shared_data_->sGraph_ptr_->getSize());
      }
    }

    shared_data_->is_static_layer_ready_ = true;
    is_ground_and_map_being_initialized_once_ = true;
  }
  else{ //@ disable ready flag when state is different
    if(!trajectory_ready){
      is_ground_and_map_being_initialized_once_ = false;
      shared_data_->is_static_layer_ready_ = false;
    }
    else if(!is_ground_and_map_being_initialized_once_)
      shared_data_->is_static_layer_ready_ = false;
  }

}

void StaticLayer::generateStaticGraph(){
  
  #pragma omp parallel for
  for(unsigned int index_cnt = 0; index_cnt<pcl_ground_->points.size(); index_cnt++){

    pcl::PointXYZI pcl_node = pcl_ground_->points[index_cnt];

    //@Kd-tree to find nn point for planar equation
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(!use_adaptive_connection_){
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, radius_of_ground_connection_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    }
    else{

      int hard_interrupt_cnt = 100;
      float search_r = 0.5;
      int search_cnt = 1;
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      while(pointIdxRadiusSearch.size()<adaptive_connection_number_ && hard_interrupt_cnt>0){
        search_cnt++;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);    
        hard_interrupt_cnt--;   
      }
    }
    
    for(auto it = pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end();it++){
      //chekc relative z value for the edge, because we need to eliminate stair and wheel chair passage issue
      edge_t a_edge;
      auto node = index_cnt;
      a_edge.first = (*it);
      //@Create an edge
      double distance_between_pair = sqrt(pcl::geometry::squaredDistance(pcl_ground_->points[node], pcl_ground_->points[a_edge.first]));
      a_edge.second = distance_between_pair;
      shared_data_->sGraph_ptr_->insertEdgeInNode(node, a_edge);

    }

    const bool trusted_stair_ground =
      traversed_stair_clearance_enabled_ &&
      traversed_stair_trajectory_valid_ &&
      index_cnt < traversed_stair_ground_mask_.size() &&
      traversed_stair_ground_mask_[index_cnt] != 0U;
    if(!trusted_stair_ground){
      //@ use map to impose weight on each node
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      if(shared_data_->kdtree_map_->nearestKSearch(
        pcl_node, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0){
        double distance_to_obstacle = sqrt(pointRadiusSquaredDistance[0]);
        if(distance_to_obstacle < gbl_utils_->getInscribedRadius())
          dGraph_.setValue(index_cnt, distance_to_obstacle);
      }
    }
  }
}

void StaticLayer::radiusSearchConnection(){
  
  // 1. Determine total loop size
  const unsigned int total_points = pcl_ground_->points.size();
  std::size_t static_obstacle_nodes = 0;
  std::size_t static_lethal_nodes = 0;
  std::size_t trusted_stair_ground_nodes = 0;

  // 2. OpenMP Parallel For Loop
  // We specify that 'index_cnt' is the loop variable (implicitly private)
  #pragma omp parallel for reduction(+:static_obstacle_nodes,static_lethal_nodes,trusted_stair_ground_nodes)
  for(unsigned int index_cnt = 0; index_cnt < total_points; index_cnt++){
    
    pcl::PointXYZI pcl_node = pcl_ground_->points[index_cnt];

    //@Kd-tree to find nn point for planar equation
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    if(!use_adaptive_connection_){
      // kdtree radiusSearch is thread-safe for concurrent reads
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, radius_of_ground_connection_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
    }
    else{
      int hard_interrupt_cnt = 100;
      float search_r = 0.5;
      int search_cnt = 1;
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);

      while(pointIdxRadiusSearch.size()<adaptive_connection_number_ && hard_interrupt_cnt>0){
        search_cnt++;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        shared_data_->kdtree_ground_->radiusSearch (pcl_node, search_r + 0.2*search_cnt, pointIdxRadiusSearch, pointRadiusSquaredDistance);    
        hard_interrupt_cnt--;   
      }
    }
    
    // Allocating objects inside the loop ensures they are thread-local (private)
    pcl::PointCloud<pcl::PointXYZI>::Ptr nn_pc (new pcl::PointCloud<pcl::PointXYZI>);
    for(auto it = pointIdxRadiusSearch.begin(); it!=pointIdxRadiusSearch.end();it++){
      //@Push back the points for plane equation later
      nn_pc->push_back(pcl_ground_->points[(*it)]);
    }

    float weight = 1.0;
    float intensity_penality = 0.0;
    float max_radius = intensity_search_radius_; 
    int reject_threshold = 0;
    //@ consider this scenario to be boundary of ground
    if(nn_pc->points.size()<5){
      weight = 1000;
    }
    else{
      //@Use RANSAC to get normal
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      
      // Creating the segmentation object locally per thread
      pcl::SACSegmentation<pcl::PointXYZI> seg;
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.05); 

      seg.setInputCloud (nn_pc);
      seg.segment (*inliers, *coefficients);
      
      for(float ring_radius=max_radius; ring_radius>0; ring_radius-=0.25){
        for(float d_theta=-3.1415926; d_theta<=3.1415926; d_theta+=0.174){ //per 10 deg
          pcl::PointXYZI pcl_ring;
          pcl_ring.x = pcl_node.x + ring_radius*sin(d_theta);
          pcl_ring.y = pcl_node.y + ring_radius*cos(d_theta);
          pcl_ring.z = (-coefficients->values[3]-coefficients->values[0]*pcl_ring.x-coefficients->values[1]*pcl_ring.y)/coefficients->values[2];
          if(isinf(pcl_ring.z)){
            pcl_ring.z = 0.0;
          }
          if(std::isnan(pcl_ring.z))
            continue;
          
          std::vector<int> pointIdxRadiusSearch_ring;
          std::vector<float> pointRadiusSquaredDistance_ring;
          if(shared_data_->kdtree_ground_->radiusSearch (pcl_ring, 0.3, pointIdxRadiusSearch_ring, pointRadiusSquaredDistance_ring)<1) 
            reject_threshold++;
        }
      }
      intensity_penality += reject_threshold*intensity_search_punish_weight_;
      
      const bool trusted_stair_ground =
        traversed_stair_clearance_enabled_ &&
        traversed_stair_trajectory_valid_ &&
        index_cnt < traversed_stair_ground_mask_.size() &&
        traversed_stair_ground_mask_[index_cnt] != 0U;
      if(trusted_stair_ground){
        // Only the static map vote is bypassed. The local/global live lidar
        // layers can still mark a person, object, or changed stair as lethal.
        trusted_stair_ground_nodes++;
      }
      else{
        //@ use map to impose weight on each node
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        shared_data_->kdtree_map_->radiusSearch(
          pcl_node, static_imposing_radius_, pointIdxRadiusSearch,
          pointRadiusSquaredDistance);
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_z_axes(
          new pcl::PointCloud<pcl::PointXYZI>);
        for(const int map_index : pointIdxRadiusSearch){
          pcl_z_axes->push_back(pcl_map_->points[map_index]);
        }

        // Local Filter instantiation
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(pcl_z_axes);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(pcl_node.z + 0.1, pcl_node.z + 1.0);
        pass.filter(*pcl_z_axes);
        pass.setInputCloud(pcl_z_axes);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(
          pcl_node.x - static_obstacle_xy_radius_,
          pcl_node.x + static_obstacle_xy_radius_);
        pass.filter(*pcl_z_axes);
        pass.setInputCloud(pcl_z_axes);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(
          pcl_node.y - static_obstacle_xy_radius_,
          pcl_node.y + static_obstacle_xy_radius_);
        pass.filter(*pcl_z_axes);

        if(static_cast<int>(pcl_z_axes->points.size()) >=
          static_obstacle_min_points_){
          double min_horizontal_distance =
            std::numeric_limits<double>::infinity();
          for(const auto & obstacle_point : pcl_z_axes->points){
            min_horizontal_distance = std::min(
              min_horizontal_distance,
              std::hypot(
                static_cast<double>(obstacle_point.x - pcl_node.x),
                static_cast<double>(obstacle_point.y - pcl_node.y)));
          }
          if(std::isfinite(min_horizontal_distance)){
            // Preserve actual centerline clearance. A detected wall remains
            // lethal without forcing every classified node to a fixed value.
            dGraph_.setValue(index_cnt, min_horizontal_distance);
            static_obstacle_nodes++;
            if(min_horizontal_distance < gbl_utils_->getInscribedRadius()){
              static_lethal_nodes++;
            }
          }
        }
      }
    }
    shared_data_->sGraph_ptr_->setPenality(index_cnt, intensity_penality);
  }
  RCLCPP_INFO(node_->get_logger().get_child(name_),
    "Static obstacle ground nodes: %zu/%u classified, %zu/%u lethal "
    "(minimum map points: %d, XY radius: %.2f); trusted stair ground: %zu/%u "
    "exempted from static-map voting only",
    static_obstacle_nodes, total_points, static_lethal_nodes, total_points,
    static_obstacle_min_points_, static_obstacle_xy_radius_,
    trusted_stair_ground_nodes, total_points);
  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Static graph has been generated.");
}

void StaticLayer::resetdGraph(){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "%s starts to reset dynamic graph.", name_.c_str());
  dGraph_.clear();
  dGraph_.initial(shared_data_->static_ground_size_, gbl_utils_->getMaxObstacleDistance());
}

double StaticLayer::get_dGraphValue(const unsigned int index){
  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  if (dGraph_.graph_.find(index) == dGraph_.graph_.end())
    return 0.0;
  return dGraph_.getValue(index);
}

bool StaticLayer::isCurrent(){
  
  current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr StaticLayer::getObservation(){
  return sensor_current_observation_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr StaticLayer::getLethal(){
  return current_lethal_;
}
}//end of name space
