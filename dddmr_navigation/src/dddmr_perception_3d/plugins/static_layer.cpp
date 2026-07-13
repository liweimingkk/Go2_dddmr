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

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{

bool staticRiserSnapshotReady(
  const perception_3d::StairRiserSemanticsConfig & config,
  const perception_3d::TerrainSnapshotConstPtr & snapshot,
  std::size_t ground_size,
  std::int64_t now_nanoseconds)
{
  if (!config.enabled || !perception_3d::StairRiserSemantics::validConfig(config) ||
    !snapshot || !snapshot->valid() || snapshot->mapHash() != config.expected_map_hash ||
    snapshot->nodes().size() != ground_size ||
    now_nanoseconds < snapshot->stampNanoseconds())
  {
    return false;
  }
  return now_nanoseconds - snapshot->stampNanoseconds() <=
         config.max_snapshot_age_nanoseconds;
}

bool isExpectedStaticMapRiser(
  const perception_3d::StairRiserSemanticsConfig & config,
  const perception_3d::TerrainSnapshotConstPtr & snapshot,
  std::int64_t now_nanoseconds,
  std::size_t ground_index,
  const pcl::PointXYZI & ground_point,
  const pcl::PointXYZI & static_map_point)
{
  perception_3d::StairRiserObservation observation;
  observation.snapshot = snapshot;
  observation.terrain_ground_version = snapshot ? snapshot->version() : 0U;
  observation.now_nanoseconds = now_nanoseconds;
  observation.terrain_node_index = ground_index;
  observation.terrain_node_position = Eigen::Vector3f(
    ground_point.x, ground_point.y, ground_point.z);
  observation.obstacle_position = Eigen::Vector3f(
    static_map_point.x, static_map_point.y, static_map_point.z);
  // Only immutable mapcloud points enter this path.  Live LiDAR layers keep
  // their independent lethal dGraph values and still win the stacked minimum.
  observation.dynamic_obstacle_confirmed = false;
  return perception_3d::StairRiserSemantics::classify(config, observation).expected_riser;
}

}  // namespace

PLUGINLIB_EXPORT_CLASS(perception_3d::StaticLayer, perception_3d::Sensor)

namespace perception_3d
{

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

  const std::string stair_prefix = name_ + ".stair_riser_map_semantics.";
  node_->declare_parameter(stair_prefix + "enabled", rclcpp::ParameterValue(false));
  node_->declare_parameter(stair_prefix + "fail_closed", rclcpp::ParameterValue(true));
  node_->declare_parameter(stair_prefix + "expected_map_hash", rclcpp::ParameterValue(""));
  node_->declare_parameter(stair_prefix + "max_snapshot_age_sec", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(
    stair_prefix + "minimum_stair_confidence", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(
    stair_prefix + "max_node_match_distance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(
    stair_prefix + "riser_plane_tolerance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(
    stair_prefix + "riser_lateral_tolerance_m", rclcpp::ParameterValue(0.0));
  node_->declare_parameter(
    stair_prefix + "riser_vertical_tolerance_m", rclcpp::ParameterValue(0.0));
  node_->get_parameter(
    stair_prefix + "enabled", stair_riser_map_semantics_config_.enabled);
  node_->get_parameter(
    stair_prefix + "fail_closed", stair_riser_map_semantics_config_.fail_closed);
  node_->get_parameter(
    stair_prefix + "expected_map_hash",
    stair_riser_map_semantics_config_.expected_map_hash);
  double max_snapshot_age_sec = 0.0;
  double minimum_stair_confidence = 0.0;
  double max_node_match_distance_m = 0.0;
  double riser_plane_tolerance_m = 0.0;
  double riser_lateral_tolerance_m = 0.0;
  double riser_vertical_tolerance_m = 0.0;
  node_->get_parameter(stair_prefix + "max_snapshot_age_sec", max_snapshot_age_sec);
  node_->get_parameter(
    stair_prefix + "minimum_stair_confidence", minimum_stair_confidence);
  node_->get_parameter(
    stair_prefix + "max_node_match_distance_m", max_node_match_distance_m);
  node_->get_parameter(
    stair_prefix + "riser_plane_tolerance_m", riser_plane_tolerance_m);
  node_->get_parameter(
    stair_prefix + "riser_lateral_tolerance_m", riser_lateral_tolerance_m);
  node_->get_parameter(
    stair_prefix + "riser_vertical_tolerance_m", riser_vertical_tolerance_m);
  if(std::isfinite(max_snapshot_age_sec) && max_snapshot_age_sec > 0.0 &&
    max_snapshot_age_sec <=
    static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1.0e9)
  {
    stair_riser_map_semantics_config_.max_snapshot_age_nanoseconds =
      static_cast<std::int64_t>(max_snapshot_age_sec * 1.0e9);
  }
  stair_riser_map_semantics_config_.minimum_stair_confidence =
    static_cast<float>(minimum_stair_confidence);
  stair_riser_map_semantics_config_.max_node_match_distance_m =
    static_cast<float>(max_node_match_distance_m);
  stair_riser_map_semantics_config_.riser_plane_tolerance_m =
    static_cast<float>(riser_plane_tolerance_m);
  stair_riser_map_semantics_config_.riser_lateral_tolerance_m =
    static_cast<float>(riser_lateral_tolerance_m);
  stair_riser_map_semantics_config_.riser_vertical_tolerance_m =
    static_cast<float>(riser_vertical_tolerance_m);
  std::string stair_config_error;
  stair_riser_map_semantics_config_valid_ = StairRiserSemantics::validConfig(
    stair_riser_map_semantics_config_, &stair_config_error);
  if(!stair_riser_map_semantics_config_valid_){
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_),
      "Static map stair-riser semantics are invalid; all mapcloud points remain "
      "obstacles: %s", stair_config_error.c_str());
  }

  
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



}

void StaticLayer::ptrInitial(){
  pcl_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pcl_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pending_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  pending_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  shared_data_->sGraph_ptr_ = std::make_shared<perception_3d::StaticGraph>();
  is_local_planner_ = false;
  is_ground_and_map_being_initialized_once_ = false;
}

void StaticLayer::cbMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr decoded(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *decoded);
  std::vector<int> indices;
  decoded->is_dense = false;
  pcl::removeNaNFromPointCloud(*decoded, *decoded, indices);

  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  pending_map_ = std::move(decoded);
  map_pairing_state_.noteMapMessage();
  // Either half starts a new replacement epoch.  Invalidate terrain now so a
  // map-first delivery cannot keep publishing a model bound to the previous
  // static pair.  The latest token from either side is the only commit token.
  pending_static_update_token_ = shared_data_->beginStaticMapGroundUpdate();
  if (pending_map_->empty()) {
    shared_data_->is_static_layer_ready_ = false;
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_),
      "%s rejected an empty Map message after finite-point filtering; "
      "navigation remains fail-closed until a complete non-empty pair arrives",
      name_.c_str());
    return;
  }
  RCLCPP_WARN(
    node_->get_logger().get_child(name_),
    "%s staged a new Map message with %lu finite points (update token %llu); "
    "waiting for a new Ground message before atomically rebuilding the static pair",
    name_.c_str(), pending_map_->points.size(),
    static_cast<unsigned long long>(pending_static_update_token_));
}

void StaticLayer::cbGround(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr decoded(
    new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *decoded);
  std::vector<int> indices;
  decoded->is_dense = false;
  pcl::removeNaNFromPointCloud(*decoded, *decoded, indices);

  std::unique_lock<std::recursive_mutex> lock(shared_data_->ground_kdtree_cb_mutex_);
  pending_ground_ = std::move(decoded);
  map_pairing_state_.noteGroundMessage();
  // Block terrain publication immediately, while keeping the previous complete
  // map/ground pair visible. The generation advances only when selfClear()
  // commits both new messages together.
  pending_static_update_token_ = shared_data_->beginStaticMapGroundUpdate();
  if (pending_ground_->empty()) {
    shared_data_->is_static_layer_ready_ = false;
    RCLCPP_ERROR(
      node_->get_logger().get_child(name_),
      "%s rejected an empty Ground message after finite-point filtering; "
      "navigation remains fail-closed until a complete non-empty pair arrives",
      name_.c_str());
    return;
  }
  RCLCPP_WARN(
    node_->get_logger().get_child(name_),
    "%s staged a new Ground message with %lu finite points (update token %llu); "
    "waiting for a new Map message before atomically rebuilding the static pair",
    name_.c_str(), pending_ground_->points.size(),
    static_cast<unsigned long long>(pending_static_update_token_));
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
  if(stair_riser_map_semantics_config_.enabled &&
    stair_riser_map_semantics_config_valid_ &&
    is_ground_and_map_being_initialized_once_ && !is_local_planner_)
  {
    const auto snapshot = shared_data_->getTerrainSnapshot();
    const bool semantics_active = staticRiserSnapshotReady(
      stair_riser_map_semantics_config_, snapshot,
      pcl_ground_ ? pcl_ground_->size() : 0U, node_->now().nanoseconds());
    const std::uint64_t snapshot_version = semantics_active ? snapshot->version() : 0U;
    if(semantics_active != applied_stair_riser_map_semantics_ ||
      snapshot_version != applied_stair_riser_snapshot_version_)
    {
      resetdGraph();
      if(!mapping_mode_ && enable_edge_detection_){
        radiusSearchConnection();
      }
      if(generate_static_graph_){
        generateStaticGraph();
      }
      applied_stair_riser_map_semantics_ = semantics_active;
      applied_stair_riser_snapshot_version_ = snapshot_version;
      RCLCPP_WARN(
        node_->get_logger().get_child(name_),
        "Static map stair-riser semantics rebuilt: active=%d snapshot_version=%lu; "
        "live/dynamic layers remain unchanged",
        semantics_active,
        static_cast<unsigned long>(snapshot_version));
    }
  }
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
  if(map_pairing_state_.pairReady()){

    if(!pending_map_ || !pending_ground_ || pending_static_update_token_ == 0U ||
      !staticMapPayloadPairIsValid(pending_map_->size(), pending_ground_->size()))
    {
      shared_data_->is_static_layer_ready_ = false;
      RCLCPP_ERROR(
        node_->get_logger().get_child(name_),
        "Static map/ground pair is marked ready without a complete non-empty "
        "staged payload; keeping navigation fail-closed");
      return;
    }

    const std::uint64_t ground_generation =
      shared_data_->commitStaticMapGroundUpdate(pending_static_update_token_);
    if(ground_generation == 0U){
      shared_data_->is_static_layer_ready_ = false;
      RCLCPP_ERROR(
        node_->get_logger().get_child(name_),
        "Rejected stale static map/ground update token %llu; keeping the "
        "previous complete pair and navigation fail-closed",
        static_cast<unsigned long long>(pending_static_update_token_));
      return;
    }

    shared_data_->is_static_layer_ready_ = false;
    pcl_map_ = std::move(pending_map_);
    pcl_ground_ = std::move(pending_ground_);
    pending_map_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pending_ground_.reset(new pcl::PointCloud<pcl::PointXYZI>);

    shared_data_->static_map_size_ = pcl_map_->points.size();
    shared_data_->static_ground_size_ = pcl_ground_->points.size();
    shared_data_->kdtree_map_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
    shared_data_->kdtree_map_->setInputCloud(pcl_map_);
    shared_data_->kdtree_ground_.reset(new pcl::search::KdTree<pcl::PointXYZI>());
    shared_data_->kdtree_ground_->setInputCloud(pcl_ground_);
    shared_data_->pcl_map_ = pcl_map_;
    shared_data_->pcl_ground_ = pcl_ground_;

    const auto pair_commit = map_pairing_state_.commit();
    pending_static_update_token_ = 0U;

    RCLCPP_WARN(
      node_->get_logger().get_child(name_),
      "%s committed static pair %llu (map message %llu, ground message %llu, "
      "terrain/static-ground generation %llu): map=%lu ground=%lu",
      name_.c_str(),
      static_cast<unsigned long long>(pair_commit.pair_generation),
      static_cast<unsigned long long>(pair_commit.map_message_generation),
      static_cast<unsigned long long>(pair_commit.ground_message_generation),
      static_cast<unsigned long long>(ground_generation),
      pcl_map_->points.size(), pcl_ground_->points.size());
    shared_data_->requestAllLayersToResetDGraph();

    shared_data_->dgraph_update_request_[name_] = false;

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
    if(!is_ground_and_map_being_initialized_once_)
      shared_data_->is_static_layer_ready_ = false;
  }

}

void StaticLayer::generateStaticGraph(){

  std::unique_lock<std::mutex> terrain_identity_lease;
  TerrainSnapshotConstPtr stair_snapshot;
  const std::int64_t semantic_now_nanoseconds = node_->now().nanoseconds();
  if(stair_riser_map_semantics_config_.enabled &&
    stair_riser_map_semantics_config_valid_)
  {
    terrain_identity_lease = shared_data_->acquireTerrainIdentityLease();
    stair_snapshot = shared_data_->getTerrainSnapshot();
  }
  (void)terrain_identity_lease;
  const bool stair_semantics_active = staticRiserSnapshotReady(
    stair_riser_map_semantics_config_, stair_snapshot, pcl_ground_->size(),
    semantic_now_nanoseconds);
  
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

    //@ use map to impose weight on each node
    pointIdxRadiusSearch.clear();
    pointRadiusSquaredDistance.clear();
    if(stair_semantics_active){
      shared_data_->kdtree_map_->radiusSearch(
        pcl_node, gbl_utils_->getInscribedRadius(), pointIdxRadiusSearch,
        pointRadiusSquaredDistance);
      double minimum_retained_distance = std::numeric_limits<double>::infinity();
      for(std::size_t result_index = 0U;
        result_index < pointIdxRadiusSearch.size() &&
        result_index < pointRadiusSquaredDistance.size(); ++result_index)
      {
        const int raw_map_index = pointIdxRadiusSearch[result_index];
        if(raw_map_index < 0 ||
          static_cast<std::size_t>(raw_map_index) >= pcl_map_->size())
        {
          continue;
        }
        const std::size_t map_index = static_cast<std::size_t>(raw_map_index);
        if(isExpectedStaticMapRiser(
            stair_riser_map_semantics_config_, stair_snapshot,
            semantic_now_nanoseconds, index_cnt, pcl_node,
            pcl_map_->points[map_index]))
        {
          continue;
        }
        minimum_retained_distance = std::min(
          minimum_retained_distance,
          std::sqrt(static_cast<double>(pointRadiusSquaredDistance[result_index])));
      }
      if(std::isfinite(minimum_retained_distance)){
        dGraph_.setValue(index_cnt, minimum_retained_distance);
      }
    }else if(shared_data_->kdtree_map_->nearestKSearch(
        pcl_node, 1, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    {
      const double distance_to_obstacle = std::sqrt(pointRadiusSquaredDistance[0]);
      if(distance_to_obstacle < gbl_utils_->getInscribedRadius()){
        dGraph_.setValue(index_cnt, distance_to_obstacle);
      }
    }
  }
}

void StaticLayer::radiusSearchConnection(){

  std::unique_lock<std::mutex> terrain_identity_lease;
  TerrainSnapshotConstPtr stair_snapshot;
  const std::int64_t semantic_now_nanoseconds = node_->now().nanoseconds();
  if(stair_riser_map_semantics_config_.enabled &&
    stair_riser_map_semantics_config_valid_)
  {
    terrain_identity_lease = shared_data_->acquireTerrainIdentityLease();
    stair_snapshot = shared_data_->getTerrainSnapshot();
  }
  (void)terrain_identity_lease;
  const bool stair_semantics_active = staticRiserSnapshotReady(
    stair_riser_map_semantics_config_, stair_snapshot, pcl_ground_->size(),
    semantic_now_nanoseconds);
  
  // 1. Determine total loop size
  const unsigned int total_points = pcl_ground_->points.size();
  std::size_t static_obstacle_nodes = 0;
  std::size_t static_lethal_nodes = 0;
  std::size_t static_riser_associations_ignored = 0;

  // 2. OpenMP Parallel For Loop
  // We specify that 'index_cnt' is the loop variable (implicitly private)
  #pragma omp parallel for reduction(+:static_obstacle_nodes,static_lethal_nodes,static_riser_associations_ignored)
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
      
      //@ use map to impose weight on each node
      pointIdxRadiusSearch.clear();
      pointRadiusSquaredDistance.clear();
      shared_data_->kdtree_map_->radiusSearch (pcl_node, static_imposing_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance);
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_z_axes(new pcl::PointCloud<pcl::PointXYZI>);
      for(auto i_pcl_z_axes=pointIdxRadiusSearch.begin();i_pcl_z_axes!=pointIdxRadiusSearch.end();i_pcl_z_axes++){
        const std::size_t map_index = static_cast<std::size_t>(*i_pcl_z_axes);
        if(map_index >= pcl_map_->size()){
          continue;
        }
        if(stair_semantics_active &&
          isExpectedStaticMapRiser(
            stair_riser_map_semantics_config_, stair_snapshot,
            semantic_now_nanoseconds, index_cnt, pcl_node,
            pcl_map_->points[map_index]))
        {
          ++static_riser_associations_ignored;
          continue;
        }
        pcl_z_axes->push_back(pcl_map_->points[map_index]);
      }

      // Local Filter instantiation
      pcl::PassThrough<pcl::PointXYZI> pass;
      pass.setInputCloud (pcl_z_axes);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (pcl_node.z+0.1, pcl_node.z+1.0);
      pass.filter (*pcl_z_axes);
      pass.setInputCloud (pcl_z_axes);
      pass.setFilterFieldName ("x");
      pass.setFilterLimits (pcl_node.x-static_obstacle_xy_radius_, pcl_node.x+static_obstacle_xy_radius_);
      pass.filter (*pcl_z_axes);
      pass.setInputCloud (pcl_z_axes);
      pass.setFilterFieldName ("y");
      pass.setFilterLimits (pcl_node.y-static_obstacle_xy_radius_, pcl_node.y+static_obstacle_xy_radius_);
      pass.filter (*pcl_z_axes);
      
      if(static_cast<int>(pcl_z_axes->points.size()) >= static_obstacle_min_points_){
        double min_horizontal_distance = std::numeric_limits<double>::infinity();
        for(const auto& obstacle_point : pcl_z_axes->points){
          min_horizontal_distance = std::min(
            min_horizontal_distance,
            std::hypot(
              static_cast<double>(obstacle_point.x - pcl_node.x),
              static_cast<double>(obstacle_point.y - pcl_node.y)));
        }
        if(std::isfinite(min_horizontal_distance)){
          // Preserve the actual centerline clearance. This keeps a detected
          // wall lethal inside inscribed_radius without turning every ground
          // node in the classification window into a zero-distance obstacle.
          dGraph_.setValue(index_cnt, min_horizontal_distance);
          static_obstacle_nodes++;
          if(min_horizontal_distance < gbl_utils_->getInscribedRadius()){
            static_lethal_nodes++;
          }
        }
      }  
    }
    shared_data_->sGraph_ptr_->setPenality(index_cnt, intensity_penality);
  }
  RCLCPP_INFO(node_->get_logger().get_child(name_),
    "Static obstacle ground nodes: %zu/%u classified, %zu/%u lethal "
    "(minimum map points: %d, XY radius: %.2f, expected-riser associations ignored: %zu)",
    static_obstacle_nodes, total_points, static_lethal_nodes, total_points,
    static_obstacle_min_points_, static_obstacle_xy_radius_,
    static_riser_associations_ignored);
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
