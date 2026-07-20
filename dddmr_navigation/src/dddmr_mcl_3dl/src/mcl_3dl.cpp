/*
 * Copyright (c) 2016-2020, the mcl_3dl authors
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mcl_3dl.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

namespace mcl_3dl
{

MCL3dlNode::MCL3dlNode(std::string name) : Node(name)
  , engine_(seed_gen_())
  , is_trans_b2s_initialized_(false)
  , tf_ready_(false)
  , first_tf_(false)
  , global_localization_requested_(false)
  , use_global_map_(false)
  , last_feature_received_ns_(0)
  , last_odom_received_ns_(0)
  , last_measure_ns_(0)
  , last_global_attempt_ns_(0)
  , localizing_started_ns_(0)
  , latest_match_ratio_(0.0f)
  , feature_sequence_(0)
  , last_measured_feature_sequence_(0)
{
  //supress the no intensity found log
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  
  clock_ = this->get_clock();
}

MCL3dlNode::~MCL3dlNode(){
  //tf_publish_thread_.join();
}

bool MCL3dlNode::configure(const std::shared_ptr<mcl_3dl::SubMaps>& sub_maps)
{
  sub_maps_ = sub_maps;
  params_ = std::make_shared<Parameters>(this->get_node_logging_interface(), this->get_node_parameters_interface());
  lidar_measurements_ = std::make_shared<LidarMeasurementModelLikelihood>();
  lidar_measurements_->loadConfig(this->get_node_logging_interface(), this->get_node_parameters_interface());

  LocalizationStateConfig state_config;
  state_config.tracking_match_ratio = params_->localization_tracking_match_ratio_;
  state_config.lost_match_ratio = params_->localization_lost_match_ratio_;
  state_config.tracking_max_xy_std = params_->localization_tracking_max_xy_std_;
  state_config.tracking_max_yaw_std = params_->localization_tracking_max_yaw_std_;
  state_config.lost_max_xy_std = params_->localization_lost_max_xy_std_;
  state_config.lost_max_yaw_std = params_->localization_lost_max_yaw_std_;
  state_config.tracking_good_frames = params_->localization_tracking_good_frames_;
  state_config.lost_bad_frames = params_->localization_lost_bad_frames_;
  localization_state_machine_ = std::make_unique<LocalizationStateMachine>(state_config);

  pf_.reset(new pf::ParticleFilter<State6DOF,
                                    float,
                                    ParticleWeightedMeanQuat,
                                    std::default_random_engine>(params_->num_particles_));
  pf_->init(params_->initial_pose_, params_->initial_pose_std_);

  f_pos_.reset(new FilterVec3(
      Filter::FILTER_LPF,
      Vec3(params_->lpf_step_, params_->lpf_step_, params_->lpf_step_),
      Vec3()));
  f_ang_.reset(new FilterVec3(
      Filter::FILTER_LPF,
      Vec3(params_->lpf_step_, params_->lpf_step_, params_->lpf_step_),
      Vec3(), true));
  f_acc_.reset(new FilterVec3(
      Filter::FILTER_LPF,
      Vec3(params_->acc_lpf_step_, params_->acc_lpf_step_, params_->acc_lpf_step_),
      Vec3()));


  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_pub_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  localization_status_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  
  //@Initialize transform listener and broadcaster
  tfbuf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tfbuf_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 2,
      std::bind(&MCL3dlNode::cbOdom, this, std::placeholders::_1), sub_options);

  sub_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_3d_pose", 2,
      std::bind(&MCL3dlNode::cbPosition, this, std::placeholders::_1), sub_options);

  lc_sharp_.subscribe(this, "laser_cloud_sharp");
  lc_less_sharp_.subscribe(this, "laser_cloud_less_sharp");
  lc_flat_.subscribe(this, "laser_cloud_flat");
  lc_less_flat_.subscribe(this, "laser_cloud_less_flat");
  
  syncApproximate_ = std::make_shared<message_filters::Synchronizer<LegoSyncPolicy>>(LegoSyncPolicy(5), lc_sharp_, lc_less_sharp_, lc_flat_, lc_less_flat_);
  syncApproximate_->registerCallback(&MCL3dlNode::cbLeGoFeatureCloud, this);  
  
  tf_pub_timer_ = this->create_wall_timer(50ms, std::bind(&MCL3dlNode::publishTFThread, this), tf_pub_group_);
  localization_status_timer_ = this->create_wall_timer(
      200ms, std::bind(&MCL3dlNode::publishLocalizationStatusThread, this),
      localization_status_group_);

  pub_ground_normal_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("ground_normal", 1);  
  pub_pc_ec_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("euclidean_cluster_extraction", 1);
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("mcl_pose", 1);
  pub_particle_ = this->create_publisher<geometry_msgs::msg::PoseArray>("particles", 1);
  pub_localization_status_ = this->create_publisher<std_msgs::msg::String>(
      "localization_status",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_localization_quality_ =
      this->create_publisher<std_msgs::msg::Float32>("localization_quality", 1);

  srv_global_localization_ = this->create_service<std_srvs::srv::Trigger>(
      "global_localization",
      std::bind(
          &MCL3dlNode::cbGlobalLocalization, this,
          std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, cbs_group_);
  
  has_odom_ = false;

  motion_prediction_model_ = MotionPredictionModelBase::Ptr(
      new MotionPredictionModelDifferentialDrive(params_->odom_err_integ_lin_tc_,
                                                  params_->odom_err_integ_ang_tc_));

  if (params_->auto_global_localization_)
  {
    global_localization_requested_.store(true);
    localization_state_reason_ = "waiting for global map and live sensor data";
  }
  else
  {
    startLocalizing("configured initial pose");
  }
  publishLocalizationStatus();

  return true;
  /*

  sub_landmark_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/landmark", 2, &MCL3dlNode::cbLandmark, this);

  pub_pc_normal_ = pnh_.advertise<visualization_msgs::MarkerArray>("normal_marker", 2, true);
  srv_expansion_reset_ = pnh_.advertiseService("expansion_resetting", &MCL3dlNode::cbExpansionReset, this);
  
  */
}

void MCL3dlNode::cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg){

  const rclcpp::Time now = clock_->now();
  const int64_t now_ns = now.nanoseconds();
  last_odom_received_ns_.store(now_ns);

  odom_ =
      State6DOF(
          Vec3(msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z),
          Quat(msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w));

  odom_header_ = msg->header;
  odom_trans_.header = msg->header;
  odom_trans_.child_frame_id = msg->child_frame_id;
  odom_trans_.transform.translation.x = msg->pose.pose.position.x;
  odom_trans_.transform.translation.y = msg->pose.pose.position.y;
  odom_trans_.transform.translation.z = msg->pose.pose.position.z;
  odom_trans_.transform.rotation.x = msg->pose.pose.orientation.x;
  odom_trans_.transform.rotation.y = msg->pose.pose.orientation.y;
  odom_trans_.transform.rotation.z = msg->pose.pose.orientation.z;
  odom_trans_.transform.rotation.w = msg->pose.pose.orientation.w;
  if(odom_header_.stamp.sec==0 && odom_header_.stamp.nanosec == 0){
    odom_trans_.header.stamp = clock_->now();
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 3000, "Odometry msg.header.timestamp = 0, use clock.now() as the timestamp");
  }
  tfb_->sendTransform(odom_trans_);

  if (!has_odom_)
  {
    odom_prev_ = odom_;
    odom_last_ =
        (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
        now : rclcpp::Time(msg->header.stamp);
    has_odom_ = true;
    return;
  }
  
  double dx = odom_.pos_.x_ - odom_prev_.pos_.x_;
  double dy = odom_.pos_.y_ - odom_prev_.pos_.y_;
  double dz = odom_.pos_.z_ - odom_prev_.pos_.z_;

  tf2::Quaternion q_odom(odom_.rot_.x_, odom_.rot_.y_, odom_.rot_.z_, odom_.rot_.w_);
  tf2::Quaternion q_odom_prev_(odom_prev_.rot_.x_, odom_prev_.rot_.y_, odom_prev_.rot_.z_, odom_prev_.rot_.w_);

  double roll_odom, pitch_odom, yaw_odom;
  tf2::Matrix3x3(q_odom).getRPY(roll_odom, pitch_odom, yaw_odom);

  double roll_odom_prev, pitch_odom_prev, yaw_odom_prev;
  tf2::Matrix3x3(q_odom_prev_).getRPY(roll_odom_prev, pitch_odom_prev, yaw_odom_prev);

  double droll = roll_odom - roll_odom_prev;
  double dpitch = pitch_odom - pitch_odom_prev;
  double dyaw = yaw_odom - yaw_odom_prev;
  rclcpp::Time msg_time =
      (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) ?
      now : rclcpp::Time(msg->header.stamp);
  const double dt = msg_time.seconds() - odom_last_.seconds();

  const int64_t last_measure_ns = last_measure_ns_.load();
  const bool periodic_measure =
      last_measure_ns == 0 ||
      static_cast<double>(now_ns - last_measure_ns) / 1e9 >=
        params_->localization_measure_interval_sec_;
  const bool moved =
      std::sqrt(dx * dx + dy * dy + dz * dz) > params_->update_min_d_ ||
      std::sqrt(droll * droll + dpitch * dpitch + dyaw * dyaw) > params_->update_min_a_;
  const LocalizationState state = localizationState();
  if (state == LocalizationState::TRACKING && first_tf_.load() &&
      !moved && !periodic_measure)
  {
    return;
  }

  std::unique_lock<std::mutex> lock(protect_measure_in_odomcb_);
  if (pcl_segmentations_.empty())
  {
    return;
  }

  const int64_t last_feature_ns = last_feature_received_ns_.load();
  if (last_feature_ns == 0 ||
      static_cast<double>(now_ns - last_feature_ns) / 1e9 >
        params_->localization_sensor_timeout_sec_)
  {
    if (state == LocalizationState::TRACKING || state == LocalizationState::LOCALIZING)
    {
      markLocalizationLost("lidar feature stream timed out");
    }
    return;
  }

  bool just_seeded_globally = false;
  const LocalizationState current_state = localizationState();
  const bool should_global_localize =
      global_localization_requested_.load() ||
      (params_->auto_global_localization_ &&
       (current_state == LocalizationState::UNINITIALIZED ||
        current_state == LocalizationState::LOST));
  const uint64_t feature_sequence = feature_sequence_.load();
  if (!should_global_localize &&
      feature_sequence == last_measured_feature_sequence_.load())
  {
    return;
  }
  if (should_global_localize)
  {
    const int64_t last_attempt_ns = last_global_attempt_ns_.load();
    if (last_attempt_ns != 0 &&
        static_cast<double>(now_ns - last_attempt_ns) / 1e9 <
          params_->global_localization_retry_sec_)
    {
      return;
    }
    last_global_attempt_ns_.store(now_ns);
    if (!attemptGlobalLocalization(pcl_segmentations_))
    {
      return;
    }
    global_localization_requested_.store(false);
    just_seeded_globally = true;
  }
  else if (current_state == LocalizationState::UNINITIALIZED ||
           current_state == LocalizationState::LOST)
  {
    return;
  }

  if (!just_seeded_globally)
  {
    motion_prediction_model_->setOdoms(odom_prev_, odom_, std::max(0.0, dt));
    auto prediction_func = [this](State6DOF& s)
    {
      motion_prediction_model_->predict(s);
    };
    pf_->predict(prediction_func);
  }

  odom_last_ = msg_time;
  odom_prev_ = odom_;

  if (!measure(pcl_segmentations_))
  {
    return;
  }
  last_measured_feature_sequence_.store(feature_sequence);
  last_measure_ns_.store(now_ns);

  if (localizationState() == LocalizationState::LOST)
  {
    return;
  }

  pf_->resample(State6DOF(
      Vec3(params_->resample_var_x_,
            params_->resample_var_y_,
            params_->resample_var_z_),
      Vec3(params_->resample_var_roll_,
            params_->resample_var_pitch_,
            params_->resample_var_yaw_)));

  std::normal_distribution<float> noise(0.0, 1.0);
  auto update_noise_func = [this, &noise](State6DOF& s)
  {
    s.noise_ll_ = noise(engine_) * params_->odom_err_lin_lin_;
    s.noise_la_ = noise(engine_) * params_->odom_err_lin_ang_;
    s.noise_aa_ = noise(engine_) * params_->odom_err_ang_ang_;
    s.noise_al_ = noise(engine_) * params_->odom_err_ang_lin_;
  };
  pf_->predict(update_noise_func);

  publishParticles();
}

bool MCL3dlNode::getBaselink2SensorAF3(std_msgs::msg::Header sensor_header, Eigen::Affine3d& trans_b2s_af3){

  if(! is_trans_b2s_initialized_){
  
    if(sensor_header.frame_id.at(0) == '/'){
      sensor_header.frame_id.erase(0, 1);
    }

    try
    {
      trans_b2s_ = tfbuf_->lookupTransform(
          params_->frame_ids_["base_link"], sensor_header.frame_id, tf2::TimePointZero);
    }
    catch (tf2::TransformException& e)
    {
      RCLCPP_INFO(this->get_logger(), "Failed to transform pointcloud: %s", e.what());
      return false;
    }
    //@Transform into base_link frame, and then we can perform passthrough
    is_trans_b2s_initialized_ = true;
    trans_b2s_af3 = tf2::transformToEigen(trans_b2s_);
  }
  else{
    trans_b2s_af3 = tf2::transformToEigen(trans_b2s_);
  }
  return true;
}

void MCL3dlNode::cbLeGoFeatureCloud(const sensor_msgs::msg::PointCloud2::SharedPtr pc_sharpMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_less_sharpMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_flatMsg,
                    const sensor_msgs::msg::PointCloud2::SharedPtr pc_less_flatMsg){
  
  std::unique_lock<std::mutex> lock(protect_measure_in_odomcb_);
  
  laser_header_ = pc_less_sharpMsg->header;

  if (!sub_maps_->isCurrentReady() && !sub_maps_->isGlobalReady())
    return;

  Eigen::Affine3d trans_b2s_af3;
  if(! getBaselink2SensorAF3(pc_less_sharpMsg->header, trans_b2s_af3))
    return;

  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_sharp(new pcl::PointCloud<mcl_3dl::pcl_t>);
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_sharp(new pcl::PointCloud<mcl_3dl::pcl_t>);
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_flat(new pcl::PointCloud<mcl_3dl::pcl_t>); 
  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_flat(new pcl::PointCloud<mcl_3dl::pcl_t>);

  //@
  //@--->pc_less_flat comprises flat
  //@--->pc_less_sharp comprises sharp
  //However pc_less_flat comprises too many points causing compution overhead
  

  //pcl::fromROSMsg(*pc_sharpMsg, *pc_sharp);
  pcl::fromROSMsg(*pc_flatMsg, *pc_flat);
  pcl::fromROSMsg(*pc_less_sharpMsg, *pc_less_sharp);
  //pcl::fromROSMsg(*pc_less_flatMsg, *pc_less_flat);

  //@Transform point cloud
  //pcl::transformPointCloud(*pc_sharp, *pc_sharp, trans_b2s_af3);
  pcl::transformPointCloud(*pc_less_sharp, *pc_less_sharp, trans_b2s_af3);
  pcl::transformPointCloud(*pc_flat, *pc_flat, trans_b2s_af3);
  pcl::transformPointCloud(*pc_less_flat, *pc_less_flat, trans_b2s_af3);
  
  //*pc_less_sharp+=*pc_less_flat;

  pc_less_sharp->header.frame_id = params_->frame_ids_["base_link"];

  RCLCPP_DEBUG(this->get_logger(), "Size pc_sharp: %lu, pc_less_sharp: %lu, pc_flat: %lu, pc_less_flat: %lu", 
                          pc_sharp->points.size(), pc_less_sharp->points.size(), pc_flat->points.size(), pc_less_flat->points.size());


  //Ground will skew the result when close to obstacle, because velodyne has blind spot of 50 cm
  pcl::VoxelGrid<mcl_3dl::pcl_t> sor;
  sor.setInputCloud (pc_flat);
  sor.setLeafSize (1.0f, 1.0f, 0.1f);
  sor.filter (*pc_flat);
  
  //@Looks like downsample less sharp only 600->550 points, not very effective
  //pcl::VoxelGrid<mcl_3dl::pcl_t> sor2;
  //sor2.setInputCloud (pc_less_sharp);
  //sor2.setLeafSize (0.1f, 0.1f, 0.1f);
  //sor2.filter (*pc_less_sharp);

  pcl::PointCloud<mcl_3dl::pcl_t>::Ptr pc_less_sharp_intensity(new pcl::PointCloud<mcl_3dl::pcl_t>);

  //@Normal estimation on observation
  pcl::PointCloud<pcl::Normal>::Ptr observation_normals;
  observation_normals.reset(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<mcl_3dl::pcl_t, pcl::Normal> n2;
  pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr tree2 (new pcl::search::KdTree<mcl_3dl::pcl_t>);
  tree2->setInputCloud (pc_less_sharp);
  n2.setInputCloud (pc_less_sharp);
  n2.setSearchMethod (tree2);
  n2.setKSearch (5);
  //n2.setRadiusSearch (0.1);
  n2.compute (*observation_normals);
  
  //@ Uncomment for visualization purpose
  //pcl::PointCloud<pcl::PointXYZ> observation_xyz;
  //pcl::copyPointCloud(*pc_less_sharp, observation_xyz);
  //pcl::PointCloud<pcl::PointNormal>::Ptr observation_points_normal;
  //observation_points_normal.reset(new pcl::PointCloud<pcl::PointNormal>);
  //pcl::concatenateFields (observation_xyz, *observation_normals, *observation_points_normal);
  //normal2quaternion(observation_points_normal);
  

  double sum_normal_y = 0;
  double sum_normal_x = 0;
  for(auto it=observation_normals->points.begin(); it!=observation_normals->points.end(); it++){
    //@ compute the normals statistic
    if(!isnan((*it).normal_y))
      sum_normal_y += fabs((*it).normal_y);
    if(!isnan((*it).normal_x))
      sum_normal_x += fabs((*it).normal_x);
  }

  //@Check normal skew
  bool x_dominant = false;
  bool y_dominant = false;
  if(sum_normal_x/sum_normal_y>=1.6){
    RCLCPP_DEBUG(this->get_logger(), "Environment is x dominant: %.2f, %.2f", sum_normal_x, sum_normal_y);
    x_dominant = true;
  }
  else if(sum_normal_y/sum_normal_x>=1.6){
    RCLCPP_DEBUG(this->get_logger(), "Environment is y dominant: %.2f, %.2f", sum_normal_x, sum_normal_y);
    y_dominant = true;
  }

  if(x_dominant || y_dominant){
    size_t normal_index = 0;
    for(auto a_pt=pc_less_sharp->points.begin();a_pt!=pc_less_sharp->points.end();a_pt++){

      pcl::PointXYZI i_pt;
      i_pt.x = (*a_pt).x;
      i_pt.y = (*a_pt).y;
      i_pt.z = (*a_pt).z;   

      if(x_dominant){
        if(isnan(observation_normals->points[normal_index].normal_x)){
          i_pt.intensity = 1.0;
        }
        else{
          double y2x = observation_normals->points[normal_index].normal_y/observation_normals->points[normal_index].normal_x;
          //@cap by the ratio of sum_normal_y:sum_normal_x
          if(y2x>=0.5)
            i_pt.intensity = 0.05*sum_normal_y/sum_normal_x;
          else
            i_pt.intensity = 1.0;
        }
      }
      else if(y_dominant){
        if(isnan(observation_normals->points[normal_index].normal_x)){
          i_pt.intensity = 1.0;
        }
        else{
          double x2y = observation_normals->points[normal_index].normal_x/observation_normals->points[normal_index].normal_y;
          if(x2y>=0.5)
            i_pt.intensity = 0.05*sum_normal_x/sum_normal_y;
          else
            i_pt.intensity = 1.0;
        }      
      }
      else{
        i_pt.intensity = 1.0;
      }

      pc_less_sharp_intensity->push_back(i_pt);
      normal_index++;
    }
  }
  else{
    //@ Euclidean Distance Segmentation on less_sharp
    //@ We want to extract isolated object and give the obj the same weight instead of by each point
    //@ max size is set to 2400 for 30 degree observation as an object instead of whole object ex: a long wall
    pcl::search::KdTree<mcl_3dl::pcl_t>::Ptr pc_kdtree (new pcl::search::KdTree<mcl_3dl::pcl_t>);
    pc_kdtree->setInputCloud (pc_less_sharp);

    std::vector<pcl::PointIndices> cluster_indices_segmentation;
    pcl::EuclideanClusterExtraction<mcl_3dl::pcl_t> ec_segmentation;
    ec_segmentation.setClusterTolerance (params_->euc_cluster_distance_);
    ec_segmentation.setMinClusterSize (params_->euc_cluster_min_size_);
    ec_segmentation.setMaxClusterSize (pc_less_sharp->points.size());
    ec_segmentation.setSearchMethod (pc_kdtree);
    ec_segmentation.setInputCloud (pc_less_sharp);
    ec_segmentation.extract (cluster_indices_segmentation);

    int smaller_cluster_amount = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices_segmentation.begin (); it != cluster_indices_segmentation.end (); ++it)
    {
        
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
        pcl::PointXYZI i_pt;
        i_pt.x = pc_less_sharp->points[*pit].x;
        i_pt.y = pc_less_sharp->points[*pit].y;
        i_pt.z = pc_less_sharp->points[*pit].z;
        //@ if cluster size is small, it is like a beam, therefore we higher its weight
        if(it->indices.size() < params_->euc_cluster_min_size_+1){
          smaller_cluster_amount++;
          i_pt.intensity = (1.0*it->indices.size())/(1.0*pc_less_sharp->points.size())/2.0;
        }
        else{
          i_pt.intensity = (1.0*it->indices.size())/(1.0*pc_less_sharp->points.size());
        }
        
        pc_less_sharp_intensity->push_back(i_pt);
        //RCLCPP_DEBUG(this->get_logger(), "%.2f,%.2f,%.2f,%.4f", i_pt.x, i_pt.y, i_pt.z, i_pt.intensity);
      } 
    }
    RCLCPP_DEBUG(this->get_logger(), "Total clusters: %lu, small clusters: %d",  cluster_indices_segmentation.size(), smaller_cluster_amount);
  }

  RCLCPP_DEBUG(this->get_logger(), "Size pc_sharp: %lu, pc_less_sharp: %lu, pc_flat: %lu, pc_less_flat: %lu", 
                          pc_sharp->points.size(), pc_less_sharp->points.size(), pc_flat->points.size(), pc_less_flat->points.size());

  //@
  //We need perpendicular features and surface.
  //Surface can correct roll and pitch
  //Perpendicular can correct yaw
  
  if (true){
    sensor_msgs::msg::PointCloud2 ec_out;
    pcl::toROSMsg(*pc_less_sharp_intensity, ec_out);
    ec_out.header = pc_less_sharpMsg->header;
    ec_out.header.frame_id = params_->frame_ids_["base_link"];
    pub_pc_ec_->publish(ec_out);
  }
  
  pcl_segmentations_[std::string("flat")] = pc_flat;
  pcl_segmentations_[std::string("less_sharp")] = pc_less_sharp_intensity;
  if (!pc_flat->empty() || !pc_less_sharp_intensity->empty())
  {
    last_feature_received_ns_.store(clock_->now().nanoseconds());
    feature_sequence_.fetch_add(1);
  }
  
}

std::vector<State6DOF> MCL3dlNode::buildGlobalCandidates() const
{
  const auto key_poses = sub_maps_->getKeyPoses();
  std::vector<geometry_msgs::msg::Pose> position_seeds;
  position_seeds.reserve(key_poses.size());

  for (const auto& pose : key_poses)
  {
    if (position_seeds.empty())
    {
      position_seeds.push_back(pose);
      continue;
    }

    const auto& previous = position_seeds.back().position;
    const double dx = pose.position.x - previous.x;
    const double dy = pose.position.y - previous.y;
    const double dz = pose.position.z - previous.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) >=
        params_->global_localization_grid_)
    {
      position_seeds.push_back(pose);
    }
  }

  if (!key_poses.empty() && !position_seeds.empty())
  {
    const auto& last_selected = position_seeds.back().position;
    const auto& last_key_pose = key_poses.back();
    const double dx = last_key_pose.position.x - last_selected.x;
    const double dy = last_key_pose.position.y - last_selected.y;
    const double dz = last_key_pose.position.z - last_selected.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > 1e-3)
    {
      position_seeds.push_back(last_key_pose);
    }
  }

  const std::size_t yaw_bins =
      static_cast<std::size_t>(params_->global_localization_div_yaw_);
  const std::size_t max_positions = std::max<std::size_t>(
      1, static_cast<std::size_t>(params_->global_localization_max_candidates_) /
        yaw_bins);
  if (position_seeds.size() > max_positions)
  {
    std::vector<geometry_msgs::msg::Pose> uniformly_sampled;
    uniformly_sampled.reserve(max_positions);
    for (std::size_t i = 0; i < max_positions; ++i)
    {
      const std::size_t index = max_positions == 1 ? 0 :
          i * (position_seeds.size() - 1) / (max_positions - 1);
      uniformly_sampled.push_back(position_seeds[index]);
    }
    position_seeds.swap(uniformly_sampled);
  }

  constexpr double kPi = 3.14159265358979323846;
  std::vector<State6DOF> candidates;
  candidates.reserve(position_seeds.size() * yaw_bins);
  for (const auto& pose : position_seeds)
  {
    tf2::Quaternion orientation(
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w);
    double roll = 0.0;
    double pitch = 0.0;
    double unused_yaw = 0.0;
    tf2::Matrix3x3(orientation).getRPY(roll, pitch, unused_yaw);

    for (std::size_t yaw_index = 0; yaw_index < yaw_bins; ++yaw_index)
    {
      const double yaw = -kPi +
          2.0 * kPi * static_cast<double>(yaw_index) /
            static_cast<double>(yaw_bins);
      candidates.emplace_back(
          Vec3(pose.position.x, pose.position.y, pose.position.z),
          Quat(Vec3(roll, pitch, yaw)));
    }
  }
  return candidates;
}

std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr>
MCL3dlNode::makeSparseObservation(
    const std::map<std::string, pcl::PointCloud<pcl_t>::Ptr>& pcl_segmentations) const
{
  std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr> sparse;
  const std::size_t per_cloud_limit = std::max<std::size_t>(
      8, static_cast<std::size_t>(params_->global_localization_max_observation_points_) / 2);

  for (const std::string& name : {std::string("flat"), std::string("less_sharp")})
  {
    auto output = std::make_shared<pcl::PointCloud<mcl_3dl::pcl_t>>();
    const auto found = pcl_segmentations.find(name);
    if (found != pcl_segmentations.end() && found->second)
    {
      const auto& input = *found->second;
      const std::size_t step = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(
            static_cast<double>(input.size()) /
            static_cast<double>(per_cloud_limit))));
      output->reserve(std::min(input.size(), per_cloud_limit));
      for (std::size_t i = 0; i < input.size() && output->size() < per_cloud_limit; i += step)
      {
        output->push_back(input.points[i]);
      }
    }
    sparse[name] = output;
  }
  return sparse;
}

void MCL3dlNode::initializeGlobalParticles(
    const std::vector<GlobalCandidate>& candidates)
{
  pf_->resizeParticle(params_->global_localization_num_particles_);
  std::size_t particle_index = 0;
  const float probability = 1.0f / static_cast<float>(pf_->getParticleSize());
  for (auto particle = pf_->begin(); particle != pf_->end(); ++particle, ++particle_index)
  {
    const auto& candidate = candidates[particle_index % candidates.size()];
    const DiagonalNoiseGenerator<float> noise_generator(
        candidate.state, params_->global_localization_seed_std_);
    particle->state_ = State6DOF::generateNoise<State6DOF>(engine_, noise_generator);
    particle->state_.normalize();
    particle->probability_ = probability;
    particle->probability_bias_ = 1.0f;
    particle->accum_probability_ = 0.0f;
  }
}

bool MCL3dlNode::attemptGlobalLocalization(
    const std::map<std::string, pcl::PointCloud<pcl_t>::Ptr>& pcl_segmentations)
{
  if (!sub_maps_->isGlobalReady())
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Global localization is waiting for complete map, ground, and key poses");
    return false;
  }

  const auto sparse_observation = makeSparseObservation(pcl_segmentations);
  if (sparse_observation.at("flat")->empty() &&
      sparse_observation.at("less_sharp")->empty())
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Global localization is waiting for non-empty lidar features");
    return false;
  }

  const auto candidate_states = buildGlobalCandidates();
  if (candidate_states.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Global localization has no pose-graph candidates");
    return false;
  }

  std::vector<GlobalCandidate> scored(candidate_states.size());
  #pragma omp parallel for
  for (std::size_t i = 0; i < candidate_states.size(); ++i)
  {
    const auto result = lidar_measurements_->measure(
        sub_maps_->kdtree_map_global_, sub_maps_->kdtree_ground_global_,
        sub_maps_->normals_ground_global_, sparse_observation, candidate_states[i]);
    scored[i].state = candidate_states[i];
    scored[i].quality = std::isfinite(result.quality) ? result.quality : 0.0f;
    scored[i].likelihood = std::isfinite(result.likelihood) ? result.likelihood : 0.0f;
  }

  std::sort(scored.begin(), scored.end(),
      [](const GlobalCandidate& lhs, const GlobalCandidate& rhs)
      {
        if (lhs.quality != rhs.quality)
        {
          return lhs.quality > rhs.quality;
        }
        return lhs.likelihood > rhs.likelihood;
      });

  if (scored.front().quality < params_->global_localization_min_match_ratio_)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Global localization rejected %lu candidates: best match %.3f is below %.3f",
        scored.size(), scored.front().quality,
        params_->global_localization_min_match_ratio_);
    return false;
  }

  scored.resize(std::min<std::size_t>(
      scored.size(), static_cast<std::size_t>(params_->global_localization_top_candidates_)));
  initializeGlobalParticles(scored);
  state_prev_ = scored.front().state;
  last_feature_received_ns_.store(clock_->now().nanoseconds());
  use_global_map_.store(true);
  tf_ready_.store(false);
  first_tf_.store(false);

  geometry_msgs::msg::PoseWithCovarianceStamped seed_pose;
  seed_pose.header.frame_id = params_->frame_ids_["map"];
  seed_pose.header.stamp = clock_->now();
  seed_pose.pose.pose.position.x = scored.front().state.pos_.x_;
  seed_pose.pose.pose.position.y = scored.front().state.pos_.y_;
  seed_pose.pose.pose.position.z = scored.front().state.pos_.z_;
  seed_pose.pose.pose.orientation.x = scored.front().state.rot_.x_;
  seed_pose.pose.pose.orientation.y = scored.front().state.rot_.y_;
  seed_pose.pose.pose.orientation.z = scored.front().state.rot_.z_;
  seed_pose.pose.pose.orientation.w = scored.front().state.rot_.w_;
  {
    std::unique_lock<mcl_3dl::SubMaps::sub_maps_mutex_t> lock(*(sub_maps_->getMutex()));
    sub_maps_->setInitialPose(seed_pose);
  }

  startLocalizing(
      "global coarse match accepted with quality " +
      std::to_string(scored.front().quality));
  publishParticles();
  RCLCPP_WARN(
      this->get_logger(),
      "Global localization seeded %d particles from %lu/%lu candidates; best pose "
      "(%.2f, %.2f, %.2f) quality=%.3f",
      params_->global_localization_num_particles_, scored.size(), candidate_states.size(),
      scored.front().state.pos_.x_, scored.front().state.pos_.y_,
      scored.front().state.pos_.z_, scored.front().quality);
  return true;
}

std::pair<double, double> MCL3dlNode::particleSpread(const State6DOF& mean) const
{
  const double mean_yaw = mean.rot_.getRPY().z_;
  double xy_variance = 0.0;
  double yaw_variance = 0.0;
  double probability_sum = 0.0;
  for (std::size_t i = 0; i < pf_->getParticleSize(); ++i)
  {
    const State6DOF particle = pf_->getParticle(i);
    const double probability = pf_->getParticleProbability(i);
    const double dx = particle.pos_.x_ - mean.pos_.x_;
    const double dy = particle.pos_.y_ - mean.pos_.y_;
    const double yaw = particle.rot_.getRPY().z_;
    const double yaw_error = std::atan2(
        std::sin(yaw - mean_yaw), std::cos(yaw - mean_yaw));
    xy_variance += probability * (dx * dx + dy * dy);
    yaw_variance += probability * yaw_error * yaw_error;
    probability_sum += probability;
  }

  if (probability_sum <= 0.0)
  {
    return {std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()};
  }
  return {
    std::sqrt(std::max(0.0, xy_variance / probability_sum)),
    std::sqrt(std::max(0.0, yaw_variance / probability_sum))};
}

bool MCL3dlNode::measure(
    const std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr>& pcl_segmentations)
{
  if (sub_maps_->isWarmUpReady())
  {
    sub_maps_->swapKdTree();
    if (isTracking())
    {
      use_global_map_.store(false);
    }
  }

  const bool use_global =
      sub_maps_->isGlobalReady() &&
      (use_global_map_.load() || localizationState() != LocalizationState::TRACKING);
  if (!use_global && !sub_maps_->isCurrentReady())
  {
    return false;
  }

  auto& map_tree = use_global ?
      sub_maps_->kdtree_map_global_ : sub_maps_->kdtree_map_current_;
  auto& ground_tree = use_global ?
      sub_maps_->kdtree_ground_global_ : sub_maps_->kdtree_ground_current_;
  auto& ground_normals = use_global ?
      sub_maps_->normals_ground_global_ : sub_maps_->normals_ground_current_;

  const auto ts = std::chrono::high_resolution_clock::now();
  lidar_measurements_->setGlobalLocalizationStatus(
      params_->num_particles_, pf_->getParticleSize());

  std::atomic<float> match_ratio_min(1.0f);
  std::atomic<float> match_ratio_max(0.0f);
  auto measure_func = [this, &pcl_segmentations, &map_tree, &ground_tree,
                        &ground_normals, &match_ratio_min,
                        &match_ratio_max](const State6DOF& s) -> float
  {
    const LidarMeasurementResult result = lidar_measurements_->measure(
        map_tree, ground_tree, ground_normals, pcl_segmentations, s);

    float observed_min = match_ratio_min.load();
    while (observed_min > result.quality &&
           !match_ratio_min.compare_exchange_weak(observed_min, result.quality))
    {
    }
    float observed_max = match_ratio_max.load();
    while (observed_max < result.quality &&
           !match_ratio_max.compare_exchange_weak(observed_max, result.quality))
    {
    }
    return result.likelihood;
  };

  //@ pf_->measure(measure_func) will loop particles
  pf_->measure(measure_func);

  //@ This block first calculate the weight (p.probability_bias_) based on the particle state
  //@ It means that the particle away from last pose has less weight
  //@ This weight is different from the weight of likelihood
  if (static_cast<int>(pf_->getParticleSize()) > params_->num_particles_)
  {
    auto bias_func = [](const State6DOF& s, float& p_bias) -> void
    {
      p_bias = 1.0;
    };
    pf_->bias(bias_func);
  }
  else
  {
    NormalLikelihood<float> nl_lin(params_->bias_var_dist_);
    NormalLikelihood<float> nl_ang(params_->bias_var_ang_);
    auto bias_func = [this, &nl_lin, &nl_ang](const State6DOF& s, float& p_bias) -> void
    {
      const float lin_diff = (s.pos_ - state_prev_.pos_).norm();
      Vec3 axis;
      float ang_diff;
      (s.rot_ * state_prev_.rot_.inv()).getAxisAng(axis, ang_diff);
      p_bias = nl_lin(lin_diff) * nl_ang(ang_diff) + 1e-6;
      assert(std::isfinite(p_bias));
    };
    //@ generate p.probability_bias_
    pf_->bias(bias_func);
  }

  //@ Weight particle based on the observation weight and p.probability_bias_
  auto e = pf_->expectationBiased();
  const auto e_max = pf_->max();

  assert(std::isfinite(e.pos_.x_));
  assert(std::isfinite(e.pos_.y_));
  assert(std::isfinite(e.pos_.z_));
  assert(std::isfinite(e.rot_.x_));
  assert(std::isfinite(e.rot_.y_));
  assert(std::isfinite(e.rot_.z_));
  assert(std::isfinite(e.rot_.w_));

  e.rot_.normalize();

  Vec3 map_pos;
  Quat map_rot;
  map_pos = e.pos_ - e.rot_ * odom_.rot_.inv() * odom_.pos_;
  map_rot = e.rot_ * odom_.rot_.inv();

  bool jump = false;
  if (static_cast<int>(pf_->getParticleSize()) > params_->num_particles_)
  {
    jump = true;
    state_prev_ = e;
  }
  else
  {
    Vec3 jump_axis;
    float jump_ang;
    float jump_dist = (e.pos_ - state_prev_.pos_).norm();
    (e.rot_.inv() * state_prev_.rot_).getAxisAng(jump_axis, jump_ang);
    if (jump_dist > params_->jump_dist_ ||
        fabs(jump_ang) > params_->jump_ang_)
    {
      RCLCPP_INFO(this->get_logger(), "Pose jumped pos:%0.3f, ang:%0.3f", jump_dist, jump_ang);
      jump = true;

      auto integ_reset_func = [](State6DOF& s)
      {
        s.odom_err_integ_lin_ = Vec3();
        s.odom_err_integ_ang_ = Vec3();
      };
      pf_->predict(integ_reset_func);
    }
    state_prev_ = e;
  }

  
  map2odom_trans_.header.stamp = laser_header_.stamp;
  map2odom_trans_.header.frame_id = params_->frame_ids_["map"];
  map2odom_trans_.child_frame_id = params_->frame_ids_["odom"];
  const auto rpy = map_rot.getRPY();
  if (jump)
  {
    f_ang_->set(rpy);
    f_pos_->set(map_pos);
  }
  map_rot.setRPY(f_ang_->in(rpy));
  map_pos = f_pos_->in(map_pos);
  map2odom_trans_.transform.translation = tf2::toMsg(tf2::Vector3(map_pos.x_, map_pos.y_, map_pos.z_));
  map2odom_trans_.transform.rotation = tf2::toMsg(tf2::Quaternion(map_rot.x_, map_rot.y_, map_rot.z_, map_rot.w_));

  // Calculate covariance from sampled particles to reduce calculation cost on global localization.
  // Use the number of original particles or at least 10% of full particles.
  auto cov = pf_->covariance(
      1.0,
      std::max(
          0.1f, static_cast<float>(params_->num_particles_) / pf_->getParticleSize()));

  const auto spread = particleSpread(e);
  const float best_match_ratio = match_ratio_max.load();
  latest_match_ratio_.store(best_match_ratio);
  std_msgs::msg::Float32 quality_msg;
  quality_msg.data = best_match_ratio;
  pub_localization_quality_->publish(quality_msg);

  bool state_changed = false;
  LocalizationState new_state;
  {
    std::lock_guard<std::mutex> state_lock(localization_state_mutex_);
    state_changed = localization_state_machine_->observe(LocalizationObservation{
        best_match_ratio,
        spread.first,
        spread.second,
        static_cast<int>(pf_->getParticleSize()) <= params_->num_particles_});
    new_state = localization_state_machine_->state();
    if (state_changed && new_state == LocalizationState::TRACKING)
    {
      localization_state_reason_ = "particle filter converged on consecutive observations";
    }
    else if (state_changed && new_state == LocalizationState::LOST)
    {
      localization_state_reason_ = "match quality or particle spread exceeded lost thresholds";
    }
  }

  if (new_state == LocalizationState::TRACKING)
  {
    tf_ready_.store(true);
  }
  else
  {
    tf_ready_.store(false);
  }

  if (state_changed)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Localization state changed to %s: match=%.3f xy_std=%.3f yaw_std=%.3f particles=%lu",
        localizationStateName(new_state), best_match_ratio, spread.first,
        spread.second, pf_->getParticleSize());
    publishLocalizationStatus();
    if (new_state == LocalizationState::LOST)
    {
      use_global_map_.store(true);
      if (params_->auto_global_localization_)
      {
        global_localization_requested_.store(true);
      }
    }
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = odom_last_;
  pose.header.frame_id = map2odom_trans_.header.frame_id;
  pose.pose.pose.position.x = e.pos_.x_;
  pose.pose.pose.position.y = e.pos_.y_;
  pose.pose.pose.position.z = e.pos_.z_;
  pose.pose.pose.orientation.x = e.rot_.x_;
  pose.pose.pose.orientation.y = e.rot_.y_;
  pose.pose.pose.orientation.z = e.rot_.z_;
  pose.pose.pose.orientation.w = e.rot_.w_;
  pose.pose.covariance.fill(0.0);
  pose.pose.covariance[0] = cov[0][0];
  pose.pose.covariance[7] = cov[1][1];
  pose.pose.covariance[14] = cov[2][2];
  pose.pose.covariance[21] = cov[3][3];
  pose.pose.covariance[28] = cov[4][4];
  pose.pose.covariance[35] = spread.second * spread.second;
  pub_pose_->publish(pose);

  if (new_state == LocalizationState::TRACKING)
  {
    std::unique_lock<mcl_3dl::SubMaps::sub_maps_mutex_t> lock(*(sub_maps_->getMutex()));
    if (state_changed)
    {
      sub_maps_->setInitialPose(pose);
      use_global_map_.store(true);
    }
    else
    {
      sub_maps_->setPose(pose);
    }
  }

  const auto tnow = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "MCL (%0.3f sec.)",
            std::chrono::duration<float>(tnow - ts).count());
  const auto err_integ_map = e_max.rot_ * e_max.odom_err_integ_lin_;
  RCLCPP_DEBUG(this->get_logger(),"odom error integral lin: %0.3f, %0.3f, %0.3f, "
            "ang: %0.3f, %0.3f, %0.3f, "
            "pos: %0.3f, %0.3f, %0.3f, "
            "err on map: %0.3f, %0.3f, %0.3f",
            e_max.odom_err_integ_lin_.x_,
            e_max.odom_err_integ_lin_.y_,
            e_max.odom_err_integ_lin_.z_,
            e_max.odom_err_integ_ang_.x_,
            e_max.odom_err_integ_ang_.y_,
            e_max.odom_err_integ_ang_.z_,
            e_max.pos_.x_,
            e_max.pos_.y_,
            e_max.pos_.z_,
            err_integ_map.x_,
            err_integ_map.y_,
            err_integ_map.z_);
  RCLCPP_DEBUG(this->get_logger(),"match ratio min: %0.3f, max: %0.3f, pos: %0.3f, %0.3f, %0.3f",
            match_ratio_min.load(),
            best_match_ratio,
            e.pos_.x_,
            e.pos_.y_,
            e.pos_.z_);

  if (new_state != LocalizationState::LOST &&
      best_match_ratio < params_->match_ratio_thresh_)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 3000, "Low match_ratio. Expansion resetting.");
    pf_->noise(State6DOF(
        Vec3(params_->expansion_var_x_,
              params_->expansion_var_y_,
              params_->expansion_var_z_),
        Vec3(params_->expansion_var_roll_,
              params_->expansion_var_pitch_,
              params_->expansion_var_yaw_)));
  }

  if (static_cast<int>(pf_->getParticleSize()) > params_->num_particles_)
  {
    const int reduced = pf_->getParticleSize() * 0.75;
    if (reduced > params_->num_particles_)
    {
      pf_->resizeParticle(reduced);
    }
    else
    {
      pf_->resizeParticle(params_->num_particles_);
    }
  }

  return true;
}

void MCL3dlNode::publishParticles()
{
  geometry_msgs::msg::PoseArray pa;
  pa.header.stamp = odom_header_.stamp;
  pa.header.frame_id = params_->frame_ids_["map"];
  for (size_t i = 0; i < pf_->getParticleSize(); i++)
  {
    geometry_msgs::msg::Pose pm;
    auto p = pf_->getParticle(i);
    p.rot_.normalize();
    pm.position.x = p.pos_.x_;
    pm.position.y = p.pos_.y_;
    pm.position.z = p.pos_.z_;
    pm.orientation.x = p.rot_.x_;
    pm.orientation.y = p.rot_.y_;
    pm.orientation.z = p.rot_.z_;
    pm.orientation.w = p.rot_.w_;
    pa.poses.push_back(pm);
  }
  pub_particle_->publish(pa);
}

LocalizationState MCL3dlNode::localizationState() const
{
  std::lock_guard<std::mutex> lock(localization_state_mutex_);
  return localization_state_machine_->state();
}

bool MCL3dlNode::isTracking() const
{
  return localizationState() == LocalizationState::TRACKING;
}

void MCL3dlNode::startLocalizing(const std::string& reason)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    changed = localization_state_machine_->startLocalizing();
    localization_state_reason_ = reason;
  }
  localizing_started_ns_.store(clock_->now().nanoseconds());
  tf_ready_.store(false);
  first_tf_.store(false);
  if (changed)
  {
    RCLCPP_WARN(this->get_logger(), "Localization state changed to LOCALIZING: %s", reason.c_str());
  }
  publishLocalizationStatus();
}

void MCL3dlNode::markLocalizationLost(const std::string& reason)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    changed = localization_state_machine_->markLost();
    localization_state_reason_ = reason;
  }
  tf_ready_.store(false);
  first_tf_.store(false);
  use_global_map_.store(true);
  if (params_->auto_global_localization_)
  {
    global_localization_requested_.store(true);
  }
  if (changed)
  {
    RCLCPP_ERROR(this->get_logger(), "Localization state changed to LOST: %s", reason.c_str());
  }
  publishLocalizationStatus();
}

void MCL3dlNode::requestGlobalLocalization(const std::string& reason)
{
  global_localization_requested_.store(true);
  last_global_attempt_ns_.store(0);
  markLocalizationLost(reason);
}

void MCL3dlNode::cbGlobalLocalization(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  requestGlobalLocalization("global localization requested by service");
  response->success = true;
  response->message =
      "Global localization scheduled; motion remains blocked until TRACKING";
}

void MCL3dlNode::publishLocalizationStatus()
{
  if (!pub_localization_status_)
  {
    return;
  }
  std_msgs::msg::String status;
  {
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    status.data = localization_state_machine_->stateName();
  }
  pub_localization_status_->publish(status);
}

void MCL3dlNode::publishLocalizationStatusThread()
{
  const int64_t now_ns = clock_->now().nanoseconds();
  const LocalizationState state = localizationState();
  if (state == LocalizationState::TRACKING || state == LocalizationState::LOCALIZING)
  {
    const int64_t feature_ns = last_feature_received_ns_.load();
    const int64_t odom_ns = last_odom_received_ns_.load();
    const bool feature_stale = state == LocalizationState::TRACKING ?
        (feature_ns == 0 || static_cast<double>(now_ns - feature_ns) / 1e9 >
          params_->localization_sensor_timeout_sec_) :
        (feature_ns != 0 && static_cast<double>(now_ns - feature_ns) / 1e9 >
          params_->localization_sensor_timeout_sec_);
    const bool odom_stale = state == LocalizationState::TRACKING ?
        (odom_ns == 0 || static_cast<double>(now_ns - odom_ns) / 1e9 >
          params_->localization_sensor_timeout_sec_) :
        (odom_ns != 0 && static_cast<double>(now_ns - odom_ns) / 1e9 >
          params_->localization_sensor_timeout_sec_);
    if (feature_stale || odom_stale)
    {
      markLocalizationLost(feature_stale ?
          "lidar feature stream timed out" : "odometry stream timed out");
      return;
    }

    if (state == LocalizationState::LOCALIZING)
    {
      const int64_t started_ns = localizing_started_ns_.load();
      if (started_ns != 0 &&
          static_cast<double>(now_ns - started_ns) / 1e9 >
            params_->localization_timeout_sec_)
      {
        markLocalizationLost("localization convergence timed out");
        return;
      }
    }
  }
  publishLocalizationStatus();
}

void MCL3dlNode::publishTFThread()
{
  if (tf_ready_.load() && isTracking() && params_->publish_tf_){
    if(laser_header_.stamp.sec==0 && laser_header_.stamp.nanosec == 0){
      laser_header_.stamp = odom_header_.stamp;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 3000, "Laser msg.header.timestamp = 0, use odom stamp as the timestamp");
    }
    map2odom_trans_.header.stamp = laser_header_.stamp;
    tfb_->sendTransform(map2odom_trans_);
    first_tf_.store(true);
  }
}

void MCL3dlNode::cbPosition(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
  const double len2 =
      msg->pose.pose.orientation.x * msg->pose.pose.orientation.x +
      msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
      msg->pose.pose.orientation.z * msg->pose.pose.orientation.z +
      msg->pose.pose.orientation.w * msg->pose.pose.orientation.w;
  if (std::abs(len2 - 1.0) > 0.1)
  {
    RCLCPP_ERROR(this->get_logger(), "Discarded invalid initialpose. The orientation must be unit quaternion.");
    return;
  }
  
  if(msg->header.frame_id!=params_->frame_ids_["map"]){
    RCLCPP_ERROR(this->get_logger(), "Discarded initialpose. The frame_id should be: %s", params_->frame_ids_["map"].c_str());
    return;
  }

  {
    std::unique_lock<mcl_3dl::SubMaps::sub_maps_mutex_t> lock(*(sub_maps_->getMutex()));
    sub_maps_->setInitialPose(*msg);
  }
  
  //@Try to find the ground
  geometry_msgs::msg::PoseStamped pose;
  mcl_3dl::pcl_t initial_pose_pt;
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  initial_pose_pt.x = msg->pose.pose.position.x;
  initial_pose_pt.y = msg->pose.pose.position.y;
  initial_pose_pt.z = msg->pose.pose.position.z;
  pose.pose.position.x = initial_pose_pt.x;
  pose.pose.position.y = initial_pose_pt.y;
  pose.pose.position.z = initial_pose_pt.z;
  pose.pose.orientation = msg->pose.pose.orientation;

  pcl::KdTreeFLANN<mcl_3dl::pcl_t>* ground_tree = nullptr;
  if (sub_maps_->isGlobalReady())
  {
    ground_tree = &sub_maps_->kdtree_ground_global_;
  }
  else if (sub_maps_->isCurrentReady())
  {
    ground_tree = &sub_maps_->kdtree_ground_current_;
  }

  if (ground_tree != nullptr)
  {
    const auto ground_cloud = ground_tree->getInputCloud();
    bool ground_found = false;
    for (double z = 0.0; z < 5.0 && !ground_found; z += 0.1)
    {
      for (const double direction : {1.0, -1.0})
      {
        initial_pose_pt.z = msg->pose.pose.position.z + direction * z;
        pointIdxRadiusSearch.clear();
        pointRadiusSquaredDistance.clear();
        if (ground_tree->radiusSearch(
              initial_pose_pt, 0.3, pointIdxRadiusSearch,
              pointRadiusSquaredDistance, 1) > 0)
        {
          pose.pose.position.z = ground_cloud->points[pointIdxRadiusSearch[0]].z;
          RCLCPP_INFO(this->get_logger(), "Found ground at z: %.2f", pose.pose.position.z);
          ground_found = true;
          break;
        }
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "Set initial pose at: %.2f, %.2f, %.2f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  const State6DOF mean(Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                        Quat(pose.pose.orientation.x,
                            pose.pose.orientation.y,
                            pose.pose.orientation.z,
                            pose.pose.orientation.w));
  const MultivariateNoiseGenerator<float> noise_gen(mean, msg->pose.covariance);
  if (static_cast<int>(pf_->getParticleSize()) != params_->num_particles_)
  {
    pf_->resizeParticle(params_->num_particles_);
  }
  pf_->initUsingNoiseGenerator(noise_gen);

  auto integ_reset_func = [](State6DOF& s)
  {
    s.odom_err_integ_lin_ = Vec3();
    s.odom_err_integ_ang_ = Vec3();
  };
  pf_->predict(integ_reset_func);

  state_prev_ = mean;
  global_localization_requested_.store(false);
  use_global_map_.store(sub_maps_->isGlobalReady());
  last_measure_ns_.store(0);
  startLocalizing("manual initial pose received");
  publishParticles();
  first_tf_.store(false);
}

/*
void MCL3dlNode::cbLandmark(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  NormalLikelihoodNd<float, 6> nd(
      Eigen::Matrix<double, 6, 6>(
          msg->pose.covariance.data())
          .cast<float>());
  const State6DOF measured(
      Vec3(msg->pose.pose.position.x,
            msg->pose.pose.position.y,
            msg->pose.pose.position.z),
      Quat(msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w));
  auto measure_func = [this, &measured, &nd](const State6DOF& s) -> float
  {
    State6DOF diff = s - measured;
    const Vec3 rot_rpy = diff.rot_.getRPY();
    const Eigen::Matrix<float, 6, 1> diff_vec =
        (Eigen::MatrixXf(6, 1) << diff.pos_.x_,
          diff.pos_.y_,
          diff.pos_.z_,
          rot_rpy.x_,
          rot_rpy.y_,
          rot_rpy.z_)
            .finished();

    const auto n = nd(diff_vec);
    return n;
  };
  pf_->measure(measure_func);

  pf_->resample(State6DOF(
      Vec3(params_->resample_var_x_,
            params_->resample_var_y_,
            params_->resample_var_z_),
      Vec3(params_->resample_var_roll_,
            params_->resample_var_pitch_,
            params_->resample_var_yaw_)));

  publishParticles();
}
*/
/*
bool cbResizeParticle(mcl_3dl_msgs::ResizeParticleRequest& request,
                      mcl_3dl_msgs::ResizeParticleResponse& response)
{
  pf_->resizeParticle(request.size);
  publishParticles();
  return true;
}
*/
/*
bool MCL3dlNode::cbExpansionReset(std_srvs::TriggerRequest& request,
                      std_srvs::TriggerResponse& response)
{
  pf_->noise(State6DOF(
      Vec3(params_.expansion_var_x_,
            params_.expansion_var_y_,
            params_.expansion_var_z_),
      Vec3(params_.expansion_var_roll_,
            params_.expansion_var_pitch_,
            params_.expansion_var_yaw_)));
  publishParticles();
  return true;
}
*/




/*
void MCL3dlNode::normal2quaternion(pcl::PointCloud<pcl::PointNormal>::Ptr i_normals){

  visualization_msgs::MarkerArray markerArray;
  for(size_t i=0;i<i_normals->points.size();i++){

    tf2::Vector3 axis_vector(i_normals->points[i].normal_x, i_normals->points[i].normal_y, i_normals->points[i].normal_z);

    tf2::Vector3 up_vector(1.0, 0.0, 0.0);
    tf2::Vector3 right_vector = axis_vector.cross(up_vector);
    right_vector.normalized();
    tf2::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
    q.normalize();

    //@Create arrow
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = i_normals->header.frame_id;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = i;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::ARROW;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = i_normals->points[i].x;
    marker.pose.position.y = i_normals->points[i].y;
    marker.pose.position.z = i_normals->points[i].z;
    marker.pose.orientation.x = q.getX();
    marker.pose.orientation.y = q.getY();
    marker.pose.orientation.z = q.getZ();
    marker.pose.orientation.w = q.getW();

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3; //scale.x is the arrow length,
    marker.scale.y = 0.05; //scale.y is the arrow width 
    marker.scale.z = 0.1; //scale.z is the arrow height. 

    double angle = atan2(i_normals->points[i].normal_z, 
                  sqrt(i_normals->points[i].normal_x*i_normals->points[i].normal_x+ i_normals->points[i].normal_y*i_normals->points[i].normal_y) ) * 180 / 3.1415926535;

    if(fabs(angle)<=10){
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.0f;      
    }
    else{
      marker.color.r = 0.0f;
      marker.color.g = 0.8f;
      marker.color.b = 0.2f; 
    }

    marker.color.a = 0.6f;   
    markerArray.markers.push_back(marker); 
  }
  pub_pc_normal_.publish(markerArray);
}
*/




}  // namespace mcl_3dl
