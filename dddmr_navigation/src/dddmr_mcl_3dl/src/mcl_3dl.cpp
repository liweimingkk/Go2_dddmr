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

#include <iomanip>
#include <sstream>

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
  , latest_match_ratio_(0.0f)
  , latest_residual_(std::numeric_limits<float>::infinity())
  , feature_sequence_(0)
  , last_measured_feature_sequence_(0)
  , feature_sync_count_(0)
  , feature_processing_count_(0)
  , feature_processing_total_ns_(0)
  , feature_last_header_age_ns_(0)
  , local_recovery_pending_(false)
  , operator_global_confirmed_(false)
  , operator_initialization_confirmed_(false)
{
  for (auto& count : feature_received_counts_)
  {
    count.store(0);
  }
  for (auto& stamp : feature_last_received_ns_)
  {
    stamp.store(0);
  }
  feature_metric_previous_received_.fill(0);
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
  lidar_measurements_->setFlatGroundConfig(
      params_->flat_ground_enabled_, params_->flat_ground_base_link_height_);

  LocalizationStateConfig state_config;
  state_config.tracking_match_ratio = params_->localization_tracking_match_ratio_;
  state_config.lost_match_ratio = params_->localization_lost_match_ratio_;
  state_config.tracking_max_xy_std = params_->localization_tracking_max_xy_std_;
  state_config.tracking_max_z_std = params_->localization_tracking_max_z_std_;
  state_config.tracking_max_slope_normal_std =
      params_->localization_tracking_max_slope_normal_std_;
  state_config.tracking_max_roll_std = params_->localization_tracking_max_roll_std_;
  state_config.tracking_max_pitch_std = params_->localization_tracking_max_pitch_std_;
  state_config.tracking_max_yaw_std = params_->localization_tracking_max_yaw_std_;
  state_config.tracking_max_residual = params_->localization_tracking_max_residual_;
  state_config.tracking_max_map_odom_tilt = params_->localization_tracking_max_map_odom_tilt_;
  state_config.tracking_max_ground_normal_error =
      params_->localization_tracking_max_ground_normal_error_;
  state_config.tracking_max_base_height_error =
      params_->localization_tracking_max_base_height_error_;
  state_config.tracking_max_pose_height_error =
      params_->localization_tracking_max_pose_height_error_;
  state_config.lost_max_xy_std = params_->localization_lost_max_xy_std_;
  state_config.lost_max_z_std = params_->localization_lost_max_z_std_;
  state_config.lost_max_slope_normal_std =
      params_->localization_lost_max_slope_normal_std_;
  state_config.lost_max_roll_std = params_->localization_lost_max_roll_std_;
  state_config.lost_max_pitch_std = params_->localization_lost_max_pitch_std_;
  state_config.lost_max_yaw_std = params_->localization_lost_max_yaw_std_;
  state_config.lost_max_residual = params_->localization_lost_max_residual_;
  state_config.lost_max_map_odom_tilt = params_->localization_lost_max_map_odom_tilt_;
  state_config.lost_max_ground_normal_error =
      params_->localization_lost_max_ground_normal_error_;
  state_config.lost_max_base_height_error =
      params_->localization_lost_max_base_height_error_;
  state_config.lost_max_pose_height_error =
      params_->localization_lost_max_pose_height_error_;
  state_config.require_ground_health = params_->localization_require_ground_health_;
  state_config.require_operator_initialization =
      params_->localization_require_operator_initialization_;
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
  sensor_heartbeat_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  
  //@Initialize transform listener and broadcaster
  tfbuf_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
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
  tfbuf_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tfbuf_);
  tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 2,
      std::bind(&MCL3dlNode::cbOdom, this, std::placeholders::_1), sub_options);

  // Keep transport freshness independent from particle-filter computation.
  // cbOdom and cbLeGoFeatureCloud intentionally serialize expensive state
  // updates; on embedded CPUs either callback can hold that lock longer than
  // localization_sensor_timeout_sec even while DDS inputs remain healthy.
  // These callbacks only record arrival and run in a dedicated reentrant
  // group.  Processed-pose freshness is still enforced separately by the
  // command gate's /mcl_pose timeout.
  rclcpp::SubscriptionOptions heartbeat_options;
  heartbeat_options.callback_group = sensor_heartbeat_group_;
  sub_odom_heartbeat_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr) {
        last_odom_received_ns_.store(clock_->now().nanoseconds());
      },
      heartbeat_options);
  const std::array<std::string, 4> feature_topics = {
      "laser_cloud_sharp", "laser_cloud_less_sharp",
      "laser_cloud_flat", "laser_cloud_less_flat"};
  for (std::size_t index = 0; index < feature_topics.size(); ++index)
  {
    sub_feature_heartbeats_[index] =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
        feature_topics[index], rclcpp::SensorDataQoS(),
        [this, index](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          recordFeatureMetrics(index, msg);
        },
        heartbeat_options);
  }

  sub_position_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initial_3d_pose", 2,
      std::bind(&MCL3dlNode::cbPosition, this, std::placeholders::_1), sub_options);

  // Match mcl_feature's live latest-state SensorDataQoS.  A reliable
  // subscription would be incompatible with its best-effort writers and
  // would also reintroduce stale feature backlog during global localization.
  lc_sharp_.subscribe(this, "laser_cloud_sharp", rmw_qos_profile_sensor_data);
  lc_less_sharp_.subscribe(
      this, "laser_cloud_less_sharp", rmw_qos_profile_sensor_data);
  lc_flat_.subscribe(this, "laser_cloud_flat", rmw_qos_profile_sensor_data);
  lc_less_flat_.subscribe(
      this, "laser_cloud_less_flat", rmw_qos_profile_sensor_data);
  
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
  pub_localization_health_ = this->create_publisher<std_msgs::msg::String>(
      "localization_health",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  pub_localization_quality_ =
      this->create_publisher<std_msgs::msg::Float32>("localization_quality", 1);
  pub_localization_residual_ =
      this->create_publisher<std_msgs::msg::Float32>("localization_residual", 1);
  pub_feature_stream_metrics_ =
      this->create_publisher<std_msgs::msg::String>(
      "feature_stream_metrics",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

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

  // A fixed-pose task must never jump globally before its recorded
  // /initial_3d_pose arrives. Always start from the configured/local seed.
  // auto_global_localization only permits escalation after local recovery
  // fails; the service remains the explicit operator-confirmed path.
  startLocalizing(
    "configured initial pose; global search is blocked",
    LocalizationAttemptKind::CONFIGURED_INITIAL);
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
  const LocalizationStateSnapshot state_snapshot =
      localizationStateSnapshot();
  const LocalizationState state = state_snapshot.state;
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

  bool feature_stream_stale = false;
  for (const auto& received_ns : feature_last_received_ns_)
  {
    const int64_t stamp_ns = received_ns.load();
    if (stamp_ns == 0 ||
        static_cast<double>(now_ns - stamp_ns) / 1e9 >
          params_->localization_sensor_timeout_sec_)
    {
      feature_stream_stale = true;
      break;
    }
  }
  if (feature_stream_stale)
  {
    if (sensorTimeoutCausesLost(state))
    {
      markLocalizationLostIfCurrent(
        "one or more lidar feature topics timed out", state_snapshot);
    }
    else if (state == LocalizationState::LOCALIZING)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Lidar feature heartbeat is stale while LOCALIZING; motion remains "
        "blocked and the bounded convergence timer is still authoritative");
    }
    return;
  }

  bool just_seeded_globally = false;
  bool just_seeded_locally = false;
  LocalizationState current_state = localizationState();
  const bool should_global_localize = global_localization_requested_.load();
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
    // Global candidate scoring can take longer than the sensor timeout on
    // embedded CPUs.  The odometry callback is still actively consuming a
    // valid synchronized frame, so refresh both processing heartbeats before
    // exposing LOCALIZING to the independent status timer.  Without this, the
    // timer can immediately undo an accepted global localization using the
    // callback's pre-computation receipt time.
    const int64_t global_localization_complete_ns = clock_->now().nanoseconds();
    last_odom_received_ns_.store(global_localization_complete_ns);
    last_feature_received_ns_.store(global_localization_complete_ns);
    just_seeded_globally = true;
  }
  else if (current_state == LocalizationState::LOST &&
           local_recovery_pending_.load())
  {
    if (!startLocalRecovery())
    {
      return;
    }
    current_state = localizationState();
    just_seeded_locally = true;
  }
  else if (current_state == LocalizationState::UNINITIALIZED ||
           current_state == LocalizationState::LOST)
  {
    return;
  }

  if (!just_seeded_globally && !just_seeded_locally)
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

  const bool measurement_valid = measure(pcl_segmentations_);
  // A completed measurement proves that both inputs made forward processing
  // progress.  Keep timeout accounting based on completion rather than the
  // callback entry instant, which may precede an expensive particle update by
  // several seconds on Orin.
  const int64_t measurement_complete_ns = clock_->now().nanoseconds();
  last_odom_received_ns_.store(measurement_complete_ns);
  last_feature_received_ns_.store(measurement_complete_ns);
  if (!measurement_valid)
  {
    return;
  }
  // Prerequisite work before the first usable particle-filter result can take
  // longer than the bounded recovery window on Orin. Start convergence timing
  // only after a measurement completes. Motion remains blocked throughout
  // LOCALIZING.
  bool convergence_timer_armed = false;
  {
    std::lock_guard<std::mutex> state_lock(localization_state_mutex_);
    if (localization_state_machine_->state() == LocalizationState::LOCALIZING)
    {
      convergence_timer_armed =
        localization_convergence_timer_.armAfterCompletedMeasurement(
          measurement_complete_ns);
    }
  }
  if (convergence_timer_armed)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Localization convergence timeout armed after the first completed "
      "measurement");
  }
  last_measured_feature_sequence_.store(feature_sequence);
  last_measure_ns_.store(measurement_complete_ns);

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

  if (params_->flat_ground_enabled_)
  {
    const bool use_global_ground = sub_maps_->isGlobalReady() &&
        use_global_map_.load();
    if (use_global_ground)
    {
      constrainParticles2p5D(
          sub_maps_->kdtree_ground_global_, sub_maps_->normals_ground_global_);
    }
    else if (sub_maps_->isCurrentReady())
    {
      constrainParticles2p5D(
          sub_maps_->kdtree_ground_current_, sub_maps_->normals_ground_current_);
    }
  }

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

  const auto processing_started = std::chrono::steady_clock::now();
  const int64_t callback_now_ns = clock_->now().nanoseconds();
  feature_sync_count_.fetch_add(1);
  last_feature_received_ns_.store(callback_now_ns);
  const rclcpp::Time feature_stamp(pc_less_sharpMsg->header.stamp);
  if (feature_stamp.nanoseconds() > 0)
  {
    feature_last_header_age_ns_.store(
        std::max<int64_t>(0, callback_now_ns - feature_stamp.nanoseconds()));
  }
  const auto finish_processing = [this, processing_started]()
  {
    const int64_t duration_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - processing_started).count();
    feature_processing_total_ns_.fetch_add(duration_ns);
    feature_processing_count_.fetch_add(1);
  };

  std::unique_lock<std::mutex> lock(protect_measure_in_odomcb_);
  
  laser_header_ = pc_less_sharpMsg->header;

  if (!sub_maps_->isCurrentReady() && !sub_maps_->isGlobalReady())
  {
    finish_processing();
    return;
  }

  Eigen::Affine3d trans_b2s_af3;
  if(! getBaselink2SensorAF3(pc_less_sharpMsg->header, trans_b2s_af3))
  {
    finish_processing();
    return;
  }

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
  pcl::fromROSMsg(*pc_less_flatMsg, *pc_less_flat);

  //@Transform point cloud
  //pcl::transformPointCloud(*pc_sharp, *pc_sharp, trans_b2s_af3);
  pcl::transformPointCloud(*pc_less_sharp, *pc_less_sharp, trans_b2s_af3);
  pcl::transformPointCloud(*pc_flat, *pc_flat, trans_b2s_af3);
  pcl::transformPointCloud(*pc_less_flat, *pc_less_flat, trans_b2s_af3);

  observation_ground_ = estimateObservationGround(*pc_flat);
  
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
  // Dense surfaces are only used to disambiguate global candidates.
  pcl_segmentations_[std::string("less_flat")] = pc_less_flat;
  if (!pc_flat->empty() || !pc_less_sharp_intensity->empty())
  {
    last_feature_received_ns_.store(clock_->now().nanoseconds());
    feature_sequence_.fetch_add(1);
  }

  finish_processing();
}

MCL3dlNode::ObservationGround MCL3dlNode::estimateObservationGround(
    const pcl::PointCloud<mcl_3dl::pcl_t>& flat_cloud) const
{
  ObservationGround estimate;
  auto candidates = std::make_shared<pcl::PointCloud<mcl_3dl::pcl_t>>();
  candidates->reserve(flat_cloud.size());
  const double expected_ground_z = -params_->flat_ground_base_link_height_;
  for (const auto& point : flat_cloud.points)
  {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) &&
        std::abs(static_cast<double>(point.z) - expected_ground_z) <=
          params_->flat_ground_observation_height_tolerance_)
    {
      candidates->push_back(point);
    }
  }
  if (candidates->size() < static_cast<std::size_t>(params_->flat_ground_min_points_))
  {
    return estimate;
  }

  pcl::SACSegmentation<mcl_3dl::pcl_t> segmentation;
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setAxis(Eigen::Vector3f::UnitZ());
  segmentation.setEpsAngle(params_->flat_ground_observation_max_tilt_);
  segmentation.setDistanceThreshold(params_->flat_ground_observation_plane_distance_);
  segmentation.setMaxIterations(100);
  segmentation.setInputCloud(candidates);
  pcl::PointIndices inliers;
  pcl::ModelCoefficients coefficients;
  segmentation.segment(inliers, coefficients);
  if (inliers.indices.size() < static_cast<std::size_t>(params_->flat_ground_min_points_) ||
      coefficients.values.size() < 4)
  {
    return estimate;
  }

  Vec3 normal(
      coefficients.values[0], coefficients.values[1], coefficients.values[2]);
  double offset = coefficients.values[3];
  if (normal.norm() < 1e-6)
  {
    return estimate;
  }
  const double original_norm = normal.norm();
  normal /= original_norm;
  offset /= original_norm;
  if (normal.z_ < 0.0)
  {
    normal *= -1.0;
    offset *= -1.0;
  }
  if (normal.z_ < std::cos(params_->flat_ground_observation_max_tilt_))
  {
    return estimate;
  }

  estimate.base_height = offset / normal.z_;
  estimate.normal = normal;
  std::vector<double> distances;
  distances.reserve(inliers.indices.size());
  for (const int index : inliers.indices)
  {
    if (index < 0 || static_cast<std::size_t>(index) >= candidates->size())
    {
      continue;
    }
    const auto& point = candidates->points[static_cast<std::size_t>(index)];
    distances.push_back(std::abs(
        normal.x_ * point.x + normal.y_ * point.y + normal.z_ * point.z + offset));
  }
  estimate.roughness = median(distances);
  estimate.valid = std::isfinite(estimate.base_height) &&
      std::isfinite(estimate.roughness) && estimate.base_height > 0.0;
  return estimate;
}

bool MCL3dlNode::constrainState2p5D(
    State6DOF& state,
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& ground_tree,
    const pcl::PointCloud<pcl::Normal>& ground_normals) const
{
  if (!params_->flat_ground_enabled_)
  {
    return true;
  }
  if (!has_odom_)
  {
    return false;
  }

  const FlatGroundEstimate ground = estimateFlatGround(
      ground_tree, ground_normals, state.pos_.x_, state.pos_.y_,
      state.pos_.z_ - params_->flat_ground_base_link_height_,
      params_->flat_ground_search_radius_,
      static_cast<std::size_t>(params_->flat_ground_min_points_));
  if (!ground.valid)
  {
    return false;
  }

  const double desired_map_yaw = state.rot_.getRPY().z_;
  const double odom_yaw = odom_.rot_.getRPY().z_;
  const double map_to_odom_yaw = std::atan2(
      std::sin(desired_map_yaw - odom_yaw),
      std::cos(desired_map_yaw - odom_yaw));
  state.rot_ = Quat(Vec3(0.0, 0.0, map_to_odom_yaw)) * odom_.rot_;
  state.rot_.normalize();
  state.pos_.z_ = ground.z + params_->flat_ground_base_link_height_;
  return true;
}

void MCL3dlNode::constrainParticles2p5D(
    pcl::KdTreeFLANN<mcl_3dl::pcl_t>& ground_tree,
    const pcl::PointCloud<pcl::Normal>& ground_normals)
{
  if (!params_->flat_ground_enabled_)
  {
    return;
  }
  auto constrain = [this, &ground_tree, &ground_normals](State6DOF& state)
  {
    constrainState2p5D(state, ground_tree, ground_normals);
  };
  pf_->predict(constrain);
}

std::vector<MCL3dlNode::GlobalCandidate>
MCL3dlNode::buildGlobalCandidates() const
{
  struct PositionSeed
  {
    geometry_msgs::msg::Pose pose;
    std::size_t key_frame_index;
  };

  const auto key_poses = sub_maps_->getKeyPoses();
  std::vector<PositionSeed> position_seeds;
  position_seeds.reserve(key_poses.size());

  for (std::size_t key_frame_index = 0;
       key_frame_index < key_poses.size(); ++key_frame_index)
  {
    const auto& pose = key_poses[key_frame_index];
    if (position_seeds.empty())
    {
      position_seeds.push_back(PositionSeed{pose, key_frame_index});
      continue;
    }

    const auto& previous = position_seeds.back().pose.position;
    const double dx = pose.position.x - previous.x;
    const double dy = pose.position.y - previous.y;
    const double dz = pose.position.z - previous.z;
    const double distance = params_->flat_ground_enabled_ ?
        std::hypot(dx, dy) : std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance >=
        params_->global_localization_grid_)
    {
      position_seeds.push_back(PositionSeed{pose, key_frame_index});
    }
  }

  if (!key_poses.empty() && !position_seeds.empty())
  {
    const auto& last_selected = position_seeds.back().pose.position;
    const auto& last_key_pose = key_poses.back();
    const double dx = last_key_pose.position.x - last_selected.x;
    const double dy = last_key_pose.position.y - last_selected.y;
    const double dz = last_key_pose.position.z - last_selected.z;
    const double distance = params_->flat_ground_enabled_ ?
        std::hypot(dx, dy) : std::sqrt(dx * dx + dy * dy + dz * dz);
    if (distance > 1e-3)
    {
      position_seeds.push_back(
          PositionSeed{last_key_pose, key_poses.size() - 1});
    }
  }

  const std::size_t yaw_bins =
      static_cast<std::size_t>(params_->global_localization_div_yaw_);
  const std::size_t max_positions = std::max<std::size_t>(
      1, static_cast<std::size_t>(params_->global_localization_max_candidates_) /
        yaw_bins);
  if (position_seeds.size() > max_positions)
  {
    std::vector<PositionSeed> uniformly_sampled;
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
  std::vector<GlobalCandidate> candidates;
  candidates.reserve(position_seeds.size() * yaw_bins);
  for (const auto& seed : position_seeds)
  {
    const auto& pose = seed.pose;
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
      GlobalCandidate candidate;
      candidate.state = State6DOF(
          Vec3(pose.position.x, pose.position.y, pose.position.z),
          Quat(Vec3(roll, pitch, yaw)));
      candidate.key_frame_index = seed.key_frame_index;
      candidates.push_back(candidate);
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

  auto sparse_surface =
      std::make_shared<pcl::PointCloud<mcl_3dl::pcl_t>>();
  const auto surface = pcl_segmentations.find("less_flat");
  if (surface != pcl_segmentations.end() && surface->second)
  {
    const auto& input = *surface->second;
    const std::size_t limit = static_cast<std::size_t>(
        params_->global_localization_max_surface_points_);
    const std::size_t step = std::max<std::size_t>(
        1, static_cast<std::size_t>(std::ceil(
          static_cast<double>(input.size()) /
          static_cast<double>(limit))));
    sparse_surface->reserve(std::min(input.size(), limit));
    for (std::size_t i = 0;
         i < input.size() && sparse_surface->size() < limit; i += step)
    {
      sparse_surface->push_back(input.points[i]);
    }
  }
  sparse["less_flat"] = sparse_surface;
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
    if (params_->flat_ground_enabled_)
    {
      constrainState2p5D(
          particle->state_, sub_maps_->kdtree_ground_global_,
          sub_maps_->normals_ground_global_);
    }
    particle->probability_ = probability;
    particle->probability_bias_ = 1.0f;
    particle->accum_probability_ = 0.0f;
  }
}

State6DOF MCL3dlNode::odomContinuousPose() const
{
  if (!has_last_trusted_pose_.load())
  {
    return state_prev_;
  }
  Quat map_to_odom_rotation =
      last_trusted_state_.rot_ * last_trusted_odom_.rot_.inv();
  map_to_odom_rotation.normalize();
  const Vec3 map_to_odom_translation =
      last_trusted_state_.pos_ -
      map_to_odom_rotation * last_trusted_odom_.pos_;
  State6DOF expected(
      map_to_odom_translation + map_to_odom_rotation * odom_.pos_,
      map_to_odom_rotation * odom_.rot_);
  expected.normalize();
  return expected;
}

bool MCL3dlNode::startLocalRecovery()
{
  if (!has_last_trusted_pose_.load() || !sub_maps_->isCurrentReady())
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Local recovery is waiting for a last trusted pose and its current "
        "submap");
    return false;
  }

  State6DOF expected = odomContinuousPose();
  if (params_->flat_ground_enabled_ &&
      !constrainState2p5D(
        expected, sub_maps_->kdtree_ground_current_,
        sub_maps_->normals_ground_current_))
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Local recovery rejected: no trusted ground near odometry-continuous "
        "pose");
    return false;
  }

  if (static_cast<int>(pf_->getParticleSize()) != params_->num_particles_)
  {
    pf_->resizeParticle(params_->num_particles_);
  }
  pf_->init(expected, params_->local_recovery_std_);
  if (params_->flat_ground_enabled_)
  {
    constrainParticles2p5D(
        sub_maps_->kdtree_ground_current_,
        sub_maps_->normals_ground_current_);
  }
  state_prev_ = expected;
  startLocalizing(
    "local recovery around last trusted pose",
    LocalizationAttemptKind::LOCAL_RECOVERY);
  publishParticles();
  RCLCPP_WARN(
      this->get_logger(),
      "Local recovery seeded around odometry-continuous pose "
      "(%.2f, %.2f, %.2f); global search remains blocked",
      expected.pos_.x_, expected.pos_.y_, expected.pos_.z_);
  return true;
}

void MCL3dlNode::resetGlobalConfirmation()
{
  std::lock_guard<std::mutex> lock(global_confirmation_mutex_);
  has_global_confirmation_ = false;
  global_confirmation_count_ = 0;
  global_confirmation_feature_sequence_ = 0;
}

bool MCL3dlNode::confirmGlobalCandidate(
    const GlobalCandidate& candidate)
{
  std::lock_guard<std::mutex> lock(global_confirmation_mutex_);
  const uint64_t sequence = feature_sequence_.load();
  if (sequence == 0 || sequence == global_confirmation_feature_sequence_)
  {
    return false;
  }
  global_confirmation_feature_sequence_ = sequence;

  if (has_global_confirmation_)
  {
    const double dx =
        candidate.state.pos_.x_ - global_confirmation_state_.pos_.x_;
    const double dy =
        candidate.state.pos_.y_ - global_confirmation_state_.pos_.y_;
    const double yaw = candidate.state.rot_.getRPY().z_;
    const double previous_yaw = global_confirmation_state_.rot_.getRPY().z_;
    const double yaw_error = std::abs(std::atan2(
        std::sin(yaw - previous_yaw), std::cos(yaw - previous_yaw)));
    if (std::hypot(dx, dy) <=
          params_->global_localization_confirmation_max_xy_ &&
        yaw_error <= params_->global_localization_confirmation_max_yaw_)
    {
      ++global_confirmation_count_;
    }
    else
    {
      global_confirmation_count_ = 1;
    }
  }
  else
  {
    has_global_confirmation_ = true;
    global_confirmation_count_ = 1;
  }
  global_confirmation_state_ = candidate.state;

  if (global_confirmation_count_ <
      static_cast<std::size_t>(
        params_->global_localization_confirmation_frames_))
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Global candidate keyframe %zu held for confirmation %zu/%d; "
        "localization remains stopped",
        candidate.key_frame_index, global_confirmation_count_,
        params_->global_localization_confirmation_frames_);
    return false;
  }
  return true;
}

bool MCL3dlNode::attemptGlobalLocalization(
    const std::map<std::string, pcl::PointCloud<pcl_t>::Ptr>& pcl_segmentations)
{
  if (!sub_maps_->isGlobalReady() || !sub_maps_->areKeyFramesReady())
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Global localization is waiting for the complete static map and all "
        "ordered keyframes");
    return false;
  }
  if (!has_last_trusted_pose_.load() && !operator_global_confirmed_.load())
  {
    RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Automatic global localization rejected: no last trusted pose exists; "
        "operator confirmation is required");
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
  if (sparse_observation.at("less_flat")->empty())
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Global localization is waiting for a non-empty synchronized surface "
        "cloud");
    return false;
  }

  auto scored = buildGlobalCandidates();
  if (scored.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Global localization has no pose-graph candidates");
    return false;
  }

  const State6DOF odom_expected = odomContinuousPose();
  const bool enforce_odom_continuity = has_last_trusted_pose_.load();
  #pragma omp parallel for
  for (std::size_t i = 0; i < scored.size(); ++i)
  {
    State6DOF candidate = scored[i].state;
    if (!constrainState2p5D(
          candidate, sub_maps_->kdtree_ground_global_,
          sub_maps_->normals_ground_global_))
    {
      scored[i].state = candidate;
      continue;
    }
    const auto result = lidar_measurements_->measure(
        sub_maps_->kdtree_feature_global_, sub_maps_->kdtree_ground_global_,
        sub_maps_->normals_ground_global_, sparse_observation, candidate);
    scored[i].state = candidate;
    scored[i].quality = std::isfinite(result.quality) ? result.quality : 0.0f;
    scored[i].likelihood = std::isfinite(result.likelihood) ? result.likelihood : 0.0f;
    scored[i].residual =
        (!params_->flat_ground_enabled_ || result.ground_valid) &&
        std::isfinite(result.residual) ?
        result.residual : std::numeric_limits<float>::infinity();

    pcl::PointCloud<pcl_t> surface_in_map =
        *sparse_observation.at("less_flat");
    candidate.transform(surface_in_map);
    scored[i].surface_quality = sub_maps_->keyFrameSurfaceMatchRatio(
        scored[i].key_frame_index, surface_in_map,
        params_->global_localization_surface_match_distance_);
  }

  if (!sub_maps_->isGlobalReady() || !sub_maps_->areKeyFramesReady() ||
      !sub_maps_->isMapSnapshotValid())
  {
    resetGlobalConfirmation();
    RCLCPP_ERROR(
        this->get_logger(),
        "Global localization map generation changed during scoring; result "
        "rejected");
    return false;
  }

  const std::size_t evaluated_candidates = scored.size();
  const auto best_quality_it = std::max_element(
      scored.begin(), scored.end(),
      [](const GlobalCandidate& lhs, const GlobalCandidate& rhs)
      {
        return lhs.quality < rhs.quality;
      });
  const auto best_residual_it = std::min_element(
      scored.begin(), scored.end(),
      [](const GlobalCandidate& lhs, const GlobalCandidate& rhs)
      {
        return lhs.residual < rhs.residual;
      });
  const float best_observed_quality = best_quality_it->quality;
  const float best_observed_residual = best_residual_it->residual;
  scored.erase(
      std::remove_if(
          scored.begin(), scored.end(),
          [this, &odom_expected, enforce_odom_continuity](
              const GlobalCandidate& candidate)
          {
            if (candidate.quality <
                  params_->global_localization_min_match_ratio_ ||
                candidate.surface_quality <
                  params_->global_localization_min_surface_match_ratio_ ||
                candidate.residual >
                  params_->global_localization_max_residual_)
            {
              return true;
            }
            if (!enforce_odom_continuity)
            {
              return false;
            }
            const double dx =
                candidate.state.pos_.x_ - odom_expected.pos_.x_;
            const double dy =
                candidate.state.pos_.y_ - odom_expected.pos_.y_;
            const double candidate_yaw = candidate.state.rot_.getRPY().z_;
            const double expected_yaw = odom_expected.rot_.getRPY().z_;
            const double yaw_error = std::abs(std::atan2(
                std::sin(candidate_yaw - expected_yaw),
                std::cos(candidate_yaw - expected_yaw)));
            return std::hypot(dx, dy) >
                params_->global_localization_max_odom_xy_error_ ||
              yaw_error >
                params_->global_localization_max_odom_yaw_error_;
          }),
      scored.end());

  if (scored.empty())
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Global localization rejected %zu candidates: no candidate passed "
        "feature/surface/residual and odometry-continuity gates; best "
        "feature=%.3f residual=%.3f",
        evaluated_candidates, best_observed_quality,
        best_observed_residual);
    resetGlobalConfirmation();
    return false;
  }

  std::sort(scored.begin(), scored.end(),
      [](const GlobalCandidate& lhs, const GlobalCandidate& rhs)
      {
        if (lhs.surface_quality != rhs.surface_quality)
        {
          return lhs.surface_quality > rhs.surface_quality;
        }
        if (lhs.residual != rhs.residual)
        {
          return lhs.residual < rhs.residual;
        }
        if (lhs.quality != rhs.quality)
        {
          return lhs.quality > rhs.quality;
        }
        return lhs.likelihood > rhs.likelihood;
      });

  const auto& best = scored.front();
  constexpr double kPi = 3.14159265358979323846;
  const double basin_xy = std::max(
      0.50, 2.0 * params_->global_localization_grid_);
  const double basin_yaw = std::max(
      0.20, 3.0 * kPi /
        static_cast<double>(params_->global_localization_div_yaw_));
  const double best_yaw = best.state.rot_.getRPY().z_;
  const auto is_same_basin =
      [&best, best_yaw, basin_xy, basin_yaw](
          const GlobalCandidate& candidate)
      {
        const double dx = candidate.state.pos_.x_ - best.state.pos_.x_;
        const double dy = candidate.state.pos_.y_ - best.state.pos_.y_;
        const double yaw = candidate.state.rot_.getRPY().z_;
        const double yaw_error = std::abs(std::atan2(
            std::sin(yaw - best_yaw), std::cos(yaw - best_yaw)));
        return std::hypot(dx, dy) <= basin_xy && yaw_error <= basin_yaw;
      };
  float competitor_surface_quality = 0.0f;
  std::size_t competitor_key_frame = best.key_frame_index;
  for (std::size_t index = 1; index < scored.size(); ++index)
  {
    if (!is_same_basin(scored[index]))
    {
      competitor_surface_quality = scored[index].surface_quality;
      competitor_key_frame = scored[index].key_frame_index;
      break;
    }
  }
  const float candidate_margin =
      best.surface_quality - competitor_surface_quality;
  if (candidate_margin <
      params_->global_localization_min_surface_match_margin_)
  {
    resetGlobalConfirmation();
    RCLCPP_WARN(
        this->get_logger(),
        "Global localization rejected ambiguous first/second basins: "
        "keyframe %zu surface=%.3f, keyframe %zu surface=%.3f, "
        "margin=%.3f < %.3f",
        best.key_frame_index, best.surface_quality, competitor_key_frame,
        competitor_surface_quality, candidate_margin,
        params_->global_localization_min_surface_match_margin_);
    return false;
  }

  if (!confirmGlobalCandidate(best))
  {
    return false;
  }

  const float retained_surface_floor =
      best.surface_quality -
      params_->global_localization_surface_candidate_max_drop_;
  scored.erase(
      std::remove_if(
          scored.begin(), scored.end(),
          [retained_surface_floor, &is_same_basin](
              const GlobalCandidate& candidate)
          {
            return candidate.surface_quality < retained_surface_floor ||
              !is_same_basin(candidate);
          }),
      scored.end());
  scored.resize(std::min<std::size_t>(
      scored.size(), static_cast<std::size_t>(params_->global_localization_top_candidates_)));
  initializeGlobalParticles(scored);
  state_prev_ = scored.front().state;
  last_feature_received_ns_.store(clock_->now().nanoseconds());

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
      std::to_string(scored.front().quality) + " residual " +
      std::to_string(scored.front().residual),
      LocalizationAttemptKind::GLOBAL_RECOVERY);
  publishParticles();
  RCLCPP_WARN(
      this->get_logger(),
      "Global localization seeded %d particles after %d confirmed frames "
      "from %zu/%zu candidates; keyframe=%zu pose=(%.2f, %.2f, %.2f) "
      "feature=%.3f surface=%.3f residual=%.3f margin=%.3f",
      params_->global_localization_num_particles_,
      params_->global_localization_confirmation_frames_,
      scored.size(), evaluated_candidates, scored.front().key_frame_index,
      scored.front().state.pos_.x_, scored.front().state.pos_.y_,
      scored.front().state.pos_.z_, scored.front().quality,
      scored.front().surface_quality, scored.front().residual,
      candidate_margin);
  return true;
}

MCL3dlNode::ParticleSpread MCL3dlNode::particleSpread(
    const State6DOF& mean, const Vec3& surface_normal) const
{
  const Vec3 mean_rpy = mean.rot_.getRPY();
  PositionSpreadAccumulator position_spread(surface_normal);
  double roll_variance = 0.0;
  double pitch_variance = 0.0;
  double yaw_variance = 0.0;
  double probability_sum = 0.0;
  for (std::size_t i = 0; i < pf_->getParticleSize(); ++i)
  {
    const State6DOF particle = pf_->getParticle(i);
    const double probability = pf_->getParticleProbability(i);
    const double dx = particle.pos_.x_ - mean.pos_.x_;
    const double dy = particle.pos_.y_ - mean.pos_.y_;
    const double dz = particle.pos_.z_ - mean.pos_.z_;
    position_spread.add(Vec3(dx, dy, dz), probability);
    const Vec3 rpy = particle.rot_.getRPY();
    const auto angle_error = [](const double value, const double reference)
    {
      return std::atan2(std::sin(value - reference), std::cos(value - reference));
    };
    const double roll_error = angle_error(rpy.x_, mean_rpy.x_);
    const double pitch_error = angle_error(rpy.y_, mean_rpy.y_);
    const double yaw_error = angle_error(rpy.z_, mean_rpy.z_);
    roll_variance += probability * roll_error * roll_error;
    pitch_variance += probability * pitch_error * pitch_error;
    yaw_variance += probability * yaw_error * yaw_error;
    probability_sum += probability;
  }

  if (probability_sum <= 0.0)
  {
    return {};
  }
  const PositionSpread position = position_spread.spread();
  ParticleSpread spread;
  spread.xy = position.tangent;
  spread.z = position.normal;
  spread.raw_xy = position.raw_xy;
  spread.raw_z = position.raw_z;
  spread.normal = position_spread.normal();
  spread.roll = std::sqrt(std::max(0.0, roll_variance / probability_sum));
  spread.pitch = std::sqrt(std::max(0.0, pitch_variance / probability_sum));
  spread.yaw = std::sqrt(std::max(0.0, yaw_variance / probability_sum));
  return spread;
}

std::string MCL3dlNode::localizationHealthReason(
    const LocalizationObservation& observation) const
{
  const auto invalid_or_above = [](const double value, const double limit)
  {
    return !std::isfinite(value) || value > limit;
  };
  if (params_->localization_require_operator_initialization_ &&
      !observation.operator_initialization_confirmed)
  {
    return "OPERATOR_INITIALIZATION_REQUIRED";
  }
  if (!observation.particle_count_converged)
  {
    return "PARTICLES_NOT_CONVERGED";
  }
  if (!std::isfinite(observation.match_ratio) ||
      observation.match_ratio < params_->localization_tracking_match_ratio_)
  {
    return "MATCH_RATIO";
  }
  if (invalid_or_above(
        observation.residual, params_->localization_tracking_max_residual_))
  {
    return "FINAL_RESIDUAL";
  }
  if (invalid_or_above(observation.xy_std, params_->localization_tracking_max_xy_std_))
  {
    return "PARTICLE_XY_SPREAD";
  }
  const double normal_std_limit = observation.slope_compensated ?
      params_->localization_tracking_max_slope_normal_std_ :
      params_->localization_tracking_max_z_std_;
  if (invalid_or_above(observation.z_std, normal_std_limit))
  {
    return "PARTICLE_Z_SPREAD";
  }
  if (invalid_or_above(
        observation.roll_std, params_->localization_tracking_max_roll_std_))
  {
    return "PARTICLE_ROLL_SPREAD";
  }
  if (invalid_or_above(
        observation.pitch_std, params_->localization_tracking_max_pitch_std_))
  {
    return "PARTICLE_PITCH_SPREAD";
  }
  if (invalid_or_above(observation.yaw_std, params_->localization_tracking_max_yaw_std_))
  {
    return "PARTICLE_YAW_SPREAD";
  }
  if (params_->localization_require_ground_health_ && !observation.ground_valid)
  {
    return "GROUND_UNAVAILABLE";
  }
  if (invalid_or_above(
        observation.map_odom_tilt, params_->localization_tracking_max_map_odom_tilt_))
  {
    return "MAP_ODOM_TILT";
  }
  if (invalid_or_above(
        observation.ground_normal_error,
        params_->localization_tracking_max_ground_normal_error_))
  {
    return "GROUND_NORMAL";
  }
  if (invalid_or_above(
        observation.base_height_error,
        params_->localization_tracking_max_base_height_error_))
  {
    return "BASE_HEIGHT";
  }
  if (invalid_or_above(
        observation.pose_height_error,
        params_->localization_tracking_max_pose_height_error_))
  {
    return "POSE_HEIGHT";
  }
  return "HEALTHY";
}

bool MCL3dlNode::measure(
    const std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr>& pcl_segmentations)
{
  if (sub_maps_->isWarmUpReady())
  {
    sub_maps_->swapKdTree();
    if (isTracking() ||
        switchesToCurrentMapAfterWarmup(
          localization_attempt_kind_.load()))
    {
      use_global_map_.store(false);
    }
  }

  const bool use_global =
      sub_maps_->isGlobalReady() && use_global_map_.load();
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

  constrainParticles2p5D(ground_tree, ground_normals);

  const auto ts = std::chrono::high_resolution_clock::now();
  lidar_measurements_->setGlobalLocalizationStatus(
      params_->num_particles_, pf_->getParticleSize());

  auto measure_func = [this, &pcl_segmentations, &map_tree, &ground_tree,
                        &ground_normals](const State6DOF& s) -> float
  {
    const LidarMeasurementResult result = lidar_measurements_->measure(
        map_tree, ground_tree, ground_normals, pcl_segmentations, s);
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
  const bool final_ground_constraint =
      constrainState2p5D(e, ground_tree, ground_normals);

  assert(std::isfinite(e.pos_.x_));
  assert(std::isfinite(e.pos_.y_));
  assert(std::isfinite(e.pos_.z_));
  assert(std::isfinite(e.rot_.x_));
  assert(std::isfinite(e.rot_.y_));
  assert(std::isfinite(e.rot_.z_));
  assert(std::isfinite(e.rot_.w_));

  e.rot_.normalize();

  // Evaluate the actual pose that will be published. Previously TRACKING used
  // the maximum hit ratio seen on any particle, which could hide a poor final
  // weighted pose or a wrong global-search basin.
  const LidarMeasurementResult final_result = lidar_measurements_->measure(
      map_tree, ground_tree, ground_normals, pcl_segmentations, e);

  Vec3 map_pos;
  Quat map_rot;
  map_pos = e.pos_ - e.rot_ * odom_.rot_.inv() * odom_.pos_;
  map_rot = e.rot_ * odom_.rot_.inv();
  double raw_map_odom_tilt = 0.0;
  if (params_->flat_ground_enabled_)
  {
    const Vec3 raw_map_up = map_rot * Vec3(0.0, 0.0, 1.0);
    raw_map_odom_tilt = std::acos(std::clamp(
        static_cast<double>(
          raw_map_up.z_ / std::max(1e-9f, raw_map_up.norm())),
        -1.0, 1.0));
  }

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

  
  geometry_msgs::msg::TransformStamped next_map2odom;
  next_map2odom.header.stamp = laser_header_.stamp;
  next_map2odom.header.frame_id = params_->frame_ids_["map"];
  next_map2odom.child_frame_id = params_->frame_ids_["odom"];
  Vec3 rpy = map_rot.getRPY();
  if (params_->flat_ground_enabled_)
  {
    rpy.x_ = 0.0;
    rpy.y_ = 0.0;
  }
  if (jump)
  {
    f_ang_->set(rpy);
    f_pos_->set(map_pos);
  }
  Vec3 filtered_rpy = f_ang_->in(rpy);
  if (params_->flat_ground_enabled_)
  {
    filtered_rpy.x_ = 0.0;
    filtered_rpy.y_ = 0.0;
  }
  map_rot.setRPY(filtered_rpy);
  map_pos = f_pos_->in(map_pos);
  next_map2odom.transform.translation.x = map_pos.x_;
  next_map2odom.transform.translation.y = map_pos.y_;
  next_map2odom.transform.translation.z = map_pos.z_;
  next_map2odom.transform.rotation =
      tf2::toMsg(tf2::Quaternion(map_rot.x_, map_rot.y_, map_rot.z_, map_rot.w_));
  {
    std::lock_guard<std::mutex> lock(tf_pub_mutex_);
    map2odom_trans_ = next_map2odom;
  }

  // Calculate covariance from sampled particles to reduce calculation cost on global localization.
  // Use the number of original particles or at least 10% of full particles.
  auto cov = pf_->covariance(
      1.0,
      std::max(
          0.1f, static_cast<float>(params_->num_particles_) / pf_->getParticleSize()));

  const Vec3 map_ground_normal = normalizedSurfaceNormalOrUp(
      final_result.ground_normal);
  const double slope_tilt = surfaceTiltFromUp(final_result.ground_normal);
  const bool slope_compensation_active = slopeCompensationActive(
      params_->localization_slope_compensation_enabled_,
      params_->flat_ground_enabled_, final_ground_constraint,
      final_result.ground_valid, final_result.ground_normal,
      params_->localization_slope_min_tilt_);
  const Vec3 spread_normal = slope_compensation_active ?
      map_ground_normal : Vec3(0.0, 0.0, 1.0);
  const auto spread = particleSpread(e, spread_normal);
  const float final_match_ratio =
      std::isfinite(final_result.quality) ? final_result.quality : 0.0f;
  const float capped_residual = std::isfinite(final_result.residual) ?
      final_result.residual : std::numeric_limits<float>::infinity();
  const float matched_residual = std::isfinite(final_result.matched_residual) ?
      final_result.matched_residual : std::numeric_limits<float>::infinity();
  const float final_residual = slope_compensation_active ?
      matched_residual : capped_residual;
  latest_match_ratio_.store(final_match_ratio);
  latest_residual_.store(final_residual);
  std_msgs::msg::Float32 quality_msg;
  quality_msg.data = final_match_ratio;
  pub_localization_quality_->publish(quality_msg);
  std_msgs::msg::Float32 residual_msg;
  residual_msg.data = final_residual;
  pub_localization_residual_->publish(residual_msg);

  LocalizationObservation observation;
  observation.match_ratio = final_match_ratio;
  observation.xy_std = spread.xy;
  observation.z_std = spread.z;
  observation.roll_std = spread.roll;
  observation.pitch_std = spread.pitch;
  observation.yaw_std = spread.yaw;
  observation.residual = final_residual;
  observation.particle_count_converged =
      static_cast<int>(pf_->getParticleSize()) <= params_->num_particles_;

  if (params_->flat_ground_enabled_)
  {
    // Check the estimator output before the published transform is projected
    // to yaw-only; otherwise this safety metric would be zero by construction.
    observation.map_odom_tilt = raw_map_odom_tilt;
    observation.ground_valid = final_ground_constraint && final_result.ground_valid &&
        observation_ground_.valid;
    if (observation.ground_valid)
    {
      Vec3 observed_normal_map = e.rot_ * observation_ground_.normal;
      observed_normal_map = observed_normal_map.normalized();
      Vec3 map_ground_normal = final_result.ground_normal.normalized();
      if (observed_normal_map.z_ < 0.0)
      {
        observed_normal_map *= -1.0;
      }
      if (map_ground_normal.z_ < 0.0)
      {
        map_ground_normal *= -1.0;
      }
      observation.ground_normal_error = std::acos(std::clamp(
          static_cast<double>(observed_normal_map.dot(map_ground_normal)), -1.0, 1.0));
      observation.base_height_error = std::abs(
          observation_ground_.base_height - params_->flat_ground_base_link_height_);
      observation.pose_height_error = std::abs(
          static_cast<double>(e.pos_.z_) -
          (static_cast<double>(final_result.ground_z) +
           params_->flat_ground_base_link_height_));
    }
    else
    {
      observation.ground_normal_error = std::numeric_limits<double>::infinity();
      observation.base_height_error = std::numeric_limits<double>::infinity();
      observation.pose_height_error = std::numeric_limits<double>::infinity();
    }
  }
  else
  {
    observation.ground_valid = true;
  }
  observation.slope_compensated = slope_compensation_active;
  observation.operator_initialization_confirmed =
      operator_initialization_confirmed_.load();

  const double tracking_normal_std_limit = slope_compensation_active ?
      params_->localization_tracking_max_slope_normal_std_ :
      params_->localization_tracking_max_z_std_;

  const std::string health_reason = localizationHealthReason(observation);
  if (health_reason != "HEALTHY")
  {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *clock_, 3000,
        "Localization health %s: match=%.3f residual=%.3f "
        "capped_residual=%.3f matched_residual=%.3f "
        "slope_compensated=%d slope_tilt=%.3f "
        "tangent_normal_std=%.3f/%.3f normal_std_limit=%.3f "
        "raw_xyz_std=%.3f/%.3f "
        "spread_normal=%.3f/%.3f/%.3f "
        "rpy_std=%.3f/%.3f/%.3f map_odom_tilt=%.3f ground_valid=%d "
        "ground_normal_error=%.3f observed_base_height=%.3f configured_base_height=%.3f "
        "base_height_error=%.3f pose_height_error=%.3f map_ground_z=%.3f pose_z=%.3f",
        health_reason.c_str(), observation.match_ratio, observation.residual,
        capped_residual, matched_residual, slope_compensation_active, slope_tilt,
        observation.xy_std, observation.z_std, tracking_normal_std_limit,
        spread.raw_xy, spread.raw_z,
        spread.normal.x_, spread.normal.y_, spread.normal.z_,
        observation.roll_std, observation.pitch_std, observation.yaw_std,
        observation.map_odom_tilt,
        observation.ground_valid, observation.ground_normal_error,
        observation_ground_.base_height, params_->flat_ground_base_link_height_,
        observation.base_height_error, observation.pose_height_error,
        final_result.ground_z, e.pos_.z_);
  }

  geometry_msgs::msg::PoseWithCovarianceStamped pose;
  pose.header.stamp = odom_last_;
  pose.header.frame_id = params_->frame_ids_["map"];
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
  pose.pose.covariance[35] = spread.yaw * spread.yaw;

  bool state_changed = false;
  bool auto_global_recovery = false;
  LocalizationState new_state;
  LocalizationAttemptKind completed_attempt =
    LocalizationAttemptKind::NONE;
  {
    // The status timer runs in another callback group. Keep the state
    // transition, TF gate, recovery flags, trusted pose, and submap update in
    // one transaction so a concurrent LOST transition cannot be overwritten
    // by a cached TRACKING result.
    std::lock_guard<std::mutex> state_lock(localization_state_mutex_);
    if (localization_state_machine_->state() != LocalizationState::LOST)
    {
      localization_health_ = health_reason;
      state_changed = localization_state_machine_->observe(observation);
    }
    new_state = localization_state_machine_->state();

    if (state_changed && new_state == LocalizationState::TRACKING)
    {
      localization_attempt_kind_.store(LocalizationAttemptKind::NONE);
      localization_convergence_timer_.reset();
      localization_state_reason_ =
          "particle filter converged on consecutive observations";
    }
    else if (state_changed && new_state == LocalizationState::LOST)
    {
      completed_attempt = localization_attempt_kind_.load();
      const std::string lost_reason =
          "final pose residual, geometry health, or particle spread exceeded "
          "lost thresholds";
      markLocalizationLostLocked(lost_reason);
      // Preserve the diagnostic that caused the observation-driven
      // transition; generic externally-triggered LOST transitions use
      // NOT_TRACKING instead.
      localization_health_ = health_reason;
      resetGlobalConfirmation();
      if (usesRecoveryTimeout(completed_attempt))
      {
        local_recovery_pending_.store(false);
        if (params_->auto_global_localization_)
        {
          global_localization_requested_.store(true);
          last_global_attempt_ns_.store(0);
          auto_global_recovery = true;
        }
      }
    }

    if (new_state == LocalizationState::TRACKING)
    {
      tf_ready_.store(true);
      has_last_trusted_pose_.store(true);
      last_trusted_state_ = e;
      last_trusted_odom_ = odom_;
      local_recovery_pending_.store(false);
      global_localization_requested_.store(false);
      operator_global_confirmed_.store(false);
      resetGlobalConfirmation();
      std::unique_lock<mcl_3dl::SubMaps::sub_maps_mutex_t> lock(
        *(sub_maps_->getMutex()));
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
    else
    {
      tf_ready_.store(false);
    }

    if (new_state != LocalizationState::LOST)
    {
      pub_pose_->publish(pose);
    }
  }

  if (state_changed)
  {
    RCLCPP_WARN(
        this->get_logger(),
        "Localization state changed to %s: match=%.3f residual=%.3f "
        "capped_residual=%.3f matched_residual=%.3f "
        "slope_compensated=%d slope_tilt=%.3f "
        "tangent_normal_std=%.3f/%.3f normal_std_limit=%.3f "
        "raw_xyz_std=%.3f/%.3f "
        "spread_normal=%.3f/%.3f/%.3f "
        "rpy_std=%.3f/%.3f/%.3f map_odom_tilt=%.3f "
        "ground_normal_error=%.3f base_height_error=%.3f pose_height_error=%.3f "
        "health=%s particles=%lu",
        localizationStateName(new_state), final_match_ratio, final_residual,
        capped_residual, matched_residual, slope_compensation_active, slope_tilt,
        spread.xy, spread.z, tracking_normal_std_limit,
        spread.raw_xy, spread.raw_z,
        spread.normal.x_, spread.normal.y_, spread.normal.z_,
        spread.roll, spread.pitch, spread.yaw,
        observation.map_odom_tilt, observation.ground_normal_error,
        observation.base_height_error, observation.pose_height_error,
        health_reason.c_str(), pf_->getParticleSize());
    if (auto_global_recovery)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Local recovery failed; automatic global recovery is now "
          "permitted but remains candidate-gated");
    }
    publishLocalizationStatus();
  }

  const auto tnow = std::chrono::high_resolution_clock::now();
  RCLCPP_DEBUG(this->get_logger(), "MCL (%0.3f sec.)",
            std::chrono::duration<float>(tnow - ts).count());
  RCLCPP_DEBUG(this->get_logger(),
            "final pose match ratio: %0.3f residual: %0.3f pos: %0.3f, %0.3f, %0.3f",
            final_match_ratio,
            final_residual,
            e.pos_.x_,
            e.pos_.y_,
            e.pos_.z_);

  if (new_state != LocalizationState::LOST &&
      final_match_ratio < params_->match_ratio_thresh_)
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

  constrainParticles2p5D(ground_tree, ground_normals);

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

MCL3dlNode::LocalizationStateSnapshot
MCL3dlNode::localizationStateSnapshot() const
{
  std::lock_guard<std::mutex> lock(localization_state_mutex_);
  return LocalizationStateSnapshot{
    localization_state_machine_->state(),
    localization_convergence_timer_.generation()};
}

bool MCL3dlNode::isTracking() const
{
  return localizationState() == LocalizationState::TRACKING;
}

void MCL3dlNode::startLocalizing(
    const std::string& reason,
    const LocalizationAttemptKind attempt_kind)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    // The independent status timer must not commit an expiry from an older
    // seed after this attempt becomes current.
    localization_convergence_timer_.reset();
    localization_attempt_kind_.store(attempt_kind);
    changed = localization_state_machine_->startLocalizing();
    localization_state_reason_ = reason;
    localization_health_ = "NOT_TRACKING";
    tf_ready_.store(false);
    first_tf_.store(false);
    global_localization_requested_.store(false);
    operator_global_confirmed_.store(false);
    local_recovery_pending_.store(false);
    const bool use_global_map =
      attempt_kind == LocalizationAttemptKind::GLOBAL_RECOVERY ||
      (attempt_kind == LocalizationAttemptKind::FIXED_POSE &&
       sub_maps_->isGlobalReady());
    use_global_map_.store(use_global_map);
    resetGlobalConfirmation();
  }
  if (changed)
  {
    RCLCPP_WARN(this->get_logger(), "Localization state changed to LOCALIZING: %s", reason.c_str());
  }
  publishLocalizationStatus();
}

bool MCL3dlNode::markLocalizationLostLocked(const std::string& reason)
{
  const bool changed = localization_state_machine_->markLost();
  localization_state_reason_ = reason;
  localization_health_ = "NOT_TRACKING";
  localization_convergence_timer_.reset();
  localization_attempt_kind_.store(LocalizationAttemptKind::NONE);
  tf_ready_.store(false);
  use_global_map_.store(false);
  const bool operator_global = operator_global_confirmed_.load();
  if (!operator_global)
  {
    global_localization_requested_.store(false);
  }
  local_recovery_pending_.store(
    has_last_trusted_pose_.load() && !operator_global);
  return changed;
}

bool MCL3dlNode::markLocalizationLostIfCurrent(
    const std::string& reason,
    const LocalizationStateSnapshot& expected)
{
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    if (localization_state_machine_->state() != expected.state ||
        !localization_convergence_timer_.isGenerationCurrent(
          expected.generation))
    {
      return false;
    }
    changed = markLocalizationLostLocked(reason);
    resetGlobalConfirmation();
  }
  if (changed)
  {
    RCLCPP_ERROR(this->get_logger(), "Localization state changed to LOST: %s", reason.c_str());
  }
  publishLocalizationStatus();
  return true;
}

bool MCL3dlNode::markLocalizationLostIfConvergenceExpired(
    const LocalizationConvergenceTimer::Snapshot& timer_snapshot,
    const int64_t now_ns)
{
  bool auto_global_recovery = false;
  std::string reason;
  {
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    if (localization_state_machine_->state() !=
          LocalizationState::LOCALIZING ||
        !localization_convergence_timer_.isCurrent(timer_snapshot))
    {
      return false;
    }

    const LocalizationAttemptKind attempt_kind =
      localization_attempt_kind_.load();
    const bool local_recovery = usesRecoveryTimeout(attempt_kind);
    const double timeout_sec = local_recovery ?
      params_->local_recovery_timeout_sec_ :
      params_->localization_timeout_sec_;
    if (!LocalizationConvergenceTimer::expired(
          timer_snapshot, now_ns, timeout_sec))
    {
      return false;
    }

    reason = local_recovery ?
      "local recovery convergence timed out" :
      "localization convergence timed out";
    markLocalizationLostLocked(reason);

    auto_global_recovery =
      local_recovery &&
      params_->auto_global_localization_ &&
      has_last_trusted_pose_.load();
    if (auto_global_recovery)
    {
      local_recovery_pending_.store(false);
      global_localization_requested_.store(true);
      last_global_attempt_ns_.store(0);
    }
    resetGlobalConfirmation();
  }

  RCLCPP_ERROR(
    this->get_logger(), "Localization state changed to LOST: %s",
    reason.c_str());
  if (auto_global_recovery)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "Local recovery timed out; automatic global recovery is now "
      "permitted");
  }
  publishLocalizationStatus();
  return true;
}

void MCL3dlNode::requestGlobalLocalization(const std::string& reason)
{
  bool changed = false;
  {
    // Treat the operator token, global request, and fail-closed LOST
    // transition as one transaction. Otherwise the status timer can clear a
    // request between the individual atomic stores.
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    operator_initialization_confirmed_.store(true);
    operator_global_confirmed_.store(true);
    global_localization_requested_.store(true);
    local_recovery_pending_.store(false);
    last_global_attempt_ns_.store(0);
    changed = markLocalizationLostLocked(reason);
    resetGlobalConfirmation();
  }
  if (changed)
  {
    RCLCPP_ERROR(
      this->get_logger(), "Localization state changed to LOST: %s",
      reason.c_str());
  }
  publishLocalizationStatus();
}

void MCL3dlNode::recordFeatureMetrics(
    const std::size_t topic_index,
    const sensor_msgs::msg::PointCloud2::SharedPtr& msg)
{
  (void)msg;
  if (topic_index >= feature_received_counts_.size())
  {
    return;
  }
  const int64_t now_ns = clock_->now().nanoseconds();
  feature_received_counts_[topic_index].fetch_add(1);
  feature_last_received_ns_[topic_index].store(now_ns);
  if (topic_index == 1)
  {
    last_feature_received_ns_.store(now_ns);
  }
}

void MCL3dlNode::publishFeatureMetrics(const int64_t now_ns)
{
  constexpr int64_t kWindowNs = 5'000'000'000LL;
  if (feature_metric_window_started_ns_ == 0)
  {
    feature_metric_window_started_ns_ = now_ns;
    for (std::size_t index = 0;
         index < feature_received_counts_.size(); ++index)
    {
      feature_metric_previous_received_[index] =
          feature_received_counts_[index].load();
    }
    feature_metric_previous_sync_ = feature_sync_count_.load();
    feature_metric_previous_processing_ = feature_processing_count_.load();
    feature_metric_previous_processing_ns_ =
        feature_processing_total_ns_.load();
    return;
  }
  const int64_t elapsed_ns = now_ns - feature_metric_window_started_ns_;
  if (elapsed_ns < kWindowNs)
  {
    return;
  }
  const double elapsed_sec = static_cast<double>(elapsed_ns) / 1e9;
  std::array<double, 4> rates{};
  uint64_t minimum_received_delta = std::numeric_limits<uint64_t>::max();
  for (std::size_t index = 0;
       index < feature_received_counts_.size(); ++index)
  {
    const uint64_t current = feature_received_counts_[index].load();
    const uint64_t delta =
        current - feature_metric_previous_received_[index];
    rates[index] = static_cast<double>(delta) / elapsed_sec;
    minimum_received_delta = std::min(minimum_received_delta, delta);
    feature_metric_previous_received_[index] = current;
  }
  const uint64_t sync_current = feature_sync_count_.load();
  const uint64_t sync_delta =
      sync_current - feature_metric_previous_sync_;
  const double sync_rate = static_cast<double>(sync_delta) / elapsed_sec;
  const double sync_ratio = minimum_received_delta == 0 ?
      0.0 :
      static_cast<double>(sync_delta) /
      static_cast<double>(minimum_received_delta);

  const uint64_t processing_current = feature_processing_count_.load();
  const uint64_t processing_delta =
      processing_current - feature_metric_previous_processing_;
  const int64_t processing_ns_current =
      feature_processing_total_ns_.load();
  const int64_t processing_ns_delta =
      processing_ns_current - feature_metric_previous_processing_ns_;
  const double mean_processing_ms = processing_delta == 0 ?
      0.0 :
      static_cast<double>(processing_ns_delta) /
      static_cast<double>(processing_delta) / 1e6;
  const double header_age_ms =
      static_cast<double>(feature_last_header_age_ns_.load()) / 1e6;

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "{\"receive_hz\":{"
         << "\"sharp\":" << rates[0] << ","
         << "\"less_sharp\":" << rates[1] << ","
         << "\"flat\":" << rates[2] << ","
         << "\"less_flat\":" << rates[3] << "},"
         << "\"sync_hz\":" << sync_rate << ","
         << "\"sync_ratio\":" << sync_ratio << ","
         << "\"processing_ms\":" << mean_processing_ms << ","
         << "\"header_age_ms\":" << header_age_ms << "}";
  std_msgs::msg::String message;
  message.data = stream.str();
  if (pub_feature_stream_metrics_)
  {
    pub_feature_stream_metrics_->publish(message);
  }
  RCLCPP_INFO(
      this->get_logger(), "Feature stream metrics: %s",
      message.data.c_str());

  feature_metric_previous_sync_ = sync_current;
  feature_metric_previous_processing_ = processing_current;
  feature_metric_previous_processing_ns_ = processing_ns_current;
  feature_metric_window_started_ns_ = now_ns;
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
  if (!pub_localization_status_ || !pub_localization_health_)
  {
    return;
  }
  std_msgs::msg::String status;
  std_msgs::msg::String health;
  {
    std::lock_guard<std::mutex> lock(localization_state_mutex_);
    status.data = localization_state_machine_->stateName();
    health.data = localization_health_;
  }
  pub_localization_status_->publish(status);
  pub_localization_health_->publish(health);
  if (pub_localization_residual_)
  {
    std_msgs::msg::Float32 residual;
    residual.data = latest_residual_.load();
    pub_localization_residual_->publish(residual);
  }
}

void MCL3dlNode::publishLocalizationStatusThread()
{
  const int64_t now_ns = clock_->now().nanoseconds();
  publishFeatureMetrics(now_ns);
  const LocalizationStateSnapshot state_snapshot =
      localizationStateSnapshot();
  const LocalizationState state = state_snapshot.state;
  if (state == LocalizationState::TRACKING || state == LocalizationState::LOCALIZING)
  {
    const int64_t odom_ns = last_odom_received_ns_.load();
    bool feature_stale = false;
    for (const auto& received_ns : feature_last_received_ns_)
    {
      const int64_t feature_ns = received_ns.load();
      const bool topic_stale = state == LocalizationState::TRACKING ?
          (feature_ns == 0 ||
           static_cast<double>(now_ns - feature_ns) / 1e9 >
             params_->localization_sensor_timeout_sec_) :
          (feature_ns != 0 &&
           static_cast<double>(now_ns - feature_ns) / 1e9 >
             params_->localization_sensor_timeout_sec_);
      feature_stale = feature_stale || topic_stale;
    }
    const bool odom_stale = state == LocalizationState::TRACKING ?
        (odom_ns == 0 || static_cast<double>(now_ns - odom_ns) / 1e9 >
          params_->localization_sensor_timeout_sec_) :
        (odom_ns != 0 && static_cast<double>(now_ns - odom_ns) / 1e9 >
          params_->localization_sensor_timeout_sec_);
    if (feature_stale || odom_stale)
    {
      if (sensorTimeoutCausesLost(state))
      {
        if (markLocalizationLostIfCurrent(
              feature_stale ?
                "one or more lidar feature topics timed out" :
                "odometry stream timed out",
              state_snapshot))
        {
          return;
        }
      }
      else
      {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *clock_, 3000,
          "Localization input heartbeat is stale while LOCALIZING; motion "
          "remains blocked while waiting for input recovery or convergence "
          "timeout");
      }
    }

    if (state == LocalizationState::LOCALIZING)
    {
      const auto timer_snapshot =
        localization_convergence_timer_.snapshot();
      if (markLocalizationLostIfConvergenceExpired(
            timer_snapshot, now_ns))
      {
        return;
      }
    }
  }
  publishLocalizationStatus();
}

void MCL3dlNode::publishTFThread()
{
  if (tf_ready_.load() && isTracking() && params_->publish_tf_){
    geometry_msgs::msg::TransformStamped transform;
    {
      std::lock_guard<std::mutex> lock(tf_pub_mutex_);
      transform = map2odom_trans_;
    }

    // map->odom is a slowly corrected localization transform while odom->base
    // carries the high-rate motion. Publish the latest correction at wall/ROS
    // time plus the configured tolerance, as AMCL-style localizers do. Using
    // the measurement cloud stamp here made the latest common TF time lag by
    // multiple seconds during an expensive MCL update, even though odometry
    // and LiDAR were both current.
    const double tolerance_sec =
        std::max(0.0, tf2::durationToSec(params_->tf_tolerance_));
    transform.header.stamp =
        clock_->now() + rclcpp::Duration::from_seconds(tolerance_sec);
    tfb_->sendTransform(transform);
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
  pcl::PointCloud<pcl::Normal>* ground_normals = nullptr;
  if (sub_maps_->isGlobalReady())
  {
    ground_tree = &sub_maps_->kdtree_ground_global_;
    ground_normals = &sub_maps_->normals_ground_global_;
  }
  else if (sub_maps_->isCurrentReady())
  {
    ground_tree = &sub_maps_->kdtree_ground_current_;
    ground_normals = &sub_maps_->normals_ground_current_;
  }

  State6DOF mean(
      Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
      Quat(pose.pose.orientation.x, pose.pose.orientation.y,
           pose.pose.orientation.z, pose.pose.orientation.w));
  if (params_->flat_ground_enabled_)
  {
    if (ground_tree == nullptr || ground_normals == nullptr)
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Discarded initial pose: 2.5D mode requires a ready map-ground tree");
      return;
    }
    if (!constrainState2p5D(mean, *ground_tree, *ground_normals))
    {
      RCLCPP_ERROR(
          this->get_logger(),
          "Discarded initial pose: no trusted local map ground for 2.5D base height");
      return;
    }
    pose.pose.position.z = mean.pos_.z_;
    pose.pose.orientation.x = mean.rot_.x_;
    pose.pose.orientation.y = mean.rot_.y_;
    pose.pose.orientation.z = mean.rot_.z_;
    pose.pose.orientation.w = mean.rot_.w_;
    RCLCPP_INFO(
        this->get_logger(), "2.5D base_link z set to ground + %.3f m",
        params_->flat_ground_base_link_height_);
  }
  else if (ground_tree != nullptr)
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

  mean = State6DOF(
      Vec3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
      Quat(pose.pose.orientation.x, pose.pose.orientation.y,
           pose.pose.orientation.z, pose.pose.orientation.w));

  geometry_msgs::msg::PoseWithCovarianceStamped adjusted_pose = *msg;
  adjusted_pose.pose.pose = pose.pose;
  {
    std::unique_lock<mcl_3dl::SubMaps::sub_maps_mutex_t> lock(*(sub_maps_->getMutex()));
    sub_maps_->setInitialPose(adjusted_pose);
  }
  
  RCLCPP_INFO(this->get_logger(), "Set initial pose at: %.2f, %.2f, %.2f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
  const MultivariateNoiseGenerator<float> noise_gen(mean, msg->pose.covariance);
  if (static_cast<int>(pf_->getParticleSize()) != params_->num_particles_)
  {
    pf_->resizeParticle(params_->num_particles_);
  }
  pf_->initUsingNoiseGenerator(noise_gen);
  if (ground_tree != nullptr && ground_normals != nullptr)
  {
    constrainParticles2p5D(*ground_tree, *ground_normals);
  }

  auto integ_reset_func = [](State6DOF& s)
  {
    s.odom_err_integ_lin_ = Vec3();
    s.odom_err_integ_ang_ = Vec3();
  };
  pf_->predict(integ_reset_func);

  state_prev_ = mean;
  last_measure_ns_.store(0);
  operator_initialization_confirmed_.store(true);
  startLocalizing(
    "fixed/manual initial pose received; global search blocked",
    LocalizationAttemptKind::FIXED_POSE);
  publishParticles();
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
