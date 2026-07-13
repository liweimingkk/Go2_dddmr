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
#include <p2p_move_base/p2p_move_base.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{

const char * stairStateName(p2p_move_base::StairTraversalState state)
{
  using p2p_move_base::StairTraversalState;
  switch (state) {
    case StairTraversalState::NORMAL: return "NORMAL";
    case StairTraversalState::PRECHECK: return "STAIR_PRECHECK";
    case StairTraversalState::APPROACH: return "STAIR_APPROACH";
    case StairTraversalState::ALIGN: return "STAIR_ALIGN";
    case StairTraversalState::COMMITTED: return "STAIR_COMMITTED";
    case StairTraversalState::LANDING_VERIFY: return "STAIR_LANDING_VERIFY";
    case StairTraversalState::FAULT_LATCH: return "STAIR_FAULT_LATCH";
  }
  return "UNKNOWN";
}

}  // namespace

namespace p2p_move_base
{

P2PMoveBase::P2PMoveBase(std::string name): Node(name)
{
  name_ = name;
  clock_ = this->get_clock();
}

rclcpp_action::GoalResponse P2PMoveBase::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse P2PMoveBase::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void P2PMoveBase::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{

  if (is_active(current_handle_)){
    RCLCPP_INFO(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
    current_handle_->abort(result);
    return;
  }
  else{
    current_handle_ = goal_handle;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&P2PMoveBase::executeCb, this, std::placeholders::_1), goal_handle}.detach();
}

void P2PMoveBase::initial(const std::shared_ptr<local_planner::Local_Planner>& lp
                    ,const std::shared_ptr<p2p_move_base::P2PGlobalPlanManager>& gpm){
  
  LP_ = lp;
  GPM_ = gpm;

  STATE_ = std::make_shared<p2p_move_base::State>(this->get_node_logging_interface(), this->get_node_parameters_interface());

  // Preserve the legacy one-cycle transition unless a robot profile opts into
  // filtering at a rate matched to its perception stream.
  this->declare_parameter("local_failure_confirmation_cycles", rclcpp::ParameterValue(1));
  const auto local_failure_confirmation_cycles =
    this->get_parameter("local_failure_confirmation_cycles").as_int();
  if(local_failure_confirmation_cycles <= 0){
    throw std::invalid_argument(
      "local_failure_confirmation_cycles must be a positive integer");
  }
  local_failure_debounce_.configure(
    static_cast<std::size_t>(local_failure_confirmation_cycles));
  RCLCPP_INFO(
    this->get_logger(), "local_failure_confirmation_cycles: %ld",
    static_cast<long>(local_failure_confirmation_cycles));

  terrain_supervisor_enabled_ = this->declare_parameter<bool>(
    "terrain_supervisor_enabled", false);
  stair_align_trajectory_generator_ = this->declare_parameter<std::string>(
    "stair_align_trajectory_generator", "differential_drive_stair_align");
  if (stair_align_trajectory_generator_.empty()) {
    throw std::invalid_argument("stair_align_trajectory_generator cannot be empty");
  }
  const auto terrain_status_topic = this->declare_parameter<std::string>(
    "terrain_status_topic", "/dddmr_terrain/status");
  const auto terrain_supervised_status_topic = this->declare_parameter<std::string>(
    "terrain_supervised_status_topic", "/dddmr_terrain/supervised_status");
  const auto gait_unchanged_topic = this->declare_parameter<std::string>(
    "gait_unchanged_topic", "/dddmr_go2/gait_unchanged");
  require_gait_monitor_ = this->declare_parameter<bool>(
    "require_gait_monitor", true);
  gait_monitor_timeout_sec_ = this->declare_parameter<double>(
    "gait_monitor_timeout_sec", 0.30);
  terrain_status_timeout_sec_ = this->declare_parameter<double>(
    "terrain_status_timeout_sec", 0.30);
  if (!std::isfinite(terrain_status_timeout_sec_) || terrain_status_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("terrain_status_timeout_sec must be finite and positive");
  }
  if (!std::isfinite(gait_monitor_timeout_sec_) || gait_monitor_timeout_sec_ <= 0.0) {
    throw std::invalid_argument("gait_monitor_timeout_sec must be finite and positive");
  }

  StairSupervisorConfig stair_config;
  stair_config.enabled = terrain_supervisor_enabled_;
  const auto confirmation_cycles = this->declare_parameter<int64_t>(
    "stair_confirmation_cycles", 5);
  if (confirmation_cycles <= 0) {
    throw std::invalid_argument("stair_confirmation_cycles must be positive");
  }
  stair_config.confirmation_cycles = static_cast<std::size_t>(confirmation_cycles);
  stair_config.min_confidence = this->declare_parameter<double>(
    "stair_min_confidence", 0.90);
  stair_config.min_support_ratio = this->declare_parameter<double>(
    "stair_min_support_ratio", 0.80);
  stair_config.max_heading_error_rad = this->declare_parameter<double>(
    "stair_max_heading_error_rad", 0.13962634015954636);
  stair_config.max_lateral_error_m = this->declare_parameter<double>(
    "stair_max_lateral_error_m", 0.10);
  stair_config.align_max_forward_mps = this->declare_parameter<double>(
    "stair_align_max_forward_mps", 0.0);
  stair_config.align_max_yaw_rps = this->declare_parameter<double>(
    "stair_align_max_yaw_rps", 0.25);
  stair_config.committed_max_yaw_rps = this->declare_parameter<double>(
    "stair_committed_max_yaw_rps", 0.0);
  stair_supervisor_.configure(stair_config);
  
  if(STATE_->use_twist_stamped_){
    stamped_cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped", 1);
  }
  else{
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  }
  decision_pub_ = this->create_publisher<std_msgs::msg::String>("/dddmr_go2/p2p_decision", 1);
  terrain_traversal_state_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/dddmr_terrain/traversal_state", 1);
  terrain_supervised_status_pub_ =
    this->create_publisher<dddmr_sys_core::msg::TerrainStatus>(
      terrain_supervised_status_topic, 10);
  terrain_status_sub_ = this->create_subscription<dddmr_sys_core::msg::TerrainStatus>(
    terrain_status_topic, 10,
    std::bind(&P2PMoveBase::terrainStatusCb, this, std::placeholders::_1));
  gait_unchanged_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    gait_unchanged_topic, 10,
    std::bind(&P2PMoveBase::gaitUnchangedCb, this, std::placeholders::_1));
  reset_stair_fault_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/dddmr_terrain/reset_stair_fault",
    std::bind(
      &P2PMoveBase::resetStairFaultCb, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_WARN(
    this->get_logger(),
    "Terrain stair supervisor enabled=%d topic=%s timeout=%.3f. "
    "This supervisor constrains Move/Stop commands and never changes gait.",
    terrain_supervisor_enabled_, terrain_status_topic.c_str(),
    terrain_status_timeout_sec_);
  RCLCPP_WARN(
    this->get_logger(), "Gait monitor required=%d topic=%s timeout=%.3f",
    require_gait_monitor_, gait_unchanged_topic.c_str(), gait_monitor_timeout_sec_);
  

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
  
  recovery_behaviors_client_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  recovery_behaviors_client_ptr_ = rclcpp_action::create_client<dddmr_sys_core::action::RecoveryBehaviors>(
      this,
      "recovery_behaviors", recovery_behaviors_client_group_);

  //@Create action server
  action_server_p2p_move_base_ = rclcpp_action::create_server<dddmr_sys_core::action::PToPMoveBase>(
    this,
    "/p2p_move_base",
    std::bind(&P2PMoveBase::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&P2PMoveBase::handle_cancel, this, std::placeholders::_1),
    std::bind(&P2PMoveBase::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);

  RCLCPP_INFO(this->get_logger(), "\033[1;32m---->\033[0m P2P move base launched.");

}

P2PMoveBase::~P2PMoveBase(){
  STATE_.reset();
  tf2Buffer_.reset();
  tfl_.reset();
  LP_.reset();
  GPM_.reset();
}

std::string P2PMoveBase::selectControllingTrajectoryGenerator() const
{
  std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
  if (terrain_supervisor_enabled_ &&
    stair_supervisor_.state() == StairTraversalState::ALIGN)
  {
    // This generator is scored as pure yaw before the supervisor sees it; no
    // post-score velocity reshaping is permitted.
    return stair_align_trajectory_generator_;
  }
  return STATE_->main_trajectory_generator_;
}

bool P2PMoveBase::isQuaternionValid(const geometry_msgs::msg::Quaternion& q){
  //first we need to check if the quaternion has nan's or infs
  if(!std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has nans or infs... discarding as a navigation goal");
    return false;
  }

  tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);

  //next, we need to check if the length of the quaternion is close to zero
  if(tf_q.length2() < 1e-6){
    RCLCPP_ERROR(this->get_logger(), "Quaternion has length close to zero... discarding as navigation goal");
    return false;
  }

  //next, we'll normalize the quaternion and check that it transforms the vertical vector correctly
  tf_q.normalize();

  tf2::Vector3 up(0, 0, 1);

  double dot = up.dot(up.rotate(tf_q.getAxis(), tf_q.getAngle()));

  if(fabs(dot - 1) > 1e-3){
    RCLCPP_ERROR(this->get_logger(), "Quaternion is invalid... for navigation the z-axis of the quaternion must be close to vertical.");
    return false;
  }

  return true;
}

void P2PMoveBase::publishZeroVelocity(){
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.0;
  cmd_vel.linear.y = 0.0;
  cmd_vel.angular.z = 0.0;
  if(STATE_->use_twist_stamped_){
    geometry_msgs::msg::TwistStamped stamped_cmd_vel;
    stamped_cmd_vel.header.frame_id = LP_->getControlFrame();
    stamped_cmd_vel.header.stamp = clock_->now();
    stamped_cmd_vel.twist = cmd_vel;
    stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
  }
  else{
    cmd_vel_pub_->publish(cmd_vel);
  }
  last_command_stopped_.store(true, std::memory_order_release);
}

void P2PMoveBase::publishVelocity(double vx, double vy, double angular_z){
  refreshTerrainSupervisor();
  StairCommandDecision stair_decision;
  bool command_fault_latched = false;
  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    stair_decision = stair_supervisor_.filterCommand(vx, vy, angular_z);
    if (!stair_decision.allowed && stair_decision.latch_fault) {
      stair_supervisor_.latchExternalFault(stair_decision.reason);
      command_fault_latched = true;
    }
  }
  if (!stair_decision.allowed) {
    RCLCPP_ERROR_THROTTLE(
      this->get_logger(), *clock_, 1000,
      "Terrain supervisor blocked velocity: %s", stair_decision.reason.c_str());
    publishZeroVelocity();
    if (command_fault_latched) {
      publishTerrainTraversalState();
      publishSupervisedTerrainStatus();
    }
    return;
  }

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = stair_decision.x;
  cmd_vel.linear.y = stair_decision.y;
  cmd_vel.angular.z = stair_decision.yaw;
  if(STATE_->use_twist_stamped_){
    geometry_msgs::msg::TwistStamped stamped_cmd_vel;
    stamped_cmd_vel.header.frame_id = LP_->getControlFrame();
    stamped_cmd_vel.header.stamp = clock_->now();
    stamped_cmd_vel.twist = cmd_vel;
    stamped_cmd_vel_pub_->publish(stamped_cmd_vel);
  }
  else{
    cmd_vel_pub_->publish(cmd_vel);
  }
  last_command_stopped_.store(
    stair_decision.x == 0.0 && stair_decision.y == 0.0 && stair_decision.yaw == 0.0,
    std::memory_order_release);
}

void P2PMoveBase::publishDecisionState(){
  if(!decision_pub_ || !STATE_){
    return;
  }
  std_msgs::msg::String msg;
  msg.data = STATE_->getCurrentDecision();
  decision_pub_->publish(msg);
}

void P2PMoveBase::publishTerrainTraversalState()
{
  if (!terrain_traversal_state_pub_) {
    return;
  }
  std_msgs::msg::String msg;
  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    msg.data = stairStateName(stair_supervisor_.state());
    if (stair_supervisor_.faultLatched()) {
      msg.data += ":" + stair_supervisor_.faultReason();
    } else if (terrain_supervisor_enabled_ && !stair_supervisor_.inputReady()) {
      msg.data += ":HOLD:" + stair_supervisor_.inputHoldReason();
    }
  }
  terrain_traversal_state_pub_->publish(msg);
}

void P2PMoveBase::publishSupervisedTerrainStatus()
{
  if (!terrain_supervised_status_pub_) {
    return;
  }
  dddmr_sys_core::msg::TerrainStatus status;
  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    if (!has_terrain_status_) {
      return;
    }
    status = latest_terrain_status_;
    const auto now = clock_->now();
    status.header.stamp = now;
    status.traversal_state = static_cast<std::uint8_t>(stair_supervisor_.state());
    const double receive_age = (now - last_terrain_status_time_).seconds();
    const double reported_age = static_cast<double>(latest_terrain_status_.data_age_sec);
    const bool terrain_fresh = std::isfinite(receive_age) && receive_age >= 0.0 &&
      receive_age <= terrain_status_timeout_sec_ && std::isfinite(reported_age) &&
      reported_age >= 0.0 && reported_age <= terrain_status_timeout_sec_;
    const auto observation = toStairObservation(latest_terrain_status_, terrain_fresh);
    status.gait_unchanged = observation.gait_fresh && observation.gait_unchanged;
    status.data_age_sec =
      std::isfinite(receive_age) && receive_age >= 0.0 &&
      std::isfinite(reported_age) && reported_age >= 0.0 ?
      static_cast<float>(std::max(receive_age, reported_age)) :
      std::numeric_limits<float>::infinity();
    if (terrain_supervisor_enabled_ &&
      (!stair_supervisor_.inputReady() || stair_supervisor_.faultLatched()))
    {
      status.allow_forward = false;
      status.allow_reverse = false;
      status.rejection_reason = stair_supervisor_.faultLatched() ?
        stair_supervisor_.faultReason() : stair_supervisor_.inputHoldReason();
    }
  }
  terrain_supervised_status_pub_->publish(status);
}

StairObservation P2PMoveBase::toStairObservation(
  const dddmr_sys_core::msg::TerrainStatus & msg, bool fresh) const
{
  StairObservation observation;
  observation.fresh = fresh;
  observation.stair_candidate =
    msg.staircase_id >= 0 ||
    msg.terrain_class == dddmr_sys_core::msg::TerrainStatus::TERRAIN_STAIR_TREAD ||
    msg.terrain_class == dddmr_sys_core::msg::TerrainStatus::TERRAIN_STAIR_RISER;
  observation.entry_valid = msg.entry_valid;
  observation.at_entry = msg.at_entry;
  observation.on_stair = msg.on_stair;
  observation.landing_valid = msg.landing_valid;
  observation.full_body_on_landing = msg.full_body_on_landing;
  observation.terrain_accepted = msg.rejection_code == 0U;
  observation.allow_forward = msg.allow_forward;
  observation.drop_detected = msg.drop_detected;
  observation.dynamic_obstacle = msg.dynamic_obstacle;
  bool monitored_gait_fresh = true;
  bool monitored_gait_unchanged = true;
  if (require_gait_monitor_) {
    const double gait_age = has_gait_status_ ?
      (clock_->now() - last_gait_status_time_).seconds() :
      std::numeric_limits<double>::infinity();
    monitored_gait_fresh = has_gait_status_ &&
      std::isfinite(gait_age) && gait_age >= 0.0 &&
      gait_age <= gait_monitor_timeout_sec_;
    monitored_gait_unchanged = monitored_gait_fresh && latest_gait_unchanged_;
  }
  observation.gait_fresh = monitored_gait_fresh;
  observation.gait_unchanged = msg.gait_unchanged && monitored_gait_unchanged;
  observation.staircase_id = msg.staircase_id;
  observation.step_index = msg.step_index;
  observation.step_count = msg.step_count;
  observation.snapshot_version = msg.snapshot_version;
  observation.static_ground_generation = msg.static_ground_generation;
  observation.confidence = msg.confidence;
  observation.support_ratio = msg.support_ratio;
  observation.heading_error_rad = msg.heading_error_rad;
  observation.lateral_error_m = msg.lateral_error_m;
  return observation;
}

void P2PMoveBase::terrainStatusCb(
  const dddmr_sys_core::msg::TerrainStatus::SharedPtr msg)
{
  const double reported_age = static_cast<double>(msg->data_age_sec);
  const bool fresh = std::isfinite(reported_age) && reported_age >= 0.0 &&
    reported_age <= terrain_status_timeout_sec_;
  StairTraversalState old_state;
  StairTraversalState new_state;
  bool new_ready;
  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    latest_terrain_status_ = *msg;
    last_terrain_status_time_ = clock_->now();
    has_terrain_status_ = true;
    old_state = stair_supervisor_.state();
    new_state = stair_supervisor_.update(toStairObservation(*msg, fresh));
    new_ready = stair_supervisor_.inputReady();
  }
  if (old_state != new_state) {
    RCLCPP_WARN(
      this->get_logger(), "Stair traversal state: %s -> %s",
      stairStateName(old_state), stairStateName(new_state));
  }
  if (terrain_supervisor_enabled_ &&
    (!new_ready || new_state == StairTraversalState::FAULT_LATCH))
  {
    publishZeroVelocity();
  }
  publishTerrainTraversalState();
  publishSupervisedTerrainStatus();
}

void P2PMoveBase::gaitUnchangedCb(const std_msgs::msg::Bool::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    latest_gait_unchanged_ = msg->data;
    has_gait_status_ = true;
    last_gait_status_time_ = clock_->now();
  }
  refreshTerrainSupervisor();
  publishSupervisedTerrainStatus();
}

void P2PMoveBase::refreshTerrainSupervisor()
{
  if (!terrain_supervisor_enabled_) {
    return;
  }
  StairTraversalState old_state;
  StairTraversalState new_state;
  bool old_ready;
  bool new_ready;
  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    old_state = stair_supervisor_.state();
    old_ready = stair_supervisor_.inputReady();
    if (!has_terrain_status_) {
      new_state = stair_supervisor_.refreshInputHealth(false, false, false);
    } else {
      const double receive_age = (clock_->now() - last_terrain_status_time_).seconds();
      const double reported_age = static_cast<double>(latest_terrain_status_.data_age_sec);
      const bool terrain_fresh = std::isfinite(receive_age) && receive_age >= 0.0 &&
        receive_age <= terrain_status_timeout_sec_ && std::isfinite(reported_age) &&
        reported_age >= 0.0 && reported_age <= terrain_status_timeout_sec_;
      const auto observation = toStairObservation(latest_terrain_status_, terrain_fresh);
      new_state = stair_supervisor_.refreshInputHealth(
        observation.fresh, observation.gait_fresh, observation.gait_unchanged);
    }
    new_ready = stair_supervisor_.inputReady();
  }
  if (old_state != new_state || old_ready != new_ready) {
    RCLCPP_ERROR(
      this->get_logger(), "Stair traversal state/readiness: %s/%d -> %s/%d",
      stairStateName(old_state), old_ready, stairStateName(new_state), new_ready);
    publishTerrainTraversalState();
    publishSupervisedTerrainStatus();
  }
  if (!new_ready || new_state == StairTraversalState::FAULT_LATCH) {
    publishZeroVelocity();
  }
}

void P2PMoveBase::resetStairFaultCb(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;
  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    const auto now = clock_->now();
    const double receive_age = has_terrain_status_ ?
      (now - last_terrain_status_time_).seconds() :
      std::numeric_limits<double>::infinity();
    const double reported_age = has_terrain_status_ ?
      static_cast<double>(latest_terrain_status_.data_age_sec) :
      std::numeric_limits<double>::infinity();
    const bool terrain_fresh = has_terrain_status_ && std::isfinite(receive_age) &&
      receive_age >= 0.0 && receive_age <= terrain_status_timeout_sec_ &&
      std::isfinite(reported_age) && reported_age >= 0.0 &&
      reported_age <= terrain_status_timeout_sec_;
    const double gait_age = has_gait_status_ ?
      (now - last_gait_status_time_).seconds() :
      std::numeric_limits<double>::infinity();
    const bool gait_fresh = !require_gait_monitor_ ||
      (has_gait_status_ && std::isfinite(gait_age) && gait_age >= 0.0 &&
      gait_age <= gait_monitor_timeout_sec_);

    auto observation = toStairObservation(latest_terrain_status_, terrain_fresh);
    observation.gait_fresh = gait_fresh;
    observation.gait_unchanged = has_terrain_status_ &&
      latest_terrain_status_.gait_unchanged &&
      (!require_gait_monitor_ || (gait_fresh && latest_gait_unchanged_));
    response->success = stair_supervisor_.resetFault(
      last_command_stopped_.load(std::memory_order_acquire), observation);
  }
  response->message = response->success ?
    "stair fault reset with stopped command and fresh healthy terrain/gait" :
    "reset refused: require a latched fault, zero command, fresh healthy terrain/gait, "
    "and full-body verified landing";
  publishTerrainTraversalState();
  publishSupervisedTerrainStatus();
}

void P2PMoveBase::executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle)
{
  auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
  auto move_base_goal = goal_handle->get_goal();

  if(!isQuaternionValid(move_base_goal->target_pose.pose.orientation)){
    RCLCPP_WARN(this->get_logger(),"Aborting on goal because it was sent with an invalid quaternion");
    goal_handle->abort(result);
    publishZeroVelocity();
    return;
  }

  rclcpp::Rate r(STATE_->controller_frequency_);

  //@ if we dont initialize oscillation pose here, the first controlling entry will cause recovery behavior.
  //@ the rclcpp::Time initial are all done in FSM class
  STATE_->initialParams(LP_->getGlobalPose(), clock_->now());
  LP_->resetInPlaceRotationHysteresis();
  local_failure_debounce_.reset();
  STATE_->current_goal_ = move_base_goal->target_pose;
  GPM_->setGoal(STATE_->current_goal_);
  GPM_->resume();
  publishDecisionState();

  while(rclcpp::ok()){

    if(!goal_handle->is_active()){
      
      if(STATE_->isCurrentDecision("d_recovery_waitdone")){
        RCLCPP_INFO(this->get_logger(), "P2P is in recovery state, cancel recovery behaviors.");
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
      }

      RCLCPP_INFO(this->get_logger(), "P2P move base preempted.");
      publishZeroVelocity();
      GPM_->stop();
      return;
    }

    if(goal_handle->is_canceling()){

      if(STATE_->isCurrentDecision("d_recovery_waitdone")){
        RCLCPP_INFO(this->get_logger(), "P2P is in recovery state, cancel recovery behaviors.");
        recovery_behaviors_client_ptr_->async_cancel_all_goals();
      }

      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "P2P move base cancelled.");
      publishZeroVelocity();
      GPM_->stop();
      return;
    }

    //the real work on pursuing a goal is done here
    bool done = executeCycle(goal_handle);
    publishDecisionState();
    
    auto feedback = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Feedback>();
    feedback->base_position = STATE_->global_pose_;
    feedback->last_decision = STATE_->getLastDecision();
    feedback->current_decision = STATE_->getCurrentDecision();
    goal_handle->publish_feedback(feedback);

    //if we're done, then we'll return from execute
    if(done){
      GPM_->stop();
      return;
    }
    
    r.sleep();

    //if(STATE_->isCurrentDecision("d_controlling") && r.cycleTime() > ros::Duration(1 / STATE_->controller_frequency_))
    //  ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", STATE_->controller_frequency_, r.cycleTime().toSec());
  }
  GPM_->stop();
}

bool P2PMoveBase::executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle){

    refreshTerrainSupervisor();
    {
      std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
      if(stair_supervisor_.faultLatched()){
        RCLCPP_ERROR(
          this->get_logger(), "Aborting navigation because stair safety fault is latched: %s",
          stair_supervisor_.faultReason().c_str());
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;
      }
      if(stair_supervisor_.state() == StairTraversalState::COMMITTED &&
        !STATE_->isCurrentDecision("d_controlling"))
      {
        stair_supervisor_.latchExternalFault("left_controlling_while_committed");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;
      }
    }

    STATE_->global_pose_ = LP_->getGlobalPose();
    if(STATE_->getDistance(STATE_->global_pose_, STATE_->oscillation_pose_) >= STATE_->oscillation_distance_ ||
          STATE_->getAngle(STATE_->global_pose_, STATE_->oscillation_pose_) >= STATE_->oscillation_angle_)
    {
      STATE_->oscillation_pose_ = STATE_->global_pose_;
      STATE_->last_oscillation_reset_ = clock_->now();
    }


    if(STATE_->isCurrentDecision("d_initial")){
      STATE_->setDecision("d_planning");
    }

    else if(STATE_->isCurrentDecision("d_planning")){
      GPM_->queryThread();
      STATE_->setDecision("d_planning_waitdone");
      return false;
    }

    else if(STATE_->isCurrentDecision("d_planning_waitdone")){
      
      //@If global planner keep return empty plan, we will enter this state for n seconds, then abort
      //@see: decision_planning
      std::vector<geometry_msgs::msg::PoseStamped> plan;
      if(GPM_->hasPlan()){
        GPM_->copyPlan(plan);
        //if the planner fails or returns a zero length plan, planning failed
        if(plan.empty()){
          RCLCPP_DEBUG(this->get_logger(), "Failed to find a plan to point (%.2f, %.2f, %.2f)", 
              STATE_->current_goal_.pose.position.x, STATE_->current_goal_.pose.position.y, STATE_->current_goal_.pose.position.z);
          STATE_->setDecision("d_planning");
        }
        else{
          RCLCPP_DEBUG(this->get_logger(), "Found a plan with its final position: (%.2f, %.2f, %.2f)", 
              plan.back().pose.position.x, plan.back().pose.position.y, plan.back().pose.position.z);
          if (!LP_->setPlan(plan)) {
            RCLCPP_ERROR(
              this->get_logger(),
              "Local planner rejected the initial global plan; stopped and requesting a new plan.");
            publishZeroVelocity();
            STATE_->setDecision("d_planning");
            return false;
          }
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_align_heading");  
        }
      }

      if((clock_->now()-STATE_->last_valid_plan_).seconds()>STATE_->planner_patience_){
        RCLCPP_WARN(this->get_logger(), "Time out to find a plan to point (%.2f, %.2f, %.2f)", 
            STATE_->current_goal_.pose.position.x, STATE_->current_goal_.pose.position.y, STATE_->current_goal_.pose.position.z);
        publishZeroVelocity();
        startRecoveryBehaviors("rotate_inplace");
        STATE_->setDecision("d_recovery_waitdone");
        return false;
      }
      return false;
    }
    
    else if(STATE_->isCurrentDecision("d_align_heading")){

      if(LP_->isInitialHeadingAligned()){
        STATE_->setDecision("d_controlling");  
      }
      else{

        if(STATE_->oscillation_patience_ > 0 && (clock_->now()-STATE_->last_oscillation_reset_).seconds() >= STATE_->oscillation_patience_){
          //@go to recovery
          auto diff = (clock_->now()-STATE_->last_oscillation_reset_).seconds();
          RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, STATE_->getDistance(STATE_->global_pose_, STATE_->oscillation_pose_));
          publishZeroVelocity();
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone");  
          return false;
        }
        
        base_trajectory::Trajectory best_traj;
        dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(STATE_->initial_heading_trajectory_generator_, best_traj);

        if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          STATE_->last_valid_control_ = clock_->now();
          STATE_->setDecision("d_align_heading");  
          publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
          //@ this assignment will allow at least one time planning query
          STATE_->last_valid_plan_ = clock_->now();
          publishZeroVelocity();
          STATE_->setDecision("d_planning");  
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
          //At least implement last_valid_control_ timeout to abort here
          //@ this assignment will allow at least one time planning query
          if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
            RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
            startRecoveryBehaviors("rotate_inplace");
            STATE_->setDecision("d_recovery_waitdone");
          }
          else{
            STATE_->last_valid_plan_ = clock_->now();
            STATE_->setDecision("d_planning");  
          }
          publishZeroVelocity();
          return false;
        }

        else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_planning");
          publishZeroVelocity();
          return false;
        }

        else{
          RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
          publishZeroVelocity();
          return false;
        }
      }

    }

    else if(STATE_->isCurrentDecision("d_align_goal_heading")){
      if(LP_->isGoalHeadingAligned()){
        RCLCPP_INFO(this->get_logger(), "Goal reach.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->succeed(result);
        publishZeroVelocity();
        return true;
      }
      else{

        if(STATE_->oscillation_patience_ >0 && (clock_->now()-STATE_->last_oscillation_reset_).seconds() >= STATE_->oscillation_patience_){
          //@go to recovery
          auto diff = (clock_->now()-STATE_->last_oscillation_reset_).seconds();
          RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, STATE_->getDistance(STATE_->global_pose_, STATE_->oscillation_pose_));
          publishZeroVelocity();
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone"); 
          return false;
        }
        
        base_trajectory::Trajectory best_traj;
        dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(STATE_->goal_heading_trajectory_generator_, best_traj);

        if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
          STATE_->last_valid_control_ = clock_->now();
          STATE_->setDecision("d_align_goal_heading");  
          publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
          RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
          publishZeroVelocity();
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
          //@ this assignment will allow at least one time planning query
          STATE_->last_valid_plan_ = clock_->now();
          publishZeroVelocity();
          STATE_->setDecision("d_planning");  
          return false;
        }
        else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL ||
                PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || 
                PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
          //At least implement last_valid_control_ timeout to abort here
          //@ this assignment will allow at least one time planning query
          if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
            RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
            startRecoveryBehaviors("rotate_inplace");
            STATE_->setDecision("d_recovery_waitdone");
          }
          else{
            STATE_->setDecision("d_align_goal_heading");  
          }
          publishZeroVelocity();
          return false;
        }
        else{
          RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
          publishZeroVelocity();
          return false;
        }
      }
    }

    else if(STATE_->isCurrentDecision("d_controlling")){

      //@Check is goal xy tolerance reach
      if(LP_->isGoalReached()){
        publishZeroVelocity();
        if(STATE_->use_position_control_at_goal_){
          RCLCPP_INFO(this->get_logger(), "Goal xy tolerance reach, align the goal with position control.");
          startRecoveryBehaviors("position_control");
          STATE_->setDecision("d_recovery_position_control_waitdone");  
        }
        else{
          STATE_->setDecision("d_align_goal_heading");  
          RCLCPP_INFO(this->get_logger(), "Goal xy tolerance reach, switch to align goal heading state.");
        }
        return false;
      }
      
      //@ update global plan
      bool terrain_replanning_allowed = true;
      {
        std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
        terrain_replanning_allowed = stair_supervisor_.replanningAllowed();
      }
      if(GPM_->hasPlan() && terrain_replanning_allowed){
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        GPM_->copyPlan(plan);
        if (!LP_->setPlan(plan)) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Local planner rejected a replacement global plan; stopped before trajectory scoring.");
          local_failure_debounce_.reset();
          STATE_->last_valid_plan_ = clock_->now();
          publishZeroVelocity();
          STATE_->setDecision("d_planning");
          return false;
        }
      }
      //@Behavior for oscillation here
      
      if(STATE_->oscillation_patience_ >0 && (clock_->now()-STATE_->last_oscillation_reset_).seconds() >= STATE_->oscillation_patience_){
        //@go to recovery
        auto diff = (clock_->now()-STATE_->last_oscillation_reset_).seconds();
        RCLCPP_WARN(this->get_logger(), "Oscillation time out is detected: %.2f secs for %.2f m.", diff, STATE_->getDistance(STATE_->global_pose_, STATE_->oscillation_pose_));
        publishZeroVelocity();
        startRecoveryBehaviors("rotate_inplace");
        STATE_->setDecision("d_recovery_waitdone");
        return false;
      }

      base_trajectory::Trajectory best_traj;
      const std::string trajectory_generator = selectControllingTrajectoryGenerator();
      dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(
        trajectory_generator, best_traj);
      if(
        PS != dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL &&
        PS != dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT)
      {
        local_failure_debounce_.reset();
      }

      if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
        local_failure_debounce_.recordSafeCycle();
        STATE_->last_valid_control_ = clock_->now();
        STATE_->setDecision("d_controlling");  
        publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
        publishZeroVelocity();
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        local_failure_debounce_.reset();
        //@ this assignment will allow at least one time planning query
        STATE_->last_valid_plan_ = clock_->now();
        publishZeroVelocity();
        STATE_->setDecision("d_planning");  
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        // A failed cycle has no safe trajectory, so stop immediately.  Do not
        // launch a full global replan for a single noisy perception frame.
        publishZeroVelocity();
        if(!local_failure_debounce_.recordFailure()){
          STATE_->setDecision("d_controlling");
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *clock_, 1000,
            "Transient local trajectory failure (%zu cycle); stopped and retrying locally.",
            local_failure_debounce_.failureCycles());
          return false;
        }
        local_failure_debounce_.reset();
        //At least implement last_valid_control_ timeout to abort here
        //@ this assignment will allow at least one time planning query
        if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
          RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone");
        }
        else{
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_planning");  
        }

        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
        local_failure_debounce_.reset();
        STATE_->last_valid_plan_ = clock_->now();
        publishZeroVelocity();
        STATE_->setDecision("d_planning"); 
        RCLCPP_WARN(this->get_logger(), "Path conflits, but no need to wait.");
       	return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT){
        // Keep the mandatory stop, but require the no-safe-trajectory result
        // to persist before changing states.  This filters the one-frame
        // controlling<->waiting chatter seen in the live Go2 trace.
        publishZeroVelocity();
        if(!local_failure_debounce_.recordFailure()){
          STATE_->setDecision("d_controlling");
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *clock_, 1000,
            "Transient path-blocked result (%zu cycle); stopped and retrying locally.",
            local_failure_debounce_.failureCycles());
          return false;
        }
        local_failure_debounce_.reset();
        STATE_->waiting_time_ = clock_->now();
        STATE_->setDecision("d_waiting");
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Path conflits, switch to waiting state.");
       	return false;
      }

      else{
        RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
        publishZeroVelocity();
        return false;
      }

    }

    else if(STATE_->isCurrentDecision("d_recovery_position_control_waitdone")){
      
      if(is_recoverying_){
        return false;
      }
      
      if(is_recoverying_succeed_){
        //we go to planning and we also need to count second recovery then abort
        RCLCPP_INFO(this->get_logger(), "Position control succeed, go to align goal heading state.");
        STATE_->last_valid_plan_ = clock_->now();
        STATE_->setDecision("d_align_goal_heading");  
        return false;  
      }
      else{
        //we may abort or go to another recovery
        RCLCPP_ERROR(this->get_logger(), "The potential collision has been detected when doing recovery - position control.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;  
      }
      
    }

    else if(STATE_->isCurrentDecision("d_recovery_waitdone")){
      
      if(is_recoverying_){
        return false;
      }
        

      if(STATE_->no_plan_recovery_count_>=STATE_->no_plan_retry_num_){
        RCLCPP_ERROR(this->get_logger(), "No global plan has been found even we try recovery %d times", STATE_->no_plan_recovery_count_);
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;        
      }
      
      if(is_recoverying_succeed_){
        //we go to planning and we also need to count second recovery then abort
        RCLCPP_INFO(this->get_logger(), "Recovery succeed, back to planning state.");
        STATE_->no_plan_recovery_count_++;
        STATE_->last_valid_plan_ = clock_->now();
        STATE_->setDecision("d_planning");
        return false;  
      }
      else{
        //we may abort or go to another recovery
        RCLCPP_ERROR(this->get_logger(), "The potential collision has been detected when doing recovery.");
        auto result = std::make_shared<dddmr_sys_core::action::PToPMoveBase::Result>();
        goal_handle->abort(result);
        publishZeroVelocity();
        return true;  
      }
      
    }

    else if(STATE_->isCurrentDecision("d_waiting")){
      
      //if continue conflict over 10s,to recalculate the path
      if((clock_->now()-STATE_->waiting_time_).seconds() >= STATE_->waiting_patience_){ 
        STATE_->last_valid_plan_ = clock_->now();
        publishZeroVelocity();
        STATE_->setDecision("d_planning");
        RCLCPP_WARN(this->get_logger(), "waiting time over %.2f,change to d_planning", STATE_->waiting_patience_);
        return false;
      }

      //@ update global plan
      if(GPM_->hasPlan()){
        std::vector<geometry_msgs::msg::PoseStamped> plan;
        GPM_->copyPlan(plan);
        if (!LP_->setPlan(plan)) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Local planner rejected a waiting-state global plan; stopped and replanning.");
          local_failure_debounce_.reset();
          STATE_->last_valid_plan_ = clock_->now();
          publishZeroVelocity();
          STATE_->setDecision("d_planning");
          return false;
        }
      }
      base_trajectory::Trajectory best_traj;
      const std::string trajectory_generator = selectControllingTrajectoryGenerator();
      dddmr_sys_core::PlannerState PS = LP_->computeVelocityCommand(
        trajectory_generator, best_traj);

      if(PS == dddmr_sys_core::PlannerState::TRAJECTORY_FOUND){
        local_failure_debounce_.recordSafeCycle();
        STATE_->last_valid_control_ = clock_->now();
        STATE_->setDecision("d_controlling");
        publishVelocity(best_traj.xv_, best_traj.yv_, best_traj.thetav_);
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PERCEPTION_MALFUNCTION){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Sensor data is out of date, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::CONFIGURATION_ERROR){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Configuration error, check your yaml and logs.");
        publishZeroVelocity();
        return false;
      }
      else if(PS == dddmr_sys_core::PlannerState::TF_FAIL){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Detect TF fail in local planner, we're not going to allow commanding of the base for safety");
        publishZeroVelocity();
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PRUNE_PLAN_FAIL){
        STATE_->last_valid_plan_ = clock_->now();
        publishZeroVelocity();
        STATE_->setDecision("d_planning");  
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::ALL_TRAJECTORIES_FAIL){
        publishZeroVelocity();
        //At least implement last_valid_control_ timeout to abort here
        //@ this assignment will allow at least one time planning query
        if((clock_->now() - STATE_->last_valid_control_).seconds() > STATE_->controller_patience_){
          RCLCPP_WARN(this->get_logger(), "Controller time out, go to recovery");
          startRecoveryBehaviors("rotate_inplace");
          STATE_->setDecision("d_recovery_waitdone");
        }
        else{
          STATE_->last_valid_plan_ = clock_->now();
          STATE_->setDecision("d_planning");  
        }
        return false;
      }

      else if(PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_WAIT || PS == dddmr_sys_core::PlannerState::PATH_BLOCKED_REPLANNING){
	      STATE_->setDecision("d_waiting");
        publishZeroVelocity();
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 5000, "Path conflits in waiting state, keep waiting.");
	      return false;
      }

      else{
        RCLCPP_FATAL(this->get_logger(), "Should not happen here, we did not catch dddmr_sys_core::PlannerState");
        publishZeroVelocity();
        return false;
      }
    }

  return false;
}

void P2PMoveBase::startRecoveryBehaviors(std::string behavior_name){

  {
    std::lock_guard<std::mutex> lock(terrain_supervisor_mutex_);
    if(!stair_supervisor_.recoveryAllowed()){
      RCLCPP_ERROR(
        this->get_logger(),
        "Recovery '%s' blocked in stair state %s; latching fault instead.",
        behavior_name.c_str(), stairStateName(stair_supervisor_.state()));
      stair_supervisor_.latchExternalFault("recovery_requested_on_stair");
      is_recoverying_ = false;
      is_recoverying_succeed_ = false;
      return;
    }
  }

  auto goal_msg = dddmr_sys_core::action::RecoveryBehaviors::Goal();
  goal_msg.behavior_name = behavior_name;
  goal_msg.target_pose = STATE_->current_goal_;

  auto send_goal_options = rclcpp_action::Client<dddmr_sys_core::action::RecoveryBehaviors>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_goal_response_callback, this, std::placeholders::_1);
  send_goal_options.result_callback =
    std::bind(&P2PMoveBase::recovery_behaviors_client_result_callback, this, std::placeholders::_1);
  
  is_recoverying_ = true;
  recovery_behaviors_client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void P2PMoveBase::recovery_behaviors_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by recovery behaviors server");
    is_recoverying_succeed_ = false;
    is_recoverying_ = false;
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by recovery behaviors server, waiting for result");
  }
}

void P2PMoveBase::recovery_behaviors_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::WrappedResult & result)
{
  is_recoverying_succeed_ = false;
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      is_recoverying_succeed_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Recovery Behaviors: Unknown result code");
      break;
  }
  
  is_recoverying_ = false;
}

}//end of name space
