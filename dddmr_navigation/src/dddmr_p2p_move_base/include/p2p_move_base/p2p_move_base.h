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
/*Debug*/
#include <chrono>
#include <atomic>
#include <mutex>
#include <p2p_move_base/p2p_state.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

//@in enum state, the p_to_p_move_base is included
#include <dddmr_sys_core/dddmr_enum_states.h>

//@local planner
#include <local_planner/local_planner.h>

//@for call global planner action
#include "dddmr_sys_core/action/get_plan.hpp"
#include "dddmr_sys_core/msg/terrain_status.hpp"
#include "p2p_move_base/local_failure_debounce.h"
#include "p2p_move_base/p2p_global_plan_manager.h"
#include "p2p_move_base/stair_traversal_supervisor.h"
//@for call recovery action
#include "dddmr_sys_core/action/recovery_behaviors.hpp"
#include "rclcpp_action/rclcpp_action.hpp"


namespace p2p_move_base
{

class P2PMoveBase : public rclcpp::Node {

  public:

    P2PMoveBase(std::string name);
    ~P2PMoveBase();

    void initial(const std::shared_ptr<local_planner::Local_Planner>& lp, const std::shared_ptr<p2p_move_base::P2PGlobalPlanManager>& gpm);
  private:

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const dddmr_sys_core::action::PToPMoveBase::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);
    
    rclcpp_action::Server<dddmr_sys_core::action::PToPMoveBase>::SharedPtr action_server_p2p_move_base_;

    std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> current_handle_;
    
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    rclcpp::CallbackGroup::SharedPtr action_server_group_;
    rclcpp::CallbackGroup::SharedPtr recovery_behaviors_client_group_;

    rclcpp::Clock::SharedPtr clock_;
    
    std::string name_;
    
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr decision_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr terrain_traversal_state_pub_;
    rclcpp::Publisher<dddmr_sys_core::msg::TerrainStatus>::SharedPtr terrain_supervised_status_pub_;
    rclcpp::Subscription<dddmr_sys_core::msg::TerrainStatus>::SharedPtr terrain_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gait_unchanged_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_stair_fault_service_;

    bool isQuaternionValid(const geometry_msgs::msg::Quaternion& q);

    void publishZeroVelocity();
    void publishVelocity(double vx, double vy, double angular_z);
    void publishDecisionState();
    void publishTerrainTraversalState();
    void publishSupervisedTerrainStatus();
    void terrainStatusCb(const dddmr_sys_core::msg::TerrainStatus::SharedPtr msg);
    void gaitUnchangedCb(const std_msgs::msg::Bool::SharedPtr msg);
    void refreshTerrainSupervisor();
    StairObservation toStairObservation(
      const dddmr_sys_core::msg::TerrainStatus & msg, bool fresh) const;
    void resetStairFaultCb(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    std::string selectControllingTrajectoryGenerator() const;

    std::shared_ptr<p2p_move_base::State> STATE_;
    std::shared_ptr<local_planner::Local_Planner> LP_;
    std::shared_ptr<p2p_move_base::P2PGlobalPlanManager> GPM_;

    void executeCb(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);

    bool executeCycle(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> goal_handle);

    bool is_active(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::PToPMoveBase>> handle) const
    {
      return handle != nullptr && handle->is_active();
    }

    rclcpp_action::Client<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr recovery_behaviors_client_ptr_;
    void recovery_behaviors_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr & goal_handle);
    void recovery_behaviors_client_result_callback(const rclcpp_action::ClientGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>::WrappedResult & result);
    bool is_recoverying_{false};
    bool is_recoverying_succeed_{false};
    void startRecoveryBehaviors(std::string behavior_name);
    LocalFailureDebounce local_failure_debounce_;
    StairTraversalSupervisor stair_supervisor_;
    bool terrain_supervisor_enabled_{false};
    std::string stair_align_trajectory_generator_{
      "differential_drive_stair_align"};
    double terrain_status_timeout_sec_{0.30};
    mutable std::mutex terrain_supervisor_mutex_;
    dddmr_sys_core::msg::TerrainStatus latest_terrain_status_;
    rclcpp::Time last_terrain_status_time_{0, 0, RCL_ROS_TIME};
    bool has_terrain_status_{false};
    bool require_gait_monitor_{true};
    double gait_monitor_timeout_sec_{0.30};
    bool latest_gait_unchanged_{false};
    bool has_gait_status_{false};
    rclcpp::Time last_gait_status_time_{0, 0, RCL_ROS_TIME};
    std::atomic<bool> last_command_stopped_{true};


};



}//end of name space
