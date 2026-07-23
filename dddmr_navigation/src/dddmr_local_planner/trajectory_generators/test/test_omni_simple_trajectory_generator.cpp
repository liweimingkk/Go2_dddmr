#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <trajectory_generators/omni_simple_trajectory_generator_theory.h>
#include <trajectory_generators/trajectory_shared_data.h>

namespace trajectory_generators
{
namespace
{

struct OmniTestConfig
{
  double min_vel_x{0.0};
  double max_vel_x{0.40};
  double min_vel_y{-0.20};
  double max_vel_y{0.20};
  double min_vel_trans{0.20};
  double max_vel_trans{0.45};
  double min_vel_theta{0.40};
  double max_vel_theta{0.50};
  double acc_lim_x{3.0};
  double acc_lim_y{2.0};
  double acc_lim_theta{4.0};
  double deceleration_ratio{2.0};
  double controller_frequency{10.0};
  double sim_time{1.5};
  double linear_x_sample{2.0};
  double linear_y_sample{3.0};
  double angular_z_sample{5.0};
  double sim_granularity{0.05};
  double angular_sim_granularity{0.025};
};

class OmniSimpleTrajectoryGeneratorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if(!rclcpp::ok()){
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }
  }

  static void TearDownTestSuite()
  {
    if(rclcpp::ok()){
      rclcpp::shutdown();
    }
  }

  std::vector<base_trajectory::Trajectory> generate(
    const OmniTestConfig & config = OmniTestConfig{},
    const double current_linear_x = 0.0,
    const double current_linear_y = 0.0,
    const double current_yaw_rate = 0.0)
  {
    static int node_id = 0;
    const std::string prefix = "omni_generator";
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter(prefix + ".min_vel_x", config.min_vel_x),
      rclcpp::Parameter(prefix + ".max_vel_x", config.max_vel_x),
      rclcpp::Parameter(prefix + ".min_vel_y", config.min_vel_y),
      rclcpp::Parameter(prefix + ".max_vel_y", config.max_vel_y),
      rclcpp::Parameter(prefix + ".min_vel_trans", config.min_vel_trans),
      rclcpp::Parameter(prefix + ".max_vel_trans", config.max_vel_trans),
      rclcpp::Parameter(prefix + ".min_vel_theta", config.min_vel_theta),
      rclcpp::Parameter(prefix + ".max_vel_theta", config.max_vel_theta),
      rclcpp::Parameter(prefix + ".acc_lim_x", config.acc_lim_x),
      rclcpp::Parameter(prefix + ".acc_lim_y", config.acc_lim_y),
      rclcpp::Parameter(prefix + ".acc_lim_theta", config.acc_lim_theta),
      rclcpp::Parameter(prefix + ".prune_forward", 3.0),
      rclcpp::Parameter(prefix + ".prune_backward", 1.0),
      rclcpp::Parameter(prefix + ".deceleration_ratio", config.deceleration_ratio),
      rclcpp::Parameter(prefix + ".use_motor_constraint", false),
      rclcpp::Parameter(prefix + ".max_motor_shaft_rpm", 3000.0),
      rclcpp::Parameter(prefix + ".wheel_diameter", 0.16),
      rclcpp::Parameter(prefix + ".gear_ratio", 1.0),
      rclcpp::Parameter(prefix + ".robot_radius", 0.25),
      rclcpp::Parameter(prefix + ".controller_frequency", config.controller_frequency),
      rclcpp::Parameter(prefix + ".sim_time", config.sim_time),
      rclcpp::Parameter(prefix + ".linear_x_sample", config.linear_x_sample),
      rclcpp::Parameter(prefix + ".linear_y_sample", config.linear_y_sample),
      rclcpp::Parameter(prefix + ".angular_z_sample", config.angular_z_sample),
      rclcpp::Parameter(prefix + ".sim_granularity", config.sim_granularity),
      rclcpp::Parameter(
        prefix + ".angular_sim_granularity", config.angular_sim_granularity),
      rclcpp::Parameter(prefix + ".cuboid.flb", std::vector<double>{0.42, 0.21, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.frb", std::vector<double>{0.42, -0.21, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.flt", std::vector<double>{0.42, 0.21, 0.60}),
      rclcpp::Parameter(prefix + ".cuboid.frt", std::vector<double>{0.42, -0.21, 0.60}),
      rclcpp::Parameter(prefix + ".cuboid.blb", std::vector<double>{-0.35, 0.21, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.brb", std::vector<double>{-0.35, -0.21, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.blt", std::vector<double>{-0.35, 0.21, 0.60}),
      rclcpp::Parameter(prefix + ".cuboid.brt", std::vector<double>{-0.35, -0.21, 0.60}),
    });

    auto node = std::make_shared<rclcpp::Node>(
      "omni_simple_trajectory_generator_test_" + std::to_string(node_id++), options);
    auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto shared_data = std::make_shared<TrajectoryGeneratorSharedData>(buffer);
    shared_data->robot_pose_.header.frame_id = "map";
    shared_data->robot_pose_.child_frame_id = "base_link";
    shared_data->robot_pose_.transform.rotation.w = 1.0;
    shared_data->robot_state_.twist.twist.linear.x = current_linear_x;
    shared_data->robot_state_.twist.twist.linear.y = current_linear_y;
    shared_data->robot_state_.twist.twist.angular.z = current_yaw_rate;
    shared_data->current_allowed_max_linear_speed_ = -1.0;

    OmniSimpleTrajectoryGeneratorTheory generator;
    generator.setSharedData(shared_data);
    generator.initialize(prefix, node);

    TrajectoryGeneratorTheory & base_generator = generator;
    base_generator.initialise();

    std::vector<base_trajectory::Trajectory> trajectories;
    while(base_generator.hasMoreTrajectories()){
      base_trajectory::Trajectory trajectory;
      if(base_generator.nextTrajectory(trajectory)){
        trajectories.push_back(trajectory);
      }
    }
    return trajectories;
  }
};

TEST_F(
  OmniSimpleTrajectoryGeneratorTest,
  RestingRobotGetsForwardLateralAndInPlaceSamples)
{
  const auto trajectories = generate();
  bool has_forward = false;
  bool has_left_strafe = false;
  bool has_right_strafe = false;
  bool has_left_rotation = false;
  bool has_right_rotation = false;

  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    const bool zero_x = std::fabs(trajectory.xv_) <= 1e-6;
    const bool zero_y = std::fabs(trajectory.yv_) <= 1e-6;
    const bool zero_yaw = std::fabs(trajectory.thetav_) <= 1e-6;
    has_forward =
      has_forward || (trajectory.xv_ >= 0.30 - 1e-6 && zero_y && zero_yaw);
    has_left_strafe =
      has_left_strafe || (zero_x && trajectory.yv_ >= 0.20 - 1e-6 && zero_yaw);
    has_right_strafe =
      has_right_strafe || (zero_x && trajectory.yv_ <= -0.20 + 1e-6 && zero_yaw);
    has_left_rotation =
      has_left_rotation || (zero_x && zero_y && trajectory.thetav_ >= 0.40 - 1e-6);
    has_right_rotation =
      has_right_rotation || (zero_x && zero_y && trajectory.thetav_ <= -0.40 + 1e-6);
  }

  EXPECT_TRUE(has_forward);
  EXPECT_TRUE(has_left_strafe);
  EXPECT_TRUE(has_right_strafe);
  EXPECT_TRUE(has_left_rotation);
  EXPECT_TRUE(has_right_rotation);
}

TEST_F(OmniSimpleTrajectoryGeneratorTest, PureLateralTrajectoryMovesSideways)
{
  const auto trajectories = generate();
  const auto lateral = std::find_if(
    trajectories.begin(), trajectories.end(),
    [](const base_trajectory::Trajectory & trajectory) {
      return
        std::fabs(trajectory.xv_) <= 1e-6 &&
        trajectory.yv_ >= 0.20 - 1e-6 &&
        std::fabs(trajectory.thetav_) <= 1e-6;
    });

  ASSERT_NE(lateral, trajectories.end());
  ASSERT_GT(lateral->getPointsSize(), 1U);
  const auto final_pose = lateral->getPoint(lateral->getPointsSize() - 1U).pose;
  EXPECT_NEAR(final_pose.position.x, 0.0, 1e-6);
  EXPECT_GT(final_pose.position.y, 0.25);
}

TEST_F(OmniSimpleTrajectoryGeneratorTest, LateralDynamicWindowPreservesCurrentDirection)
{
  const auto trajectories = generate(OmniTestConfig{}, 0.0, 0.15);

  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    EXPECT_GT(trajectory.yv_, 0.0);
    EXPECT_GE(trajectory.yv_, 0.075 - 1e-6);
    EXPECT_LE(trajectory.yv_, 0.20 + 1e-6);
  }
}

TEST_F(OmniSimpleTrajectoryGeneratorTest, OutOfRangeOdometryProducesOrderedSafeWindow)
{
  const auto trajectories = generate(OmniTestConfig{}, 0.0, 0.50);

  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    EXPECT_NEAR(trajectory.yv_, 0.20, 1e-6);
  }
}

TEST_F(OmniSimpleTrajectoryGeneratorTest, ZeroLateralLimitFailsClosed)
{
  OmniTestConfig config;
  config.min_vel_y = 0.0;
  config.max_vel_y = 0.0;
  const auto trajectories = generate(config);

  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    EXPECT_NEAR(trajectory.yv_, 0.0, 1e-6);
  }
}

TEST_F(OmniSimpleTrajectoryGeneratorTest, InvalidConfigurationFailsFast)
{
  OmniTestConfig invalid_y_bounds;
  invalid_y_bounds.min_vel_y = 0.20;
  invalid_y_bounds.max_vel_y = -0.20;
  EXPECT_THROW(generate(invalid_y_bounds), std::invalid_argument);

  OmniTestConfig invalid_frequency;
  invalid_frequency.controller_frequency = 0.0;
  EXPECT_THROW(generate(invalid_frequency), std::invalid_argument);

  OmniTestConfig invalid_y_samples;
  invalid_y_samples.linear_y_sample = 0.0;
  EXPECT_THROW(generate(invalid_y_samples), std::invalid_argument);

  OmniTestConfig invalid_translational_limit;
  invalid_translational_limit.max_vel_trans =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(generate(invalid_translational_limit), std::invalid_argument);
}

}  // namespace
}  // namespace trajectory_generators
