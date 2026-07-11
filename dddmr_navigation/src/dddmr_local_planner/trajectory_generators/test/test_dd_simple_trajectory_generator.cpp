#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <trajectory_generators/dd_simple_trajectory_generator_theory.h>
#include <trajectory_generators/trajectory_shared_data.h>

namespace trajectory_generators
{
namespace
{

class DDSimpleTrajectoryGeneratorTest : public ::testing::Test
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
    const double current_linear_speed,
    const bool enable_in_place_rotation = true,
    const double allowed_max_linear_speed = -1.0)
  {
    static int node_id = 0;
    const std::string prefix = "generator";
    rclcpp::NodeOptions options;
    options.parameter_overrides({
      rclcpp::Parameter(prefix + ".min_vel_x", 0.20),
      rclcpp::Parameter(prefix + ".max_vel_x", 0.30),
      rclcpp::Parameter(prefix + ".min_vel_theta", 0.04),
      rclcpp::Parameter(prefix + ".max_vel_theta", 0.50),
      rclcpp::Parameter(prefix + ".enable_in_place_rotation", enable_in_place_rotation),
      rclcpp::Parameter(prefix + ".in_place_rotation_max_linear_speed", 0.05),
      rclcpp::Parameter(prefix + ".acc_lim_x", 3.0),
      rclcpp::Parameter(prefix + ".acc_lim_theta", 1.0),
      rclcpp::Parameter(prefix + ".prune_forward", 3.0),
      rclcpp::Parameter(prefix + ".prune_backward", 1.0),
      rclcpp::Parameter(prefix + ".deceleration_ratio", 2.0),
      rclcpp::Parameter(prefix + ".use_motor_constraint", false),
      rclcpp::Parameter(prefix + ".max_motor_shaft_rpm", 3000.0),
      rclcpp::Parameter(prefix + ".wheel_diameter", 0.16),
      rclcpp::Parameter(prefix + ".gear_ratio", 1.0),
      rclcpp::Parameter(prefix + ".robot_radius", 0.25),
      rclcpp::Parameter(prefix + ".controller_frequency", 10.0),
      rclcpp::Parameter(prefix + ".sim_time", 1.5),
      rclcpp::Parameter(prefix + ".linear_x_sample", 5.0),
      rclcpp::Parameter(prefix + ".angular_z_sample", 10.0),
      rclcpp::Parameter(prefix + ".sim_granularity", 0.05),
      rclcpp::Parameter(prefix + ".angular_sim_granularity", 0.025),
      rclcpp::Parameter(prefix + ".cuboid.flb", std::vector<double>{0.42, 0.30, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.frb", std::vector<double>{0.42, -0.30, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.flt", std::vector<double>{0.42, 0.30, 0.60}),
      rclcpp::Parameter(prefix + ".cuboid.frt", std::vector<double>{0.42, -0.30, 0.60}),
      rclcpp::Parameter(prefix + ".cuboid.blb", std::vector<double>{-0.35, 0.30, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.brb", std::vector<double>{-0.35, -0.30, -0.20}),
      rclcpp::Parameter(prefix + ".cuboid.blt", std::vector<double>{-0.35, 0.30, 0.60}),
      rclcpp::Parameter(prefix + ".cuboid.brt", std::vector<double>{-0.35, -0.30, 0.60}),
    });

    auto node = std::make_shared<rclcpp::Node>(
      "dd_simple_trajectory_generator_test_" + std::to_string(node_id++), options);
    auto buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto shared_data = std::make_shared<TrajectoryGeneratorSharedData>(buffer);
    shared_data->robot_pose_.header.frame_id = "map";
    shared_data->robot_pose_.child_frame_id = "base_link";
    shared_data->robot_pose_.transform.rotation.w = 1.0;
    shared_data->robot_state_.twist.twist.linear.x = current_linear_speed;
    shared_data->current_allowed_max_linear_speed_ = allowed_max_linear_speed;

    DDSimpleTrajectoryGeneratorTheory generator;
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

TEST_F(DDSimpleTrajectoryGeneratorTest, RestingRobotGetsExecutableForwardAndInPlaceSamples)
{
  const auto trajectories = generate(0.0);
  bool has_left_rotation = false;
  bool has_right_rotation = false;
  bool has_forward = false;

  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    if(std::fabs(trajectory.xv_) <= 1e-6){
      has_left_rotation = has_left_rotation || trajectory.thetav_ > 0.0;
      has_right_rotation = has_right_rotation || trajectory.thetav_ < 0.0;
      EXPECT_GE(std::fabs(trajectory.thetav_), 0.04 - 1e-6);
    }else{
      has_forward = true;
      EXPECT_GE(trajectory.xv_, 0.20 - 1e-6);
    }
  }

  EXPECT_TRUE(has_left_rotation);
  EXPECT_TRUE(has_right_rotation);
  EXPECT_TRUE(has_forward);
}

TEST_F(DDSimpleTrajectoryGeneratorTest, MovingRobotDoesNotReceiveInstantInPlaceSample)
{
  const auto trajectories = generate(0.10);
  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    EXPECT_GT(trajectory.xv_, 0.0);
    EXPECT_GE(trajectory.xv_, 0.20 - 1e-6);
  }
}

TEST_F(DDSimpleTrajectoryGeneratorTest, InPlaceSampleRotatesWithoutTranslating)
{
  const auto trajectories = generate(0.0);
  const auto rotation = std::find_if(
    trajectories.begin(), trajectories.end(),
    [](const base_trajectory::Trajectory & trajectory) {
      return std::fabs(trajectory.xv_) <= 1e-6 && trajectory.thetav_ > 0.0;
    });

  ASSERT_NE(rotation, trajectories.end());
  ASSERT_GT(rotation->getPointsSize(), 1U);
  for(unsigned int index = 0; index < rotation->getPointsSize(); ++index){
    const auto pose = rotation->getPoint(index).pose;
    EXPECT_NEAR(pose.position.x, 0.0, 1e-6);
    EXPECT_NEAR(pose.position.y, 0.0, 1e-6);
  }

  const auto final_orientation =
    rotation->getPoint(rotation->getPointsSize() - 1U).pose.orientation;
  const double final_yaw = 2.0 * std::atan2(
    final_orientation.z, final_orientation.w);
  EXPECT_GT(final_yaw, 0.05);
}

TEST_F(DDSimpleTrajectoryGeneratorTest, InPlaceSamplesCanBeDisabled)
{
  const auto trajectories = generate(0.0, false);
  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    EXPECT_GE(trajectory.xv_, 0.20 - 1e-6);
  }
}

TEST_F(DDSimpleTrajectoryGeneratorTest, SubExecutableSpeedLimitProducesOnlyInPlaceSamples)
{
  const auto trajectories = generate(0.0, true, 0.15);
  ASSERT_FALSE(trajectories.empty());
  for(const auto & trajectory : trajectories){
    EXPECT_NEAR(trajectory.xv_, 0.0, 1e-6);
    EXPECT_GE(std::fabs(trajectory.thetav_), 0.04 - 1e-6);
  }
}

}  // namespace
}  // namespace trajectory_generators
