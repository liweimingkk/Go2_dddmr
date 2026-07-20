#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include <mcl_3dl/posterior_mode.h>

namespace mcl_3dl
{
namespace
{

State6DOF poseAt(const double x, const double yaw = 0.0)
{
  return State6DOF(Vec3(x, 0.0, 0.0), Quat(Vec3(0.0, 0.0, yaw)));
}

TEST(PosteriorMode, UsesConditionalMeanInsteadOfMixtureMidpoint)
{
  const std::vector<State6DOF> states{
    poseAt(-0.1), poseAt(0.1), poseAt(10.0), poseAt(10.2)};
  const std::vector<double> probabilities{0.40, 0.40, 0.10, 0.10};
  const std::vector<float> qualities{0.70f, 0.90f, 0.20f, 0.20f};

  const auto mode = estimatePosteriorMode(
      states, probabilities, qualities, 0.75, 0.35);

  ASSERT_TRUE(mode.valid);
  EXPECT_NEAR(mode.probability_mass, 0.80, 1e-6);
  EXPECT_NEAR(mode.mean.pos_.x_, 0.0, 1e-6);
  EXPECT_NEAR(mode.weighted_match_ratio, 0.80, 1e-6);
  EXPECT_NEAR(mode.xy_std, 0.10, 1e-6);
  EXPECT_LT(mode.mean.pos_.x_, 0.5);
  ASSERT_EQ(mode.members.size(), states.size());
  EXPECT_TRUE(mode.members[0]);
  EXPECT_TRUE(mode.members[1]);
  EXPECT_FALSE(mode.members[2]);
  EXPECT_FALSE(mode.members[3]);
}

TEST(PosteriorMode, ComputesCircularYawMeanAcrossPiBoundary)
{
  constexpr double kPi = 3.14159265358979323846;
  const std::vector<State6DOF> states{
    poseAt(0.0, kPi - 0.05), poseAt(0.0, -kPi + 0.05)};
  const std::vector<double> probabilities{0.50, 0.50};
  const std::vector<float> qualities{0.80f, 0.80f};

  const auto mode = estimatePosteriorMode(
      states, probabilities, qualities, 0.75, 0.20);

  ASSERT_TRUE(mode.valid);
  const double mean_yaw = mode.mean.rot_.getRPY().z_;
  EXPECT_NEAR(std::abs(mean_yaw), kPi, 1e-5);
  EXPECT_NEAR(mode.yaw_std, 0.05, 1e-5);
}

TEST(PosteriorMode, SelectsHighestMassClusterNotHighestWeightParticle)
{
  const std::vector<State6DOF> states{
    poseAt(-0.35), poseAt(-0.25), poseAt(-0.15), poseAt(-0.05),
    poseAt(0.05), poseAt(0.15), poseAt(0.25), poseAt(0.35),
    poseAt(10.0)};
  const std::vector<double> probabilities{
    0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.20};
  const std::vector<float> qualities{
    0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.8f, 0.9f};

  const auto mode = estimatePosteriorMode(
      states, probabilities, qualities, 0.75, 0.35);

  ASSERT_TRUE(mode.valid);
  EXPECT_NEAR(mode.probability_mass, 0.80, 1e-6);
  EXPECT_NEAR(mode.mean.pos_.x_, 0.0, 1e-6);
  EXPECT_LT(std::abs(mode.anchor.pos_.x_), 1.0);
}

TEST(PosteriorMode, ZeroWeightBridgeCannotMergeSeparateModes)
{
  const std::vector<State6DOF> states{
    poseAt(-0.7), poseAt(0.0), poseAt(0.7)};
  const std::vector<double> probabilities{0.50, 0.0, 0.50};
  const std::vector<float> qualities{0.8f, 0.8f, 0.8f};

  const auto mode = estimatePosteriorMode(
      states, probabilities, qualities, 0.75, 0.35);

  ASSERT_TRUE(mode.valid);
  EXPECT_NEAR(mode.probability_mass, 0.50, 1e-6);
  EXPECT_NEAR(std::abs(mode.mean.pos_.x_), 0.7, 1e-6);
  EXPECT_FALSE(mode.members[0] && mode.members[2]);
}

TEST(PosteriorMode, LowWeightBridgeCannotMergeSeparateModes)
{
  const std::vector<State6DOF> states{
    poseAt(-0.7), poseAt(0.0), poseAt(0.7)};
  const std::vector<double> probabilities{0.495, 0.01, 0.495};
  const std::vector<float> qualities{0.8f, 0.8f, 0.8f};

  const auto mode = estimatePosteriorMode(
      states, probabilities, qualities, 0.75, 0.35);

  ASSERT_TRUE(mode.valid);
  EXPECT_NEAR(mode.probability_mass, 0.505, 1e-6);
  EXPECT_GT(std::abs(mode.mean.pos_.x_), 0.68);
  EXPECT_FALSE(mode.members[0] && mode.members[2]);
}

TEST(PosteriorMode, RejectsNonFiniteInputs)
{
  std::vector<State6DOF> states{poseAt(0.0), poseAt(0.1)};
  states[1].pos_.x_ = std::numeric_limits<float>::quiet_NaN();
  const auto mode = estimatePosteriorMode(
      states, {0.5, 0.5}, {0.8f, 0.8f}, 0.75, 0.35);
  EXPECT_FALSE(mode.valid);
}

TEST(PosteriorMode, BinIndexRejectsNonFiniteAndOutOfRangeValues)
{
  long long index = 0;
  EXPECT_TRUE(posteriorBinIndex(0.75, 0.50, index));
  EXPECT_EQ(index, 1);
  EXPECT_FALSE(posteriorBinIndex(
      std::numeric_limits<double>::infinity(), 0.50, index));
  EXPECT_FALSE(posteriorBinIndex(
      std::numeric_limits<double>::max(), 0.50, index));
  EXPECT_FALSE(posteriorBinIndex(1.0, 0.0, index));
}

}  // namespace
}  // namespace mcl_3dl
