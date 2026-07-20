#include <gtest/gtest.h>

#include <mcl_3dl/pf.h>
#include <mcl_3dl/state_6dof.h>

namespace mcl_3dl
{
namespace
{

using TestFilter = pf::ParticleFilter<
    State6DOF, float, ParticleWeightedMeanQuat, std::default_random_engine>;

State6DOF stateAt(const double x)
{
  return State6DOF(Vec3(x, 0.0, 0.0), Quat(Vec3(0.0, 0.0, 0.0)));
}

void initializeParticles(TestFilter& filter)
{
  std::size_t index = 0;
  for (auto particle = filter.begin(); particle != filter.end(); ++particle, ++index)
  {
    particle->state_ = stateAt(static_cast<double>(index));
    particle->probability_ = 0.25f;
    particle->probability_bias_ = 1.0f;
  }
}

TEST(ParticleFilterPosterior, EffectiveSampleSizeDetectsWeightCollapse)
{
  TestFilter filter(4, 7U);
  initializeParticles(filter);
  EXPECT_FLOAT_EQ(filter.effectiveSampleSize(), 4.0f);
  EXPECT_FLOAT_EQ(filter.normalizedEffectiveSampleSize(), 1.0f);

  std::size_t index = 0;
  for (auto particle = filter.begin(); particle != filter.end(); ++particle, ++index)
  {
    particle->probability_ = index == 0 ? 1.0f : 0.0f;
  }
  EXPECT_FLOAT_EQ(filter.effectiveSampleSize(), 1.0f);
  EXPECT_FLOAT_EQ(filter.normalizedEffectiveSampleSize(), 0.25f);
}

TEST(ParticleFilterPosterior, InvalidMeasurementLeavesPosteriorUnchanged)
{
  TestFilter filter(4, 7U);
  initializeParticles(filter);

  EXPECT_FALSE(filter.measure([](const State6DOF&) {return 0.0f;}));
  for (std::size_t i = 0; i < filter.getParticleSize(); ++i)
  {
    EXPECT_FLOAT_EQ(filter.getParticleProbability(i), 0.25f);
  }
}

TEST(ParticleFilterPosterior, IndexedMeasurementNormalizesWeights)
{
  TestFilter filter(4, 7U);
  initializeParticles(filter);

  ASSERT_TRUE(filter.measureWithIndex(
      [](const std::size_t index, const State6DOF&)
      {
        return index == 0 ? 4.0f : 1.0f;
      }));
  EXPECT_NEAR(filter.getParticleProbability(0), 4.0 / 7.0, 1e-6);
  EXPECT_NEAR(filter.getParticleProbability(1), 1.0 / 7.0, 1e-6);
}

TEST(ParticleFilterPosterior, ResamplesDirectlyToRequestedPopulation)
{
  TestFilter filter(4, 7U);
  initializeParticles(filter);
  filter.begin()->probability_ = 0.90f;
  (filter.begin() + 1)->probability_ = 0.05f;
  (filter.begin() + 2)->probability_ = 0.03f;
  (filter.begin() + 3)->probability_ = 0.02f;

  const State6DOF zero_sigma(
      Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
  ASSERT_TRUE(filter.resampleToSize(zero_sigma, 2));
  ASSERT_EQ(filter.getParticleSize(), 2U);
  EXPECT_FLOAT_EQ(filter.getParticleProbability(0), 0.5f);
  EXPECT_FLOAT_EQ(filter.getParticleProbability(1), 0.5f);
  EXPECT_NEAR(filter.getParticle(0).pos_.x_, 0.0, 1e-6);
  EXPECT_NEAR(filter.getParticle(1).pos_.x_, 0.0, 1e-6);
}

TEST(ParticleFilterPosterior, VariableResampleSkipsZeroWeightParents)
{
  TestFilter filter(4, 7U);
  initializeParticles(filter);
  filter.begin()->probability_ = 0.0f;
  (filter.begin() + 1)->probability_ = 1.0f;
  (filter.begin() + 2)->probability_ = 0.0f;
  (filter.begin() + 3)->probability_ = 0.0f;

  const State6DOF zero_sigma(
      Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
  ASSERT_TRUE(filter.resampleToSize(zero_sigma, 4));
  for (std::size_t i = 0; i < filter.getParticleSize(); ++i)
  {
    EXPECT_NEAR(filter.getParticle(i).pos_.x_, 1.0, 1e-6);
  }
}

TEST(ParticleFilterPosterior, ConditionsAndResamplesOnlyFromSelectedMode)
{
  TestFilter filter(4, 7U);
  initializeParticles(filter);
  filter.begin()->state_ = stateAt(-0.1);
  (filter.begin() + 1)->state_ = stateAt(0.1);
  (filter.begin() + 2)->state_ = stateAt(10.0);
  (filter.begin() + 3)->state_ = stateAt(10.2);
  filter.begin()->probability_ = 0.40f;
  (filter.begin() + 1)->probability_ = 0.40f;
  (filter.begin() + 2)->probability_ = 0.10f;
  (filter.begin() + 3)->probability_ = 0.10f;

  ASSERT_TRUE(filter.conditionOn(
      [](const State6DOF& state) {return std::abs(state.pos_.x_) < 1.0;}));
  EXPECT_FLOAT_EQ(filter.getParticleProbability(0), 0.5f);
  EXPECT_FLOAT_EQ(filter.getParticleProbability(1), 0.5f);
  EXPECT_FLOAT_EQ(filter.getParticleProbability(2), 0.0f);
  EXPECT_FLOAT_EQ(filter.getParticleProbability(3), 0.0f);

  const State6DOF zero_sigma(
      Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
  ASSERT_TRUE(filter.resampleToSize(zero_sigma, 4));
  for (std::size_t i = 0; i < filter.getParticleSize(); ++i)
  {
    EXPECT_LT(std::abs(filter.getParticle(i).pos_.x_), 1.0);
  }
}

TEST(ParticleFilterPosterior, VariableResampleNeverSelectsInterleavedZeroWeights)
{
  const State6DOF zero_sigma(
      Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
  for (unsigned int seed = 1; seed <= 64; ++seed)
  {
    TestFilter filter(4, seed);
    initializeParticles(filter);
    filter.begin()->probability_ = 0.0f;
    (filter.begin() + 1)->probability_ = 0.4f;
    (filter.begin() + 2)->probability_ = 0.0f;
    (filter.begin() + 3)->probability_ = 0.6f;

    ASSERT_TRUE(filter.resampleToSize(zero_sigma, 64));
    for (std::size_t i = 0; i < filter.getParticleSize(); ++i)
    {
      const double x = filter.getParticle(i).pos_.x_;
      EXPECT_TRUE(std::abs(x - 1.0) < 1e-6 || std::abs(x - 3.0) < 1e-6)
          << "seed=" << seed << " particle=" << i << " x=" << x;
    }
  }
}

}  // namespace
}  // namespace mcl_3dl
