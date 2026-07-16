/*
 * Copyright (c) 2016-2017, the mcl_3dl authors
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

#ifndef MCL_3DL_PF_H
#define MCL_3DL_PF_H
#include <algorithm>
#include <cassert>
#include <cmath>
#include <functional>
#include <iterator>
#include <numeric>
#include <random>
#include <vector>

#include <mcl_3dl/noise_generators/diagonal_noise_generator.h>

// Required OpenMP header
#include <omp.h> 

namespace mcl_3dl
{
namespace pf
{
template <typename FLT_TYPE = float>
class ParticleBase
{
public:
  virtual FLT_TYPE& operator[](const size_t i) = 0;
  virtual size_t size() const = 0;
  virtual void normalize() = 0;
  template <typename T>
  T operator+(const T& a)
  {
    T in = a;
    T ret;
    for (size_t i = 0; i < size(); i++)
    {
      ret[i] = (*this)[i] + in[i];
    }
    return ret;
  }
  template <typename T>
  FLT_TYPE covElement(
      const T& e, const size_t& j, const size_t& k)
  {
    T exp = e;
    return ((*this)[k] - exp[k]) * ((*this)[j] - exp[j]);
  }

  template <typename T, typename RANDOM_ENGINE, typename NOISE_GEN>
  static T generateNoise(RANDOM_ENGINE& engine, const NOISE_GEN& gen)
  {
    const auto org_noise = gen(engine);
    T noise;
    for (size_t i = 0; i < noise.size(); i++)
    {
      noise[i] = org_noise[i];
    }
    return noise;
  }
};

template <typename T, typename FLT_TYPE = float>
class Particle
{
public:
  Particle()
  {
    probability_ = 0.0;
    probability_bias_ = 0.0;
  }
  explicit Particle(FLT_TYPE prob)
  {
    accum_probability_ = prob;
  }
  T state_;
  FLT_TYPE probability_;
  FLT_TYPE probability_bias_;
  FLT_TYPE accum_probability_;
  bool operator<(const Particle& p2) const
  {
    return this->accum_probability_ < p2.accum_probability_;
  }
};

template <typename T, typename FLT_TYPE = float>
class ParticleWeightedMean
{
protected:
  T e_;
  FLT_TYPE p_sum_;

public:
  ParticleWeightedMean()
    : e_()
    , p_sum_(0.0)
  {
  }

  void add(const T& s, const FLT_TYPE& prob)
  {
    p_sum_ += prob;

    T e1 = s;
    for (size_t i = 0; i < e1.size(); i++)
    {
      e1[i] = e1[i] * prob;
    }
    e_ = e1 + e_;
  }

  T getMean()
  {
    assert(p_sum_ > 0.0);

    T s = e_;

    for (size_t i = 0; i < s.size(); i++)
    {
      s[i] = s[i] / p_sum_;
    }

    return s;
  }

  FLT_TYPE getTotalProbability()
  {
    return p_sum_;
  }
};

template <typename T, typename FLT_TYPE = float, typename MEAN = ParticleWeightedMean<T, FLT_TYPE>,
          typename RANDOM_ENGINE = std::default_random_engine>
class ParticleFilter
{
public:
  // random_seed is used to generate same results in tests.
  explicit ParticleFilter(const int num_particles, const unsigned int random_seed = std::random_device()())
    : engine_(random_seed)
  {
    particles_.resize(num_particles);
  }
  void init(T mean, T sigma)
  {
    return initUsingNoiseGenerator(DiagonalNoiseGenerator<FLT_TYPE>(mean, sigma));
  }
  template <typename GEN>
  void initUsingNoiseGenerator(const GEN& generator)
  {
    for (auto& p : particles_)
    {
      p.state_ = T::template generateNoise<T>(engine_, generator);
      p.probability_ = 1.0 / particles_.size();
    }
  }
  bool resample(T sigma)
  {
    return resampleUsingNoiseGenerator(
        DiagonalNoiseGenerator<FLT_TYPE>(T(), sigma));
  }
  bool resampleToSize(T sigma, const size_t num_particles)
  {
    return resampleToSizeUsingNoiseGenerator(
        DiagonalNoiseGenerator<FLT_TYPE>(T(), sigma), num_particles);
  }
  template <typename GEN>
  bool resampleUsingNoiseGenerator(const GEN& generator)
  {
    if (particles_.empty())
    {
      return false;
    }

    FLT_TYPE accum = 0;
    for (auto& p : particles_)
    {
      if (!std::isfinite(p.probability_) || p.probability_ < 0.0)
      {
        return false;
      }
      accum += p.probability_;
      p.accum_probability_ = accum;
    }
    if (!std::isfinite(accum) || accum <= 0.0)
    {
      return false;
    }

    particles_dup_ = particles_;
    std::sort(particles_dup_.begin(), particles_dup_.end());
    const FLT_TYPE pstep = accum / particles_.size();
    const FLT_TYPE initial_p =
        std::uniform_real_distribution<FLT_TYPE>(0.0, pstep)(engine_);
    auto it = particles_dup_.begin();
    auto it_prev = particles_dup_.begin();
    const FLT_TYPE prob = 1.0 / particles_.size();
    for (size_t i = 0; i < particles_.size(); ++i)
    {
      auto& p = particles_[i];
      const FLT_TYPE pscan = pstep * i + initial_p;
      it = std::lower_bound(
          it, particles_dup_.end(), Particle<T, FLT_TYPE>(pscan));
      p.probability_ = prob;
      if (it == particles_dup_.end())
      {
        p.state_ = it_prev->state_;
        continue;
      }
      if (it == it_prev)
      {
        p.state_ = it->state_ + T::template generateNoise<T>(engine_, generator);
        p.state_.normalize();
      }
      else
      {
        p.state_ = it->state_;
      }
      it_prev = it;
    }
    return true;
  }
  template <typename GEN>
  bool resampleToSizeUsingNoiseGenerator(
      const GEN& generator, const size_t num_particles)
  {
    if (particles_.empty() || num_particles == 0)
    {
      return false;
    }

    FLT_TYPE accum = 0;
    for (auto& p : particles_)
    {
      if (!std::isfinite(p.probability_) || p.probability_ < 0.0)
      {
        return false;
      }
      accum += p.probability_;
      p.accum_probability_ = accum;
    }

    if (!std::isfinite(accum) || accum <= 0.0)
    {
      return false;
    }

    particles_dup_ = particles_;
    const FLT_TYPE pstep = accum / num_particles;
    const FLT_TYPE initial_p = std::uniform_real_distribution<FLT_TYPE>(0.0, pstep)(engine_);
    auto it = particles_dup_.begin();
    auto last_positive = particles_dup_.end();
    for (auto candidate = particles_dup_.begin();
         candidate != particles_dup_.end(); ++candidate)
    {
      if (candidate->probability_ > 0.0)
      {
        last_positive = candidate;
      }
    }
    if (last_positive == particles_dup_.end())
    {
      return false;
    }
    std::vector<size_t> parent_use_count(particles_dup_.size(), 0);
    std::vector<Particle<T, FLT_TYPE>> resampled(num_particles);
    const FLT_TYPE prob = 1.0 / num_particles;
    for (size_t i = 0; i < num_particles; ++i)
    {
      auto& p = resampled[i];
      const FLT_TYPE pscan = pstep * i + initial_p;
      it = std::upper_bound(it, particles_dup_.end(), Particle<T, FLT_TYPE>(pscan));
      if (it == particles_dup_.end())
      {
        it = last_positive;
      }

      const size_t parent_index = static_cast<size_t>(
          std::distance(particles_dup_.begin(), it));
      p.state_ = it->state_;
      if (parent_use_count[parent_index]++ > 0)
      {
        p.state_ = p.state_ + T::template generateNoise<T>(engine_, generator);
        p.state_.normalize();
      }
      p.probability_ = prob;
      p.probability_bias_ = 1.0;
      p.accum_probability_ = 0.0;
    }
    particles_.swap(resampled);
    return true;
  }
  void noise(T sigma)
  {
    addNoiseUsingNoiseGenerator(DiagonalNoiseGenerator<FLT_TYPE>(T(), sigma));
  }
  template <typename GEN>
  void addNoiseUsingNoiseGenerator(const GEN& generator)
  {
    for (auto& p : particles_)
    {
      p.state_ = p.state_ + T::template generateNoise<T>(engine_, generator);
    }
  }
  void predict(std::function<void(T&)> model)
  {
    for (auto& p : particles_)
    {
      model(p.state_);
    }
  }
  void bias(std::function<void(const T&, float& p_bias)> prob)
  {
    for (auto& p : particles_)
    {
      prob(p.state_, p.probability_bias_);
    }
  }
  template <typename PREDICATE>
  bool conditionOn(const PREDICATE& keep)
  {
    FLT_TYPE kept_probability = 0.0;
    for (const auto& particle : particles_)
    {
      if (!std::isfinite(particle.probability_) || particle.probability_ < 0.0)
      {
        return false;
      }
      if (keep(particle.state_))
      {
        kept_probability += particle.probability_;
      }
    }
    if (!std::isfinite(kept_probability) || kept_probability <= 0.0)
    {
      return false;
    }

    for (auto& particle : particles_)
    {
      particle.probability_ = keep(particle.state_) ?
          particle.probability_ / kept_probability : 0.0;
    }
    return true;
  }
  bool measure(std::function<FLT_TYPE(const T&)> likelihood)
  {
    return measureWithIndex(
        [&likelihood](const size_t, const T& state)
        {
          return likelihood(state);
        });
  }
  bool measureWithIndex(std::function<FLT_TYPE(const size_t, const T&)> likelihood)
  {
    if (particles_.empty())
    {
      return false;
    }

    FLT_TYPE sum = 0;
    std::vector<FLT_TYPE> prob(particles_.size());
    
    //@ omp for particles rating. Test 200 particles total rating time from 0.14 to 0.03 at i9-11950h 2.6Ghz computer
    //@ However, increase total cpu usage since we are now able to process in real time without losing frame.
    #pragma omp parallel for
    for (size_t i=0; i<particles_.size(); i++)
    {
      prob[i] = particles_[i].probability_ * likelihood(i, particles_[i].state_);
    }

    for (size_t i=0; i<particles_.size(); i++)
    {
      if (!std::isfinite(prob[i]) || prob[i] < 0.0)
      {
        return false;
      }
      sum+=prob[i];
    }

    if (std::isfinite(sum) && sum > 0.0)
    {
      for (size_t i = 0; i < particles_.size(); ++i)
      {
        particles_[i].probability_ = prob[i] / sum;
      }
      return true;
    }
    return false;
  }
  T expectation(const FLT_TYPE pass_ratio = 1.0)
  {
    MEAN mean;

    if (pass_ratio < 1.0)
      std::sort(particles_.rbegin(), particles_.rend());
    for (auto& p : particles_)
    {
      mean.add(p.state_, p.probability_);
      if (mean.getTotalProbability() > pass_ratio)
        break;
    }
    return mean.getMean();
  }
  T expectationBiased()
  {
    MEAN mean;

    for (auto& p : particles_)
    {
      mean.add(p.state_, p.probability_ * p.probability_bias_);
    }
    return mean.getMean();
  }
  std::vector<T> covariance(
      const FLT_TYPE pass_ratio = 1.0,
      const FLT_TYPE random_sample_ratio = 1.0)
  {
    T e = expectation(pass_ratio);
    FLT_TYPE p_sum = 0;
    std::vector<T> cov;
    cov.resize(e.size());

    size_t p_num = 0;
    for (auto& p : particles_)
    {
      p_num++;
      p_sum += p.probability_;
      if (p_sum > pass_ratio)
        break;
    }

    std::vector<size_t> indices(p_num);
    std::iota(indices.begin(), indices.end(), 0);
    if (random_sample_ratio < 1.0)
    {
      std::shuffle(indices.begin(), indices.end(), engine_);

      const size_t sample_num =
          std::min(
              p_num,
              std::max(
                  size_t(0),
                  static_cast<size_t>(p_num * random_sample_ratio)));
      indices.resize(sample_num);
    }

    p_sum = 0.0;
    for (size_t i : indices)
    {
      auto& p = particles_[i];
      p_sum += p.probability_;
      for (size_t j = 0; j < ie_.size(); j++)
      {
        for (size_t k = j; k < ie_.size(); k++)
        {
          cov[k][j] = cov[j][k] += p.state_.covElement(e, j, k) * p.probability_;
        }
      }
    }
    for (size_t j = 0; j < ie_.size(); j++)
    {
      for (size_t k = 0; k < ie_.size(); k++)
      {
        cov[k][j] /= p_sum;
      }
    }

    return cov;
  }
  T max()
  {
    T* m = &particles_[0].state_;
    FLT_TYPE max_probability = particles_[0].probability_;
    for (auto& p : particles_)
    {
      if (max_probability < p.probability_)
      {
        max_probability = p.probability_;
        m = &p.state_;
      }
    }
    return *m;
  }
  T maxBiased()
  {
    T* m = &particles_[0].state_;
    FLT_TYPE max_probability =
        particles_[0].probability_ * particles_[0].probability_bias_;
    for (auto& p : particles_)
    {
      const FLT_TYPE prob = p.probability_ * p.probability_bias_;
      if (max_probability < prob)
      {
        max_probability = prob;
        m = &p.state_;
      }
    }
    return *m;
  }
  T getParticle(const size_t i) const
  {
    return particles_[i].state_;
  }
  FLT_TYPE getParticleProbability(const size_t i) const
  {
    return particles_[i].probability_;
  }
  FLT_TYPE effectiveSampleSize() const
  {
    FLT_TYPE probability_sum = 0.0;
    FLT_TYPE squared_probability_sum = 0.0;
    for (const auto& particle : particles_)
    {
      if (!std::isfinite(particle.probability_) || particle.probability_ < 0.0)
      {
        return 0.0;
      }
      probability_sum += particle.probability_;
      squared_probability_sum += particle.probability_ * particle.probability_;
    }
    if (!std::isfinite(probability_sum) || !std::isfinite(squared_probability_sum) ||
        probability_sum <= 0.0 || squared_probability_sum <= 0.0)
    {
      return 0.0;
    }
    return probability_sum * probability_sum / squared_probability_sum;
  }
  FLT_TYPE normalizedEffectiveSampleSize() const
  {
    if (particles_.empty())
    {
      return 0.0;
    }
    return effectiveSampleSize() / static_cast<FLT_TYPE>(particles_.size());
  }
  size_t getParticleSize() const
  {
    return particles_.size();
  }
  void resizeParticle(const size_t num)
  {
    FLT_TYPE accum = 0;
    for (auto& p : particles_)
    {
      accum += p.probability_;
      p.accum_probability_ = accum;
    }

    particles_dup_ = particles_;
    std::sort(particles_dup_.begin(), particles_dup_.end());

    FLT_TYPE pstep = accum / num;
    FLT_TYPE pscan = 0;
    auto it = particles_dup_.begin();
    auto it_prev = particles_dup_.begin();

    particles_.resize(num);

    FLT_TYPE prob = 1.0 / num;
    for (auto& p : particles_)
    {
      pscan += pstep;
      it = std::lower_bound(it, particles_dup_.end(),
                            Particle<T, FLT_TYPE>(pscan));
      p.probability_ = prob;
      if (it == particles_dup_.end())
      {
        p.state_ = it_prev->state_;
        continue;
      }
      else
      {
        p.state_ = it->state_;
      }
      it_prev = it;
    }
  }
  typename std::vector<Particle<T, FLT_TYPE>>::iterator appendParticle(const size_t num)
  {
    const size_t size_orig = particles_.size();
    particles_.resize(size_orig + num);
    return begin() + size_orig;
  }
  typename std::vector<Particle<T, FLT_TYPE>>::iterator begin()
  {
    return particles_.begin();
  }
  typename std::vector<Particle<T, FLT_TYPE>>::iterator end()
  {
    return particles_.end();
  }

protected:
  std::vector<Particle<T, FLT_TYPE>> particles_;
  std::vector<Particle<T, FLT_TYPE>> particles_dup_;
  RANDOM_ENGINE engine_;
  T ie_;
};

}  // namespace pf
}  // namespace mcl_3dl

#endif  // MCL_3DL_PF_H
