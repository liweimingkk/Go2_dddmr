/*
 * Copyright (c) 2026, DDDMR Navigation contributors
 * All rights reserved.
 */

#ifndef MCL_3DL_RESIDUAL_METRICS_H
#define MCL_3DL_RESIDUAL_METRICS_H

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace mcl_3dl
{

struct ResidualMetrics
{
  double capped{std::numeric_limits<double>::infinity()};
  double matched{std::numeric_limits<double>::infinity()};
  double match_ratio{0.0};
  std::size_t total_count{0};
  std::size_t matched_count{0};
};

class ResidualAccumulator
{
public:
  explicit ResidualAccumulator(const double distance_cap)
    : distance_cap_(std::max(0.0, distance_cap))
  {
  }

  void add(const double nearest_distance, const bool matched)
  {
    ++total_count_;
    if (!std::isfinite(nearest_distance) || nearest_distance < 0.0)
    {
      capped_sum_ += distance_cap_;
      return;
    }

    const double distance = std::min(nearest_distance, distance_cap_);
    capped_sum_ += distance;
    if (matched)
    {
      matched_sum_ += distance;
      ++matched_count_;
    }
  }

  ResidualMetrics metrics() const
  {
    ResidualMetrics result;
    result.total_count = total_count_;
    result.matched_count = matched_count_;
    if (total_count_ == 0)
    {
      return result;
    }

    result.capped = capped_sum_ / static_cast<double>(total_count_);
    result.match_ratio =
        static_cast<double>(matched_count_) / static_cast<double>(total_count_);
    if (matched_count_ > 0)
    {
      result.matched = matched_sum_ / static_cast<double>(matched_count_);
    }
    return result;
  }

private:
  double distance_cap_{0.0};
  double capped_sum_{0.0};
  double matched_sum_{0.0};
  std::size_t total_count_{0};
  std::size_t matched_count_{0};
};

}  // namespace mcl_3dl

#endif  // MCL_3DL_RESIDUAL_METRICS_H
