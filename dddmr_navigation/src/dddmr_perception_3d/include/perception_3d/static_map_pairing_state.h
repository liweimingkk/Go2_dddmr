/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2024, DDDMobileRobot
 * All rights reserved.
 */

#ifndef PERCEPTION_3D__STATIC_MAP_PAIRING_STATE_H_
#define PERCEPTION_3D__STATIC_MAP_PAIRING_STATE_H_

#include <cstddef>
#include <cstdint>
#include <limits>

namespace perception_3d
{

struct StaticMapPairCommit
{
  bool committed{false};
  std::uint64_t pair_generation{0U};
  std::uint64_t map_message_generation{0U};
  std::uint64_t ground_message_generation{0U};
};

inline bool staticMapPayloadPairIsValid(
  const std::size_t map_point_count,
  const std::size_t ground_point_count) noexcept
{
  return map_point_count > 0U && ground_point_count > 0U;
}

// Tracks message epochs, never point counts. One pair may become visible only
// after at least one new map and one new ground have both arrived since the
// previous commit. Repeated messages replace their corresponding staged
// payload; the latest complete pair is committed.
class StaticMapPairingState
{
public:
  void noteMapMessage() noexcept
  {
    incrementSaturating(map_message_generation_);
    map_pending_ = true;
  }

  void noteGroundMessage() noexcept
  {
    incrementSaturating(ground_message_generation_);
    ground_pending_ = true;
  }

  bool pairReady() const noexcept
  {
    return map_pending_ && ground_pending_;
  }

  StaticMapPairCommit commit() noexcept
  {
    StaticMapPairCommit result;
    if (!pairReady()) {
      return result;
    }
    map_pending_ = false;
    ground_pending_ = false;
    incrementSaturating(pair_generation_);
    result.committed = true;
    result.pair_generation = pair_generation_;
    result.map_message_generation = map_message_generation_;
    result.ground_message_generation = ground_message_generation_;
    return result;
  }

  std::uint64_t pairGeneration() const noexcept
  {
    return pair_generation_;
  }

private:
  static void incrementSaturating(std::uint64_t & value) noexcept
  {
    if (value != std::numeric_limits<std::uint64_t>::max()) {
      ++value;
    }
  }

  bool map_pending_{false};
  bool ground_pending_{false};
  std::uint64_t map_message_generation_{0U};
  std::uint64_t ground_message_generation_{0U};
  std::uint64_t pair_generation_{0U};
};

}  // namespace perception_3d

#endif  // PERCEPTION_3D__STATIC_MAP_PAIRING_STATE_H_
