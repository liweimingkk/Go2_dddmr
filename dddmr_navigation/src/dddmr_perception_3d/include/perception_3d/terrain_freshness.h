#ifndef PERCEPTION_3D__TERRAIN_FRESHNESS_H_
#define PERCEPTION_3D__TERRAIN_FRESHNESS_H_

#include <cmath>
#include <cstdint>

namespace perception_3d
{

// Evaluate the timestamp that belongs to the exact cloud copied into a terrain
// build.  A newer callback must never make an older copied cloud look fresh.
inline bool capturedTerrainInputIsFresh(
  const bool present,
  const std::int64_t now_nanoseconds,
  const std::int64_t captured_stamp_nanoseconds,
  const double max_age_sec) noexcept
{
  if (!present || now_nanoseconds < 0 || captured_stamp_nanoseconds <= 0 ||
    now_nanoseconds < captured_stamp_nanoseconds || !std::isfinite(max_age_sec) ||
    max_age_sec <= 0.0)
  {
    return false;
  }
  const double age_sec = static_cast<double>(
    now_nanoseconds - captured_stamp_nanoseconds) / 1.0e9;
  return std::isfinite(age_sec) && age_sec <= max_age_sec;
}

// max_age_sec is an online-observation contract.  A snapshot built only from
// immutable, map-hash-bound artifacts remains current until its map generation
// changes; repeatedly re-stamping the same static model would fabricate
// freshness and churn snapshot versions.  Future-dated snapshots always fail.
inline bool terrainSnapshotIsFresh(
  const bool online_inputs_required,
  const std::int64_t now_nanoseconds,
  const std::int64_t snapshot_stamp_nanoseconds,
  const double max_age_sec) noexcept
{
  if (now_nanoseconds < 0 || snapshot_stamp_nanoseconds < 0 ||
    now_nanoseconds < snapshot_stamp_nanoseconds)
  {
    return false;
  }
  if (!online_inputs_required) {
    return true;
  }
  if (!std::isfinite(max_age_sec) || max_age_sec <= 0.0) {
    return false;
  }
  const double age_sec = static_cast<double>(
    now_nanoseconds - snapshot_stamp_nanoseconds) / 1.0e9;
  return std::isfinite(age_sec) && age_sec <= max_age_sec;
}

}  // namespace perception_3d

#endif  // PERCEPTION_3D__TERRAIN_FRESHNESS_H_
