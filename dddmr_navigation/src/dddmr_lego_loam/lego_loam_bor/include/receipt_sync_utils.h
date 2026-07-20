#ifndef LEGO_LOAM_BOR_RECEIPT_SYNC_UTILS_H_
#define LEGO_LOAM_BOR_RECEIPT_SYNC_UTILS_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace lego_loam_bor
{

struct ReceiptSyncSample
{
  int64_t header_stamp_ns = 0;
  int64_t receipt_stamp_ns = 0;
};

struct ReceiptSyncSelection
{
  bool has_sample = false;
  bool valid = false;
  bool needs_future = false;
  std::size_t index = 0U;
  int64_t signed_receipt_delta_ns = 0;
  int64_t inferred_time_offset_ns = 0;
  double sync_error_sec = std::numeric_limits<double>::infinity();
};

inline ReceiptSyncSelection selectClosestReceiptSample(
  const std::vector<ReceiptSyncSample> & samples,
  const ReceiptSyncSample & target,
  const double tolerance_sec)
{
  ReceiptSyncSelection result;
  int64_t best_abs_delta_ns = std::numeric_limits<int64_t>::max();

  for (std::size_t index = 0U; index < samples.size(); ++index) {
    const int64_t signed_delta_ns =
      samples[index].receipt_stamp_ns - target.receipt_stamp_ns;
    const int64_t abs_delta_ns = std::abs(signed_delta_ns);
    if (abs_delta_ns >= best_abs_delta_ns) {
      continue;
    }

    result.has_sample = true;
    result.index = index;
    result.signed_receipt_delta_ns = signed_delta_ns;
    // Remove the signed sampling phase from the raw header difference. This
    // is diagnostic only; live receipt-time synchronization does not depend
    // on either sensor's clock epoch.
    result.inferred_time_offset_ns =
      target.header_stamp_ns - samples[index].header_stamp_ns + signed_delta_ns;
    best_abs_delta_ns = abs_delta_ns;
  }

  if (!result.has_sample) {
    result.needs_future = true;
    return result;
  }

  result.sync_error_sec = static_cast<double>(best_abs_delta_ns) * 1e-9;
  result.valid = tolerance_sec <= 0.0 || result.sync_error_sec <= tolerance_sec;
  result.needs_future = !result.valid && result.signed_receipt_delta_ns < 0;
  return result;
}

inline int64_t shiftHeaderByReceiptDelta(
  const int64_t main_header_stamp_ns,
  const int64_t signed_receipt_delta_ns)
{
  return main_header_stamp_ns + signed_receipt_delta_ns;
}

}  // namespace lego_loam_bor

#endif  // LEGO_LOAM_BOR_RECEIPT_SYNC_UTILS_H_
