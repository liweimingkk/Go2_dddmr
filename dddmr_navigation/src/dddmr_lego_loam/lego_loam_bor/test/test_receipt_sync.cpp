#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "receipt_sync_utils.h"

namespace
{

using lego_loam_bor::ReceiptSyncSample;

TEST(ReceiptSync, MatchesDifferentRatesWithinThirtyMilliseconds)
{
  std::vector<ReceiptSyncSample> mouth;
  for (int index = -2; index < 20; ++index) {
    const int64_t receipt_ns = 20000000LL + index * 66666667LL;
    mouth.push_back({receipt_ns - 3624000000LL, receipt_ns});
  }

  for (int index = 0; index < 10; ++index) {
    const int64_t receipt_ns = index * 100000000LL;
    const ReceiptSyncSample target{receipt_ns, receipt_ns};
    const auto result = lego_loam_bor::selectClosestReceiptSample(
      mouth, target, 0.03);
    ASSERT_TRUE(result.has_sample);
    EXPECT_TRUE(result.valid);
    EXPECT_LE(result.sync_error_sec, 0.03);
  }
}

TEST(ReceiptSync, RejectsStaleSampleAndRequestsFutureData)
{
  const ReceiptSyncSample target{10000000000LL, 1000000000LL};
  std::vector<ReceiptSyncSample> samples{
    {6200000000LL, 880000000LL}};

  const auto stale = lego_loam_bor::selectClosestReceiptSample(
    samples, target, 0.03);
  ASSERT_TRUE(stale.has_sample);
  EXPECT_FALSE(stale.valid);
  EXPECT_TRUE(stale.needs_future);

  samples.push_back({6400000000LL, 1010000000LL});
  const auto fresh = lego_loam_bor::selectClosestReceiptSample(
    samples, target, 0.03);
  ASSERT_TRUE(fresh.valid);
  EXPECT_EQ(fresh.index, 1U);
  EXPECT_NEAR(fresh.sync_error_sec, 0.01, 1e-12);
}

TEST(ReceiptSync, RejectsLateFutureSampleWithoutWaitingForMore)
{
  const ReceiptSyncSample target{10000000000LL, 1000000000LL};
  const std::vector<ReceiptSyncSample> samples{
    {6500000000LL, 1040000000LL}};

  const auto result = lego_loam_bor::selectClosestReceiptSample(
    samples, target, 0.03);

  ASSERT_TRUE(result.has_sample);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.needs_future);
  EXPECT_NEAR(result.sync_error_sec, 0.04, 1e-12);
}

TEST(ReceiptSync, AcceptsRecentCausalSample)
{
  const ReceiptSyncSample target{10000000000LL, 1000000000LL};
  const std::vector<ReceiptSyncSample> samples{
    {6350000000LL, 980000000LL}};

  const auto result = lego_loam_bor::selectClosestReceiptSample(
    samples, target, 0.03);

  EXPECT_TRUE(result.valid);
  EXPECT_FALSE(result.needs_future);
  EXPECT_EQ(result.signed_receipt_delta_ns, -20000000LL);
}

TEST(ReceiptSync, IgnoresHeaderEpochJumpWhenReceiptsStayMonotonic)
{
  const ReceiptSyncSample target{200000000000LL, 5000000000LL};
  const std::vector<ReceiptSyncSample> samples{
    {180500000000LL, 4900000000LL},
    {196440000000LL, 5005000000LL}};

  const auto result = lego_loam_bor::selectClosestReceiptSample(
    samples, target, 0.03);

  ASSERT_TRUE(result.valid);
  EXPECT_EQ(result.index, 1U);
  EXPECT_EQ(result.signed_receipt_delta_ns, 5000000LL);
  EXPECT_NEAR(
    static_cast<double>(result.inferred_time_offset_ns) * 1e-9,
    3.565, 1e-12);
}

TEST(ReceiptSync, ShiftsSyntheticTransformStampByReceiptPhase)
{
  EXPECT_EQ(
    lego_loam_bor::shiftHeaderByReceiptDelta(
      10000000000LL, -17000000LL),
    9983000000LL);
}

}  // namespace
