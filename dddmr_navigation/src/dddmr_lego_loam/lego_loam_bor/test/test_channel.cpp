#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include "channel.h"

TEST(Channel, TryReceiveDoesNotBlockWhenEmpty)
{
  Channel<int> channel(false);
  int value = -1;

  const auto started = std::chrono::steady_clock::now();
  EXPECT_FALSE(channel.try_receive(value));
  const auto elapsed = std::chrono::steady_clock::now() - started;

  EXPECT_LT(elapsed, std::chrono::milliseconds(50));
  EXPECT_EQ(value, -1);
}

TEST(Channel, NonBlockingSenderKeepsLatestItem)
{
  Channel<int> channel(false);
  channel.send(1);
  channel.send(2);

  int value = 0;
  ASSERT_TRUE(channel.try_receive(value));
  EXPECT_EQ(value, 2);
  EXPECT_FALSE(channel.try_receive(value));
}

TEST(Channel, BlockingSenderIsReleasedByTryReceive)
{
  Channel<int> channel(true);
  channel.send(1);
  auto sender = std::async(std::launch::async, [&channel]() {
    channel.send(2);
  });

  EXPECT_EQ(sender.wait_for(std::chrono::milliseconds(20)), std::future_status::timeout);
  int value = 0;
  ASSERT_TRUE(channel.try_receive(value));
  EXPECT_EQ(value, 1);
  EXPECT_EQ(sender.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  ASSERT_TRUE(channel.try_receive(value));
  EXPECT_EQ(value, 2);
}
