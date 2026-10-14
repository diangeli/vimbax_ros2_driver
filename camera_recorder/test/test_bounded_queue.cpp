#include <gtest/gtest.h>

#include <chrono>
#include <future>

#include "camera_recorder/bounded_queue.hpp"

using camera_recorder::BoundedQueue;
using namespace std::chrono_literals;

TEST(BoundedQueue, RejectsNewItemsWhenFull)
{
  BoundedQueue<int> queue(2);

  EXPECT_TRUE(queue.try_push(1));
  EXPECT_TRUE(queue.try_push(2));
  EXPECT_FALSE(queue.try_push(3));
  EXPECT_EQ(queue.size(), 2u);
  EXPECT_EQ(queue.pop(), 1);
  EXPECT_TRUE(queue.try_push(4));
  EXPECT_EQ(queue.pop(), 2);
  EXPECT_EQ(queue.pop(), 4);
}

TEST(BoundedQueue, PopBlocksUntilAnItemIsPushed)
{
  BoundedQueue<int> queue(1);
  auto popped = std::async(std::launch::async, [&] {return queue.pop();});

  EXPECT_EQ(popped.wait_for(100ms), std::future_status::timeout);
  queue.try_push(7);

  ASSERT_EQ(popped.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(popped.get(), 7);
}

TEST(BoundedQueue, CloseDrainsRemainingItemsThenEnds)
{
  BoundedQueue<int> queue(3);
  queue.try_push(1);
  queue.try_push(2);

  queue.close();

  EXPECT_FALSE(queue.try_push(3));
  EXPECT_EQ(queue.pop(), 1);
  EXPECT_EQ(queue.pop(), 2);
  EXPECT_EQ(queue.pop(), std::nullopt);
}

TEST(BoundedQueue, CloseUnblocksAWaitingPop)
{
  BoundedQueue<int> queue(1);
  auto popped = std::async(std::launch::async, [&] {return queue.pop();});

  EXPECT_EQ(popped.wait_for(100ms), std::future_status::timeout);
  queue.close();

  ASSERT_EQ(popped.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(popped.get(), std::nullopt);
}
