#include <gtest/gtest.h>

#include <vimbax_camera/stamp_pairer.hpp>

using ::vimbax_camera::StampPairer;

constexpr int64_t kMs = 1000000;
constexpr int64_t kPeriod = 33 * kMs;

TEST(stamp_pairer, perfect_pairs)
{
  StampPairer pairer{16 * kMs};
  for (int64_t i = 0; i < 10; ++i) {
    pairer.add_rgb(i * kPeriod);
    pairer.add_ir(i * kPeriod);
  }

  auto const window = pairer.take_window();
  EXPECT_EQ(window.rgb, 10u);
  EXPECT_EQ(window.ir, 10u);
  EXPECT_EQ(window.unmatched_rgb, 0u);
  EXPECT_EQ(window.unmatched_ir, 0u);
  EXPECT_EQ(window.rgb_minus_ir_ns, std::vector<int64_t>(10, 0));
}

TEST(stamp_pairer, constant_offset)
{
  StampPairer pairer{16 * kMs};
  for (int64_t i = 0; i < 10; ++i) {
    pairer.add_rgb(i * kPeriod + 2 * kMs);
    pairer.add_ir(i * kPeriod);
  }

  auto const window = pairer.take_window();
  EXPECT_EQ(window.unmatched_rgb, 0u);
  EXPECT_EQ(window.unmatched_ir, 0u);
  EXPECT_EQ(window.rgb_minus_ir_ns, std::vector<int64_t>(10, 2 * kMs));
}

TEST(stamp_pairer, dropped_frame_on_each_side)
{
  StampPairer pairer{16 * kMs};
  for (int64_t i = 0; i < 10; ++i) {
    if (i != 3) {
      pairer.add_rgb(i * kPeriod);
    }
    if (i != 6) {
      pairer.add_ir(i * kPeriod);
    }
  }

  auto const window = pairer.take_window();
  EXPECT_EQ(window.rgb, 9u);
  EXPECT_EQ(window.ir, 9u);
  EXPECT_EQ(window.unmatched_rgb, 1u);
  EXPECT_EQ(window.unmatched_ir, 1u);
  EXPECT_EQ(window.rgb_minus_ir_ns.size(), 8u);
}

TEST(stamp_pairer, late_arrival_from_one_camera_still_pairs)
{
  StampPairer pairer{16 * kMs};
  for (int64_t i = 0; i < 5; ++i) {
    pairer.add_rgb(i * kPeriod);
  }
  for (int64_t i = 0; i < 5; ++i) {
    pairer.add_ir(i * kPeriod + kMs);
  }

  auto const window = pairer.take_window();
  EXPECT_EQ(window.unmatched_rgb, 0u);
  EXPECT_EQ(window.unmatched_ir, 0u);
  EXPECT_EQ(window.rgb_minus_ir_ns, std::vector<int64_t>(5, -kMs));
}

TEST(stamp_pairer, offset_exceeding_max_is_unmatched)
{
  StampPairer pairer{5 * kMs};
  for (int64_t i = 0; i < 5; ++i) {
    pairer.add_rgb(i * kPeriod);
    pairer.add_ir(i * kPeriod + 6 * kMs);
  }

  auto const window = pairer.take_window();
  EXPECT_TRUE(window.rgb_minus_ir_ns.empty());
  EXPECT_EQ(window.unmatched_rgb, 5u);
  EXPECT_EQ(window.unmatched_ir, 4u);
}

TEST(stamp_pairer, take_window_resets_counts)
{
  StampPairer pairer{16 * kMs};
  pairer.add_rgb(0);
  pairer.add_ir(0);
  pairer.take_window();

  auto const window = pairer.take_window();
  EXPECT_EQ(window.rgb, 0u);
  EXPECT_EQ(window.ir, 0u);
  EXPECT_TRUE(window.rgb_minus_ir_ns.empty());
}
