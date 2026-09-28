#include <gtest/gtest.h>

#include <ctime>

#include "camera_recorder/bag_uri.hpp"

using camera_recorder::bag_uri;

namespace
{
std::tm at(int year, int month, int day, int hour, int minute, int second)
{
  std::tm t{};
  t.tm_year = year - 1900;
  t.tm_mon = month - 1;
  t.tm_mday = day;
  t.tm_hour = hour;
  t.tm_min = minute;
  t.tm_sec = second;
  return t;
}
}  // namespace

TEST(BagUri, AppendsZeroPaddedTimestampedName)
{
  EXPECT_EQ(bag_uri("/data/bags", "/home/u", at(2026, 3, 4, 5, 6, 7)),
    "/data/bags/camera_20260304_050607");
}

TEST(BagUri, ExpandsLeadingTildeToHome)
{
  EXPECT_EQ(bag_uri("~/bags", "/home/u", at(2026, 12, 31, 23, 59, 58)),
    "/home/u/bags/camera_20261231_235958");
  EXPECT_EQ(bag_uri("~", "/home/u", at(2026, 1, 1, 0, 0, 0)), "/home/u/camera_20260101_000000");
}

TEST(BagUri, LeavesTildeElsewhereAlone)
{
  EXPECT_EQ(bag_uri("/data/~x", "/home/u", at(2026, 1, 1, 0, 0, 0)),
    "/data/~x/camera_20260101_000000");
}

TEST(BagUri, ToleratesTrailingSlash)
{
  EXPECT_EQ(bag_uri("/data/bags/", "/home/u", at(2026, 1, 1, 0, 0, 0)),
    "/data/bags/camera_20260101_000000");
}
