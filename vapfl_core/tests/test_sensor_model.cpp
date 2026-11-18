#include <gtest/gtest.h>
#include "vapfl_core/sensor_model.hpp"
#include <cmath>

using namespace vapfl_core;

namespace {

MapData create_square_room(double size = 10.0, double resolution = 0.1) {
  MapData map;
  map.resolution = resolution;
  map.width = static_cast<uint32_t>(size / resolution);
  map.height = static_cast<uint32_t>(size / resolution);
  map.origin = {0.0, 0.0, 0.0};
  map.data.assign(static_cast<size_t>(map.width) * static_cast<size_t>(map.height), 0);

  // Mark borders as occupied (100)
  for (uint32_t x = 0; x < map.width; ++x) {
    map.data[x] = 100;                                           // Bottom wall
    map.data[(map.height - 1) * map.width + x] = 100;            // Top wall
  }
  for (uint32_t y = 0; y < map.height; ++y) {
    map.data[y * map.width] = 100;                               // Left wall
    map.data[y * map.width + (map.width - 1)] = 100;             // Right wall
  }
  return map;
}

}  // namespace

TEST(SensorModelTest, RaycastHitsWallsAccurately) {
  const MapData map = create_square_room(10.0, 0.1);

  // Ray from center (5.0, 5.0) pointing East (+X, angle 0)
  // Distance from 5.0 to 9.9 wall is approx 4.9m
  const double range_east = SensorModel::raycast(map, 5.0, 5.0, 0.0, 20.0);
  EXPECT_NEAR(range_east, 4.9, 0.15);

  // Ray from center (5.0, 5.0) pointing North (+Y, angle pi/2)
  const double range_north = SensorModel::raycast(map, 5.0, 5.0, std::numbers::pi / 2.0, 20.0);
  EXPECT_NEAR(range_north, 4.9, 0.15);

  // Ray pointing West (-X, angle pi)
  const double range_west = SensorModel::raycast(map, 5.0, 5.0, std::numbers::pi, 20.0);
  EXPECT_NEAR(range_west, 4.9, 0.15);

  // Ray pointing South (-Y, angle -pi/2)
  const double range_south = SensorModel::raycast(map, 5.0, 5.0, -std::numbers::pi / 2.0, 20.0);
  EXPECT_NEAR(range_south, 4.9, 0.15);
}

TEST(SensorModelTest, RaycastExceedingMaxRangeReturnsMax) {
  const MapData map = create_square_room(10.0, 0.1);
  // Max range smaller than wall distance
  const double range = SensorModel::raycast(map, 5.0, 5.0, 0.0, 2.0);
  EXPECT_NEAR(range, 2.0, 1e-4);
}

TEST(SensorModelTest, LikelihoodIsMaximizedAtTruePose) {
  const MapData map = create_square_room(10.0, 0.1);
  SensorModel model;

  // Generate synthetic scan from true pose (5.0, 5.0, 0.0)
  LaserScanData scan;
  scan.angle_min = -std::numbers::pi;
  scan.angle_max = std::numbers::pi;
  scan.angle_increment = std::numbers::pi / 18.0;  // 36 beams
  scan.range_min = 0.1;
  scan.range_max = 15.0;

  for (double a = scan.angle_min; a <= scan.angle_max; a += scan.angle_increment) {
    const double r = SensorModel::raycast(map, 5.0, 5.0, a, scan.range_max);
    scan.ranges.push_back(r);
  }

  // True pose particle
  const Particle true_p{5.0, 5.0, 0.0, 1.0};
  const double true_lik = model.compute_likelihood(true_p, scan, map);

  // Displaced pose particle
  const Particle offset_p{6.5, 5.0, 0.0, 1.0};
  const double offset_lik = model.compute_likelihood(offset_p, scan, map);

  // Inside-wall particle
  const Particle wall_p{0.05, 5.0, 0.0, 1.0};
  const double wall_lik = model.compute_likelihood(wall_p, scan, map);

  EXPECT_GT(true_lik, offset_lik);
  EXPECT_GT(offset_lik, wall_lik);
  EXPECT_LT(wall_lik, 1e-6);
}
