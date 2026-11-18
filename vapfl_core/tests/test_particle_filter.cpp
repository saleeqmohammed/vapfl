#include <gtest/gtest.h>
#include "vapfl_core/particle_filter.hpp"
#include <cmath>

using namespace vapfl_core;

namespace {

MapData create_test_room(double size = 10.0, double resolution = 0.1) {
  MapData map;
  map.resolution = resolution;
  map.width = static_cast<uint32_t>(size / resolution);
  map.height = static_cast<uint32_t>(size / resolution);
  map.origin = {0.0, 0.0, 0.0};
  map.data.assign(static_cast<size_t>(map.width) * static_cast<size_t>(map.height), 0);

  for (uint32_t x = 0; x < map.width; ++x) {
    map.data[x] = 100;
    map.data[(map.height - 1) * map.width + x] = 100;
  }
  for (uint32_t y = 0; y < map.height; ++y) {
    map.data[y * map.width] = 100;
    map.data[y * map.width + (map.width - 1)] = 100;
  }
  return map;
}

LaserScanData generate_scan(const MapData& map, const Pose2D& pose) {
  LaserScanData scan;
  scan.angle_min = -std::numbers::pi;
  scan.angle_max = std::numbers::pi;
  scan.angle_increment = std::numbers::pi / 18.0;  // 36 beams
  scan.range_min = 0.05;
  scan.range_max = 15.0;

  for (double a = scan.angle_min; a <= scan.angle_max; a += scan.angle_increment) {
    const double world_beam_angle = normalize_angle(pose.theta + a);
    const double r = SensorModel::raycast(map, pose.x, pose.y, world_beam_angle, scan.range_max);
    scan.ranges.push_back(r);
  }
  return scan;
}

}  // namespace

TEST(ParticleFilterTest, EndToEndLocalizationConvergence) {
  const MapData map = create_test_room(10.0, 0.1);

  ParticleFilterConfig cfg;
  cfg.sampler.min_particles = 150;
  cfg.sampler.max_particles = 400;
  cfg.update_min_d = 0.05;
  cfg.update_min_a = 0.05;

  ParticleFilter pf(cfg);
  pf.set_seed(777);
  pf.set_map(map);

  // Initialize with initial uncertainty around true start (3.0, 3.0, 0.0)
  const Pose2D start_pose{3.0, 3.0, 0.0};
  const std::array<double, 3> initial_std{0.25, 0.25, 0.10};
  pf.initialize_gaussian(start_pose, initial_std, 200);

  EXPECT_TRUE(pf.is_initialized());
  EXPECT_EQ(pf.particles().size(), 200);

  // Initial scan update at t=0
  const LaserScanData scan0 = generate_scan(map, start_pose);
  const FilterState s0 = pf.process_laser_scan(scan0);
  EXPECT_NEAR(s0.pose.x, 3.0, 0.20);
  EXPECT_NEAR(s0.pose.y, 3.0, 0.20);

  // Simulate robot trajectory: step forward along X by 0.2m each step
  Pose2D ground_truth = start_pose;
  Pose2D odom = start_pose;

  for (int step = 1; step <= 8; ++step) {
    ground_truth.x += 0.2;
    odom.x += 0.2;

    pf.process_odometry(odom);
    const LaserScanData scan = generate_scan(map, ground_truth);
    const FilterState s = pf.process_laser_scan(scan);

    // After several updates, filter should track ground truth closely
    if (step >= 5) {
      EXPECT_NEAR(s.pose.x, ground_truth.x, 0.15);
      EXPECT_NEAR(s.pose.y, ground_truth.y, 0.15);
      EXPECT_NEAR(s.pose.theta, ground_truth.theta, 0.15);
    }
  }

  // Final check
  const FilterState final_state = pf.state();
  EXPECT_NEAR(final_state.pose.x, ground_truth.x, 0.10);
  EXPECT_NEAR(final_state.pose.y, ground_truth.y, 0.10);
}
