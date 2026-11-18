#include <gtest/gtest.h>
#include "vapfl_core/motion_model.hpp"
#include <cmath>
#include <numeric>
#include <vector>

using namespace vapfl_core;

TEST(MotionModelTest, ZeroMotionProducesMinimalDrift) {
  MotionModel model;
  model.set_seed(42);

  std::vector<Particle> particles(100, Particle{0.0, 0.0, 0.0, 0.01});
  const Pose2D odom1{1.0, 2.0, 0.5};
  const Pose2D odom2{1.0, 2.0, 0.5};

  model.predict(particles, odom1, odom2);

  for (const auto& p : particles) {
    EXPECT_NEAR(p.x, 0.0, 1e-4);
    EXPECT_NEAR(p.y, 0.0, 1e-4);
    EXPECT_NEAR(p.theta, 0.0, 1e-4);
  }
}

TEST(MotionModelTest, PureForwardTranslationDispersal) {
  MotionModelConfig cfg;
  cfg.alpha1 = 0.01;
  cfg.alpha2 = 0.01;
  cfg.alpha3 = 0.05;
  cfg.alpha4 = 0.01;

  MotionModel model(cfg);
  model.set_seed(12345);

  constexpr size_t num_particles = 2000;
  std::vector<Particle> particles(num_particles, Particle{0.0, 0.0, 0.0, 1.0 / static_cast<double>(num_particles)});

  const Pose2D prev_odom{0.0, 0.0, 0.0};
  const Pose2D curr_odom{2.0, 0.0, 0.0};  // 2.0m forward along X

  model.predict(particles, prev_odom, curr_odom);

  double sum_x = 0.0;
  double sum_y = 0.0;
  for (const auto& p : particles) {
    sum_x += p.x;
    sum_y += p.y;
  }
  const double mean_x = sum_x / static_cast<double>(num_particles);
  const double mean_y = sum_y / static_cast<double>(num_particles);

  // Mean should be centered close to commanded 2.0m forward
  EXPECT_NEAR(mean_x, 2.0, 0.10);
  EXPECT_NEAR(mean_y, 0.0, 0.10);

  // Check dispersal variance is non-zero
  double var_x = 0.0;
  for (const auto& p : particles) {
    const double dx = p.x - mean_x;
    var_x += dx * dx;
  }
  var_x /= static_cast<double>(num_particles);
  EXPECT_GT(var_x, 0.001);
}

TEST(MotionModelTest, PureRotationDispersal) {
  MotionModel model;
  model.set_seed(999);

  constexpr size_t num_particles = 1000;
  std::vector<Particle> particles(num_particles, Particle{0.0, 0.0, 0.0, 1.0 / static_cast<double>(num_particles)});

  const Pose2D prev_odom{0.0, 0.0, 0.0};
  const Pose2D curr_odom{0.0, 0.0, 0.785};  // ~pi/4 rad rotation in place

  model.predict(particles, prev_odom, curr_odom);

  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const auto& p : particles) {
    sin_sum += std::sin(p.theta);
    cos_sum += std::cos(p.theta);
  }
  const double mean_th = std::atan2(sin_sum, cos_sum);

  EXPECT_NEAR(mean_th, 0.785, 0.05);
}
