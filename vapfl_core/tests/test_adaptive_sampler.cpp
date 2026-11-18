#include <gtest/gtest.h>
#include "vapfl_core/adaptive_sampler.hpp"
#include <cmath>
#include <numeric>
#include <vector>

using namespace vapfl_core;

TEST(AdaptiveSamplerTest, NormalizationAndStatistics) {
  AdaptiveSampler sampler;

  std::vector<Particle> particles = {
      {0.0, 0.0, 0.0, 2.0},
      {1.0, 1.0, 0.0, 2.0},
      {2.0, 2.0, 0.0, 2.0},
      {3.0, 3.0, 0.0, 2.0}};

  double var = 0.0;
  double neff = 0.0;
  double avg_lik = 0.0;

  sampler.normalize_and_analyze(particles, var, neff, avg_lik);

  EXPECT_NEAR(particles[0].weight, 0.25, 1e-6);
  EXPECT_NEAR(particles[3].weight, 0.25, 1e-6);
  EXPECT_NEAR(var, 0.0, 1e-9);
  EXPECT_NEAR(neff, 4.0, 1e-6);
  EXPECT_NEAR(avg_lik, 2.0, 1e-6);
}

TEST(AdaptiveSamplerTest, DynamicParticleCountScaling) {
  AdaptiveSamplerConfig cfg;
  cfg.min_particles = 100;
  cfg.max_particles = 800;
  cfg.target_variance = 0.01;

  AdaptiveSampler sampler(cfg);

  // Very low variance should clamp to min_particles
  EXPECT_EQ(sampler.compute_target_particles(0.0), 100);

  // High variance should clamp to max_particles
  EXPECT_EQ(sampler.compute_target_particles(0.05), 800);

  // Intermediate variance
  const size_t mid = sampler.compute_target_particles(0.005);
  EXPECT_GT(mid, 100);
  EXPECT_LT(mid, 800);
}

TEST(AdaptiveSamplerTest, ResamplingMaintainsDominantParticles) {
  AdaptiveSampler sampler;
  sampler.set_seed(42);

  // Particle 0 has 99% of total weight
  std::vector<Particle> particles = {
      {10.0, 10.0, 0.0, 0.99},
      {0.0, 0.0, 0.0, 0.005},
      {1.0, 1.0, 0.0, 0.005}};

  MapData empty_map;
  std::vector<Particle> resampled = sampler.resample(particles, 100, false, empty_map);

  EXPECT_EQ(resampled.size(), 100);
  size_t count_dominant = 0;
  for (const auto& p : resampled) {
    if (std::abs(p.x - 10.0) < 1e-4) {
      ++count_dominant;
    }
  }
  // At least 95 out of 100 should be the dominant particle
  EXPECT_GE(count_dominant, 95);
}

TEST(AdaptiveSamplerTest, KidnapDetectorTriggersOnLikelihoodPlunge) {
  AdaptiveSamplerConfig cfg;
  cfg.doping_threshold = 0.05;
  cfg.recovery_alpha_slow = 0.01;
  cfg.recovery_alpha_fast = 0.50;  // Fast filter drops quickly

  AdaptiveSampler sampler(cfg);

  // Normal converged state (high likelihood = 1.0)
  for (int i = 0; i < 20; ++i) {
    sampler.update_kidnap_detector(1.0);
  }

  // Robot gets kidnapped: observation likelihood plunges to 0.001
  bool triggered = false;
  for (int i = 0; i < 5; ++i) {
    if (sampler.update_kidnap_detector(0.001)) {
      triggered = true;
      break;
    }
  }

  EXPECT_TRUE(triggered);
}
