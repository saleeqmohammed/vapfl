#include <gtest/gtest.h>
#include "vapfl_core/dbscan.hpp"
#include <cmath>
#include <random>
#include <vector>

using namespace vapfl_core;

TEST(DBSCANTest, ClustersBimodalDistributionAndRejectsNoise) {
  DBSCANConfig cfg;
  cfg.eps = 0.5;
  cfg.eps_theta = 0.5;
  cfg.min_pts = 5;

  DBSCAN clusterer(cfg);

  std::vector<Particle> particles;
  std::mt19937_64 rng(42);
  std::normal_distribution<double> noise(0.0, 0.05);

  // Cluster A: 50 particles around (2.0, 3.0, 0.0), each weight 0.015 (sum = 0.75)
  for (size_t i = 0; i < 50; ++i) {
    particles.push_back({2.0 + noise(rng), 3.0 + noise(rng), 0.0 + noise(rng), 0.015});
  }

  // Cluster B: 30 particles around (8.0, 8.0, 1.57), each weight 0.005 (sum = 0.15)
  for (size_t i = 0; i < 30; ++i) {
    particles.push_back({8.0 + noise(rng), 8.0 + noise(rng), 1.57 + noise(rng), 0.005});
  }

  // Outliers: 5 scattered points
  particles.push_back({-10.0, 0.0, 0.0, 0.001});
  particles.push_back({20.0, 20.0, 0.0, 0.001});
  particles.push_back({0.0, 50.0, 0.0, 0.001});

  const ClusterResult res = clusterer.cluster(particles);

  // Check dominant cluster is found
  EXPECT_GE(res.dominant_cluster_id, 0);
  EXPECT_EQ(res.dominant_cluster_size, 50);

  // Dominant pose should match Cluster A center
  EXPECT_NEAR(res.estimated_pose.x, 2.0, 0.05);
  EXPECT_NEAR(res.estimated_pose.y, 3.0, 0.05);
  EXPECT_NEAR(res.estimated_pose.theta, 0.0, 0.05);

  // Check outliers are labeled as -1 (noise)
  EXPECT_EQ(res.labels[80], -1);
  EXPECT_EQ(res.labels[81], -1);
  EXPECT_EQ(res.labels[82], -1);

  // Covariance diagonal should be positive and small
  EXPECT_GT(res.covariance[0], 0.0);
  EXPECT_LT(res.covariance[0], 0.05);
  EXPECT_GT(res.covariance[4], 0.0);
  EXPECT_LT(res.covariance[4], 0.05);
}
