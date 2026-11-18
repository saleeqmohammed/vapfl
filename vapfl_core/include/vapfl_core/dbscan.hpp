#pragma once

#include "vapfl_core/types.hpp"
#include <array>
#include <span>
#include <vector>

namespace vapfl_core {

struct ClusterResult {
  std::vector<int> labels;        // -1 for noise, >= 0 for cluster ID
  int dominant_cluster_id{-1};    // Cluster with highest cumulative weight
  size_t dominant_cluster_size{0};
  double dominant_cluster_weight{0.0};
  Pose2D estimated_pose{};
  std::array<double, 9> covariance{0.0};
};

class DBSCAN {
public:
  explicit DBSCAN(const DBSCANConfig& config = DBSCANConfig{});

  /**
   * @brief Cluster particles in SE(2) and extract dominant hypothesis pose and covariance.
   * @param particles Particles to cluster.
   * @return ClusterResult containing labels, dominant cluster, and state estimate.
   */
  [[nodiscard]] ClusterResult cluster(std::span<const Particle> particles) const;

  [[nodiscard]] const DBSCANConfig& config() const noexcept { return config_; }
  void set_config(const DBSCANConfig& config) noexcept { config_ = config; }

private:
  [[nodiscard]] bool are_neighbors(const Particle& p1, const Particle& p2) const noexcept;
  [[nodiscard]] std::vector<size_t> find_neighbors(
      std::span<const Particle> particles,
      size_t index) const;

  DBSCANConfig config_;
};

}  // namespace vapfl_core
