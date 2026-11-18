#pragma once

#include "vapfl_core/adaptive_sampler.hpp"
#include "vapfl_core/dbscan.hpp"
#include "vapfl_core/motion_model.hpp"
#include "vapfl_core/sensor_model.hpp"
#include "vapfl_core/types.hpp"
#include <span>
#include <vector>

namespace vapfl_core {

struct ParticleFilterConfig {
  MotionModelConfig motion{};
  SensorModelConfig sensor{};
  AdaptiveSamplerConfig sampler{};
  DBSCANConfig dbscan{};
  double update_min_d{0.05};     // Minimum translation to trigger filter update (meters)
  double update_min_a{0.05};     // Minimum rotation to trigger filter update (radians)
};

class ParticleFilter {
public:
  explicit ParticleFilter(const ParticleFilterConfig& config = ParticleFilterConfig{});

  /**
   * @brief Set the static map for localization.
   */
  void set_map(MapData map);

  /**
   * @brief Initialize particles around a known pose with Gaussian dispersion.
   * @param mean Center pose (x, y, theta).
   * @param std_dev Standard deviation vector {sigma_x, sigma_y, sigma_theta}.
   * @param num_particles Initial particle count.
   */
  void initialize_gaussian(
      const Pose2D& mean,
      const std::array<double, 3>& std_dev,
      size_t num_particles = 300);

  /**
   * @brief Uniform global initialization across all free cells in the map.
   * @param num_particles Particle population count.
   */
  void initialize_global(size_t num_particles = 800);

  /**
   * @brief Update particle prediction based on odometry reading.
   * @param odom Current odometry pose.
   * @return True if movement exceeded update thresholds and prediction occurred.
   */
  bool process_odometry(const Pose2D& odom);

  /**
   * @brief Process a laser scan: update weights, resample adaptively, and estimate state.
   * @param scan Decoupled 2D laser scan data.
   * @return Current updated FilterState.
   */
  const FilterState& process_laser_scan(const LaserScanData& scan);

  /**
   * @brief Seed the internal random number generators across all subcomponents.
   */
  void set_seed(uint64_t seed);

  [[nodiscard]] bool is_initialized() const noexcept { return is_initialized_; }
  [[nodiscard]] const FilterState& state() const noexcept { return state_; }
  [[nodiscard]] std::span<const Particle> particles() const noexcept { return particles_; }
  [[nodiscard]] const MapData& map() const noexcept { return map_; }
  [[nodiscard]] const ParticleFilterConfig& config() const noexcept { return config_; }

  void set_config(const ParticleFilterConfig& config);

private:
  ParticleFilterConfig config_;
  MapData map_;
  std::vector<Particle> particles_;
  FilterState state_{};

  MotionModel motion_model_;
  SensorModel sensor_model_;
  AdaptiveSampler sampler_;
  DBSCAN clusterer_;

  bool is_initialized_{false};
  bool has_prev_odom_{false};
  Pose2D prev_odom_{};
  Pose2D last_update_odom_{};
  std::mt19937_64 rng_;
};

}  // namespace vapfl_core
