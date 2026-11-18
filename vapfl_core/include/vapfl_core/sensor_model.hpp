#pragma once

#include "vapfl_core/types.hpp"
#include <memory>
#include <span>
#include <vector>

namespace vapfl_core {

class SensorModel {
public:
  explicit SensorModel(const SensorModelConfig& config = SensorModelConfig{});

  /**
   * @brief Fast 2D DDA grid raycaster calculating distance from start pose to nearest occupied cell.
   * @param map The occupancy grid map.
   * @param start_x Ray origin X in world coordinates.
   * @param start_y Ray origin Y in world coordinates.
   * @param angle Ray direction in world coordinates (radians).
   * @param max_range Maximum raycast distance in meters.
   * @return Expected range to nearest obstacle, or max_range if unobstructed.
   */
  [[nodiscard]] static double raycast(
      const MapData& map,
      double start_x,
      double start_y,
      double angle,
      double max_range) noexcept;

  /**
   * @brief Calculate measurement likelihood p(scan | particle, map).
   * @param particle Candidate particle pose.
   * @param scan Decoupled 2D laser scan data.
   * @param map The occupancy grid map.
   * @return Likelihood score.
   */
  [[nodiscard]] double compute_likelihood(
      const Particle& particle,
      const LaserScanData& scan,
      const MapData& map) const noexcept;

  /**
   * @brief Update weights across all particles based on scan measurements.
   * @param particles Particles to update.
   * @param scan 2D laser scan data.
   * @param map The occupancy grid map.
   */
  void update_weights(
      std::span<Particle> particles,
      const LaserScanData& scan,
      const MapData& map) const;

  [[nodiscard]] const SensorModelConfig& config() const noexcept { return config_; }
  void set_config(const SensorModelConfig& config) noexcept { config_ = config; }

private:
  SensorModelConfig config_;
};

}  // namespace vapfl_core
