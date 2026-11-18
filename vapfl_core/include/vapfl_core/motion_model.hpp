#pragma once

#include "vapfl_core/types.hpp"
#include <random>
#include <span>
#include <vector>

namespace vapfl_core {

class MotionModel {
public:
  explicit MotionModel(const MotionModelConfig& config = MotionModelConfig{});

  /**
   * @brief Update particle poses based on odometric change.
   * @param particles Span of particles to update in-place.
   * @param prev_odom Previous odometric pose.
   * @param curr_odom Current odometric pose.
   */
  void predict(std::span<Particle> particles, const Pose2D& prev_odom, const Pose2D& curr_odom);

  /**
   * @brief Seed the internal random number generator (e.g. for deterministic unit testing).
   */
  void set_seed(uint64_t seed);

  /**
   * @brief Get current configuration.
   */
  [[nodiscard]] const MotionModelConfig& config() const noexcept { return config_; }

  /**
   * @brief Set configuration.
   */
  void set_config(const MotionModelConfig& config) noexcept { config_ = config; }

private:
  MotionModelConfig config_;
  std::mt19937_64 rng_;
};

}  // namespace vapfl_core
