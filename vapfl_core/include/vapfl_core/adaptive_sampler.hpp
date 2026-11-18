#pragma once

#include "vapfl_core/types.hpp"
#include <random>
#include <span>
#include <vector>

namespace vapfl_core {

class AdaptiveSampler {
public:
  explicit AdaptiveSampler(const AdaptiveSamplerConfig& config = AdaptiveSamplerConfig{});

  /**
   * @brief Normalize particle weights in-place and compute weight variance and Neff.
   * @param particles Particles to normalize.
   * @param weight_variance Output parameter: variance of normalized weights.
   * @param effective_sample_size Output parameter: Neff = 1 / sum(w_i^2).
   * @param avg_likelihood Output parameter: mean unnormalized weight.
   */
  void normalize_and_analyze(
      std::span<Particle> particles,
      double& weight_variance,
      double& effective_sample_size,
      double& avg_likelihood) const noexcept;

  /**
   * @brief Calculate target particle count dynamically based on variance.
   * @param weight_variance Current weight variance.
   * @return Target particle count in [min_particles, max_particles].
   */
  [[nodiscard]] size_t compute_target_particles(double weight_variance) const noexcept;

  /**
   * @brief Check if kidnap condition is met based on fast/slow likelihood averages.
   * @param avg_likelihood Mean likelihood of current observation.
   * @return True if kidnap/recovery doping should trigger.
   */
  bool update_kidnap_detector(double avg_likelihood) noexcept;

  /**
   * @brief Perform low-variance resampling with dynamic particle sizing and free-space doping.
   * @param current_particles Current particle population.
   * @param target_count Desired output particle count.
   * @param do_doping Whether kidnap recovery doping should be injected.
   * @param map Occupancy map for valid free-space particle placement.
   * @return New vector of resampled particles.
   */
  [[nodiscard]] std::vector<Particle> resample(
      std::span<const Particle> current_particles,
      size_t target_count,
      bool do_doping,
      const MapData& map);

  /**
   * @brief Sample a random valid particle in free space.
   */
  [[nodiscard]] Particle sample_free_space(const MapData& map);

  /**
   * @brief Seed the internal random number generator.
   */
  void set_seed(uint64_t seed);

  [[nodiscard]] const AdaptiveSamplerConfig& config() const noexcept { return config_; }
  void set_config(const AdaptiveSamplerConfig& config) noexcept { config_ = config; }

  [[nodiscard]] double w_slow() const noexcept { return w_slow_; }
  [[nodiscard]] double w_fast() const noexcept { return w_fast_; }

  void reset_averages() noexcept {
    w_slow_ = 0.0;
    w_fast_ = 0.0;
  }

private:
  AdaptiveSamplerConfig config_;
  double w_slow_{0.0};
  double w_fast_{0.0};
  std::mt19937_64 rng_;
};

}  // namespace vapfl_core
