#include "vapfl_core/adaptive_sampler.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace vapfl_core {

AdaptiveSampler::AdaptiveSampler(const AdaptiveSamplerConfig& config)
    : config_(config), rng_(std::random_device{}()) {}

void AdaptiveSampler::set_seed(uint64_t seed) {
  rng_.seed(seed);
}

void AdaptiveSampler::normalize_and_analyze(
    std::span<Particle> particles,
    double& weight_variance,
    double& effective_sample_size,
    double& avg_likelihood) const noexcept {
  const size_t n = particles.size();
  if (n == 0) {
    weight_variance = 0.0;
    effective_sample_size = 0.0;
    avg_likelihood = 0.0;
    return;
  }

  double sum_weights = 0.0;
  for (const Particle& p : particles) {
    if (std::isfinite(p.weight) && p.weight > 0.0) {
      sum_weights += p.weight;
    }
  }

  avg_likelihood = sum_weights / static_cast<double>(n);

  if (sum_weights <= 0.0) {
    const double uniform_w = 1.0 / static_cast<double>(n);
    for (Particle& p : particles) {
      p.weight = uniform_w;
    }
    weight_variance = 0.0;
    effective_sample_size = static_cast<double>(n);
    return;
  }

  const double inv_sum = 1.0 / sum_weights;
  const double mean_w = 1.0 / static_cast<double>(n);
  double sum_sq_diff = 0.0;
  double sum_w_sq = 0.0;

  for (Particle& p : particles) {
    p.weight *= inv_sum;
    const double diff = p.weight - mean_w;
    sum_sq_diff += diff * diff;
    sum_w_sq += p.weight * p.weight;
  }

  weight_variance = sum_sq_diff / static_cast<double>(n);
  effective_sample_size = (sum_w_sq > 0.0) ? (1.0 / sum_w_sq) : static_cast<double>(n);
}

size_t AdaptiveSampler::compute_target_particles(double weight_variance) const noexcept {
  if (config_.target_variance <= 0.0 || config_.max_particles <= config_.min_particles) {
    return config_.min_particles;
  }

  const double ratio = std::clamp(weight_variance / config_.target_variance, 0.0, 1.0);
  const double span = static_cast<double>(config_.max_particles - config_.min_particles);
  const size_t target = config_.min_particles + static_cast<size_t>(std::round(ratio * span));
  return std::clamp(target, config_.min_particles, config_.max_particles);
}

bool AdaptiveSampler::update_kidnap_detector(double avg_likelihood) noexcept {
  if (w_slow_ <= 0.0) {
    w_slow_ = avg_likelihood;
    w_fast_ = avg_likelihood;
    return false;
  }

  w_slow_ += config_.recovery_alpha_slow * (avg_likelihood - w_slow_);
  w_fast_ += config_.recovery_alpha_fast * (avg_likelihood - w_fast_);

  if (w_slow_ > 1e-12) {
    const double ratio = std::max(0.0, 1.0 - (w_fast_ / w_slow_));
    return ratio > config_.doping_threshold;
  }
  return false;
}

Particle AdaptiveSampler::sample_free_space(const MapData& map) {
  std::uniform_real_distribution<double> angle_dist(-std::numbers::pi, std::numbers::pi);
  const double theta = angle_dist(rng_);

  if (map.empty() || map.width == 0 || map.height == 0) {
    return Particle{0.0, 0.0, theta, 0.0};
  }

  std::uniform_int_distribution<uint32_t> x_dist(0, map.width - 1);
  std::uniform_int_distribution<uint32_t> y_dist(0, map.height - 1);

  // Try up to 100 times to find a free cell
  for (size_t attempt = 0; attempt < 100; ++attempt) {
    const int mx = static_cast<int>(x_dist(rng_));
    const int my = static_cast<int>(y_dist(rng_));
    if (map.is_free(mx, my)) {
      double wx = 0.0;
      double wy = 0.0;
      map.map_to_world(mx, my, wx, wy);
      return Particle{wx, wy, theta, 0.0};
    }
  }

  // Fallback to origin
  return Particle{map.origin.x, map.origin.y, theta, 0.0};
}

std::vector<Particle> AdaptiveSampler::resample(
    std::span<const Particle> current_particles,
    size_t target_count,
    bool do_doping,
    const MapData& map) {
  const size_t in_n = current_particles.size();
  std::vector<Particle> new_particles;
  new_particles.reserve(target_count);

  if (in_n == 0 || target_count == 0) {
    return new_particles;
  }

  // Low-variance resampling (stochastic universal sampling)
  const double step = 1.0 / static_cast<double>(target_count);
  std::uniform_real_distribution<double> u_dist(0.0, step);
  const double r = u_dist(rng_);

  double c = current_particles[0].weight;
  size_t i = 0;

  for (size_t m = 0; m < target_count; ++m) {
    const double u = r + static_cast<double>(m) * step;
    while (u > c && i + 1 < in_n) {
      ++i;
      c += current_particles[i].weight;
    }
    new_particles.push_back(current_particles[i]);
  }

  // Inject free-space doping if kidnap recovery is triggered
  if (do_doping && !map.empty()) {
    const size_t num_doping = std::min(config_.doping_count, target_count / 2);
    for (size_t d = 0; d < num_doping; ++d) {
      new_particles[target_count - 1 - d] = sample_free_space(map);
    }
  }

  // Reset weights uniformly
  const double new_weight = 1.0 / static_cast<double>(target_count);
  for (Particle& p : new_particles) {
    p.weight = new_weight;
  }

  return new_particles;
}

}  // namespace vapfl_core
