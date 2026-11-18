#include "vapfl_core/sensor_model.hpp"
#include <cmath>
#include <limits>

namespace vapfl_core {

SensorModel::SensorModel(const SensorModelConfig& config)
    : config_(config) {}

double SensorModel::raycast(
    const MapData& map,
    double start_x,
    double start_y,
    double angle,
    double max_range) noexcept {
  if (map.empty() || map.resolution <= 0.0) {
    return max_range;
  }

  // Transform start position into map coordinate frame
  const double dx_orig = start_x - map.origin.x;
  const double dy_orig = start_y - map.origin.y;
  const double cos_map = std::cos(-map.origin.theta);
  const double sin_map = std::sin(-map.origin.theta);
  const double rx = cos_map * dx_orig - sin_map * dy_orig;
  const double ry = sin_map * dx_orig + cos_map * dy_orig;

  const double ray_angle = normalize_angle(angle - map.origin.theta);
  const double dir_x = std::cos(ray_angle);
  const double dir_y = std::sin(ray_angle);

  const double u = rx / map.resolution;
  const double v = ry / map.resolution;

  int mx = static_cast<int>(std::floor(u));
  int my = static_cast<int>(std::floor(v));

  if (!map.is_in_bounds(mx, my)) {
    return max_range;
  }

  if (map.is_occupied(mx, my)) {
    return 0.0;
  }

  const int step_x = (dir_x > 0.0) ? 1 : ((dir_x < 0.0) ? -1 : 0);
  const int step_y = (dir_y > 0.0) ? 1 : ((dir_y < 0.0) ? -1 : 0);

  constexpr double inf = std::numeric_limits<double>::infinity();
  const double t_delta_x = (dir_x != 0.0) ? std::abs(1.0 / dir_x) : inf;
  const double t_delta_y = (dir_y != 0.0) ? std::abs(1.0 / dir_y) : inf;

  double t_max_x = 0.0;
  if (dir_x > 0.0) {
    t_max_x = (std::floor(u) + 1.0 - u) * t_delta_x;
  } else if (dir_x < 0.0) {
    t_max_x = (u - std::floor(u)) * t_delta_x;
  } else {
    t_max_x = inf;
  }

  double t_max_y = 0.0;
  if (dir_y > 0.0) {
    t_max_y = (std::floor(v) + 1.0 - v) * t_delta_y;
  } else if (dir_y < 0.0) {
    t_max_y = (v - std::floor(v)) * t_delta_y;
  } else {
    t_max_y = inf;
  }

  const double max_cell_dist = max_range / map.resolution;
  double t_curr = 0.0;

  while (t_curr < max_cell_dist) {
    if (t_max_x < t_max_y) {
      t_curr = t_max_x;
      t_max_x += t_delta_x;
      mx += step_x;
    } else {
      t_curr = t_max_y;
      t_max_y += t_delta_y;
      my += step_y;
    }

    if (!map.is_in_bounds(mx, my)) {
      return max_range;
    }

    if (map.is_occupied(mx, my)) {
      return std::min(t_curr * map.resolution, max_range);
    }
  }

  return max_range;
}

double SensorModel::compute_likelihood(
    const Particle& particle,
    const LaserScanData& scan,
    const MapData& map) const noexcept {
  if (scan.ranges.empty() || map.empty()) {
    return 1.0;
  }

  int pmx = 0;
  int pmy = 0;
  if (!map.world_to_map(particle.x, particle.y, pmx, pmy) || map.is_occupied(pmx, pmy)) {
    return 1e-12;
  }

  const size_t total_ranges = scan.ranges.size();
  const size_t step = std::max<size_t>(1, std::max(config_.beam_step, total_ranges / config_.max_beams));

  constexpr double sqrt_2pi = 2.5066282746310005;
  const double norm_hit = 1.0 / (sqrt_2pi * config_.sigma_hit);
  const double two_sigma_sq = 2.0 * config_.sigma_hit * config_.sigma_hit;
  const double inv_range_max = 1.0 / scan.range_max;

  double log_likelihood = 0.0;
  size_t valid_beams = 0;

  for (size_t i = 0; i < total_ranges; i += step) {
    const double z = scan.ranges[i];
    if (!std::isfinite(z) || z < scan.range_min || z > scan.range_max) {
      continue;
    }

    const double beam_angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double world_beam_angle = normalize_angle(particle.theta + beam_angle);

    const double z_exp = raycast(map, particle.x, particle.y, world_beam_angle, scan.range_max);

    // Hit component
    const double dz = z - z_exp;
    const double p_hit = norm_hit * std::exp(-(dz * dz) / two_sigma_sq);

    // Short component
    const double p_short = (z <= z_exp) ? (config_.lambda_short * std::exp(-config_.lambda_short * z)) : 0.0;

    // Max component
    const double p_max = (z >= scan.range_max - 0.05) ? 1.0 : 0.0;

    // Random component
    const double p_rand = inv_range_max;

    const double p_total = config_.z_hit * p_hit +
                           config_.z_short * p_short +
                           config_.z_max * p_max +
                           config_.z_rand * p_rand;

    log_likelihood += std::log(std::max(1e-9, p_total));
    ++valid_beams;
  }

  if (valid_beams == 0) {
    return 1.0;
  }

  // Use normalized mean log-likelihood to prevent numeric underflow
  return std::exp(log_likelihood / static_cast<double>(valid_beams));
}

void SensorModel::update_weights(
    std::span<Particle> particles,
    const LaserScanData& scan,
    const MapData& map) const {
  for (Particle& p : particles) {
    const double lik = compute_likelihood(p, scan, map);
    p.weight *= lik;
  }
}

}  // namespace vapfl_core
