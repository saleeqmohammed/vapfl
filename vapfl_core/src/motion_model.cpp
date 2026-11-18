#include "vapfl_core/motion_model.hpp"
#include <cmath>

namespace vapfl_core {

MotionModel::MotionModel(const MotionModelConfig& config)
    : config_(config), rng_(std::random_device{}()) {}

void MotionModel::set_seed(uint64_t seed) {
  rng_.seed(seed);
}

void MotionModel::predict(
    std::span<Particle> particles,
    const Pose2D& prev_odom,
    const Pose2D& curr_odom) {
  const double dx = curr_odom.x - prev_odom.x;
  const double dy = curr_odom.y - prev_odom.y;
  const double dtrans = std::hypot(dx, dy);

  double drot1 = 0.0;
  double drot2 = 0.0;

  constexpr double trans_eps = 1e-4;
  if (dtrans > trans_eps) {
    drot1 = normalize_angle(std::atan2(dy, dx) - prev_odom.theta);
    drot2 = normalize_angle(curr_odom.theta - prev_odom.theta - drot1);
  } else {
    drot1 = 0.0;
    drot2 = normalize_angle(curr_odom.theta - prev_odom.theta);
  }

  // Variances based on motion noise parameters
  const double drot1_sq = drot1 * drot1;
  const double dtrans_sq = dtrans * dtrans;
  const double drot2_sq = drot2 * drot2;

  const double var_rot1 = config_.alpha1 * drot1_sq + config_.alpha2 * dtrans_sq;
  const double var_trans = config_.alpha3 * dtrans_sq + config_.alpha4 * (drot1_sq + drot2_sq);
  const double var_rot2 = config_.alpha1 * drot2_sq + config_.alpha2 * dtrans_sq;

  const double std_rot1 = std::sqrt(std::max(1e-12, var_rot1));
  const double std_trans = std::sqrt(std::max(1e-12, var_trans));
  const double std_rot2 = std::sqrt(std::max(1e-12, var_rot2));

  std::normal_distribution<double> dist_rot1(0.0, std_rot1);
  std::normal_distribution<double> dist_trans(0.0, std_trans);
  std::normal_distribution<double> dist_rot2(0.0, std_rot2);

  for (Particle& p : particles) {
    const double noisy_rot1 = drot1 + dist_rot1(rng_);
    const double noisy_trans = dtrans + dist_trans(rng_);
    const double noisy_rot2 = drot2 + dist_rot2(rng_);

    const double travel_theta = p.theta + noisy_rot1;
    p.x += noisy_trans * std::cos(travel_theta);
    p.y += noisy_trans * std::sin(travel_theta);
    p.theta = normalize_angle(p.theta + noisy_rot1 + noisy_rot2);
  }
}

}  // namespace vapfl_core
