#include "vapfl_core/particle_filter.hpp"
#include <cmath>

namespace vapfl_core {

ParticleFilter::ParticleFilter(const ParticleFilterConfig& config)
    : config_(config),
      motion_model_(config.motion),
      sensor_model_(config.sensor),
      sampler_(config.sampler),
      clusterer_(config.dbscan),
      rng_(std::random_device{}()) {}

void ParticleFilter::set_config(const ParticleFilterConfig& config) {
  config_ = config;
  motion_model_.set_config(config.motion);
  sensor_model_.set_config(config.sensor);
  sampler_.set_config(config.sampler);
  clusterer_.set_config(config.dbscan);
}

void ParticleFilter::set_seed(uint64_t seed) {
  rng_.seed(seed);
  motion_model_.set_seed(seed);
  sampler_.set_seed(seed + 1);
}

void ParticleFilter::set_map(MapData map) {
  map_ = std::move(map);
}

void ParticleFilter::initialize_gaussian(
    const Pose2D& mean,
    const std::array<double, 3>& std_dev,
    size_t num_particles) {
  num_particles = std::max<size_t>(num_particles, config_.sampler.min_particles);
  particles_.clear();
  particles_.reserve(num_particles);

  std::normal_distribution<double> dist_x(0.0, std::max(1e-4, std_dev[0]));
  std::normal_distribution<double> dist_y(0.0, std::max(1e-4, std_dev[1]));
  std::normal_distribution<double> dist_th(0.0, std::max(1e-4, std_dev[2]));

  const double uniform_w = 1.0 / static_cast<double>(num_particles);

  for (size_t i = 0; i < num_particles; ++i) {
    const double px = mean.x + dist_x(rng_);
    const double py = mean.y + dist_y(rng_);
    const double pth = normalize_angle(mean.theta + dist_th(rng_));
    particles_.push_back(Particle{px, py, pth, uniform_w});
  }

  is_initialized_ = true;
  sampler_.reset_averages();

  const ClusterResult cluster_res = clusterer_.cluster(particles_);
  state_.pose = cluster_res.estimated_pose;
  state_.covariance = cluster_res.covariance;
  state_.particle_count = particles_.size();
  state_.weight_variance = 0.0;
  state_.effective_sample_size = static_cast<double>(num_particles);
  state_.converged = false;
}

void ParticleFilter::initialize_global(size_t num_particles) {
  num_particles = std::max<size_t>(num_particles, config_.sampler.min_particles);
  particles_.clear();
  particles_.reserve(num_particles);

  const double uniform_w = 1.0 / static_cast<double>(num_particles);

  for (size_t i = 0; i < num_particles; ++i) {
    Particle p = sampler_.sample_free_space(map_);
    p.weight = uniform_w;
    particles_.push_back(p);
  }

  is_initialized_ = true;
  sampler_.reset_averages();

  const ClusterResult cluster_res = clusterer_.cluster(particles_);
  state_.pose = cluster_res.estimated_pose;
  state_.covariance = cluster_res.covariance;
  state_.particle_count = particles_.size();
  state_.weight_variance = 0.0;
  state_.effective_sample_size = static_cast<double>(num_particles);
  state_.converged = false;
}

bool ParticleFilter::process_odometry(const Pose2D& odom) {
  if (!has_prev_odom_) {
    prev_odom_ = odom;
    last_update_odom_ = odom;
    has_prev_odom_ = true;
    return false;
  }

  if (is_initialized_ && !particles_.empty()) {
    motion_model_.predict(particles_, prev_odom_, odom);
  }

  prev_odom_ = odom;

  const double dx = odom.x - last_update_odom_.x;
  const double dy = odom.y - last_update_odom_.y;
  const double dtrans = std::hypot(dx, dy);
  const double drot = std::abs(normalize_angle(odom.theta - last_update_odom_.theta));

  if (dtrans >= config_.update_min_d || drot >= config_.update_min_a) {
    last_update_odom_ = odom;
    return true;
  }

  return false;
}

const FilterState& ParticleFilter::process_laser_scan(const LaserScanData& scan) {
  if (!is_initialized_ || particles_.empty()) {
    return state_;
  }

  // 1. Update weights with sensor model
  sensor_model_.update_weights(particles_, scan, map_);

  // 2. Normalize weights and calculate statistics
  double weight_var = 0.0;
  double neff = 0.0;
  double avg_lik = 0.0;
  sampler_.normalize_and_analyze(particles_, weight_var, neff, avg_lik);

  // 3. Kidnap / Divergence detection
  const bool do_doping = sampler_.update_kidnap_detector(avg_lik);

  // 4. Compute dynamic particle population
  const size_t target_count = sampler_.compute_target_particles(weight_var);

  // 5. Adaptive resampling with recovery injection
  particles_ = sampler_.resample(particles_, target_count, do_doping, map_);

  // 6. Cluster particles and estimate state
  const ClusterResult cluster_res = clusterer_.cluster(particles_);

  state_.pose = cluster_res.estimated_pose;
  state_.covariance = cluster_res.covariance;
  state_.weight_variance = weight_var;
  state_.effective_sample_size = neff;
  state_.particle_count = particles_.size();
  state_.converged = (weight_var < config_.sampler.target_variance * 2.0 &&
                      neff > static_cast<double>(particles_.size()) * 0.15);

  return state_;
}

}  // namespace vapfl_core
