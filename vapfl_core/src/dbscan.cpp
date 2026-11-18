#include "vapfl_core/dbscan.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>

namespace vapfl_core {

DBSCAN::DBSCAN(const DBSCANConfig& config)
    : config_(config) {}

bool DBSCAN::are_neighbors(const Particle& p1, const Particle& p2) const noexcept {
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dist_sq = dx * dx + dy * dy;
  if (dist_sq > config_.eps * config_.eps) {
    return false;
  }

  const double dtheta = std::abs(normalize_angle(p1.theta - p2.theta));
  return dtheta <= config_.eps_theta;
}

std::vector<size_t> DBSCAN::find_neighbors(
    std::span<const Particle> particles,
    size_t index) const {
  std::vector<size_t> neighbors;
  neighbors.reserve(particles.size() / 4);

  const Particle& target = particles[index];
  for (size_t i = 0; i < particles.size(); ++i) {
    if (are_neighbors(target, particles[i])) {
      neighbors.push_back(i);
    }
  }
  return neighbors;
}

ClusterResult DBSCAN::cluster(std::span<const Particle> particles) const {
  ClusterResult result;
  const size_t n = particles.size();
  if (n == 0) {
    return result;
  }

  result.labels.assign(n, -2);  // -2 = unvisited
  int current_cluster_id = 0;

  for (size_t i = 0; i < n; ++i) {
    if (result.labels[i] != -2) {
      continue;
    }

    std::vector<size_t> neighbors = find_neighbors(particles, i);
    if (neighbors.size() < config_.min_pts) {
      result.labels[i] = -1;  // Mark as noise initially
      continue;
    }

    result.labels[i] = current_cluster_id;
    std::queue<size_t> seed_queue;
    for (size_t neighbor_idx : neighbors) {
      if (neighbor_idx != i) {
        seed_queue.push(neighbor_idx);
      }
    }

    while (!seed_queue.empty()) {
      const size_t q_idx = seed_queue.front();
      seed_queue.pop();

      if (result.labels[q_idx] == -1) {
        result.labels[q_idx] = current_cluster_id;  // Border point converted from noise
      }

      if (result.labels[q_idx] != -2) {
        continue;
      }

      result.labels[q_idx] = current_cluster_id;
      std::vector<size_t> q_neighbors = find_neighbors(particles, q_idx);
      if (q_neighbors.size() >= config_.min_pts) {
        for (size_t next_idx : q_neighbors) {
          if (result.labels[next_idx] == -2 || result.labels[next_idx] == -1) {
            seed_queue.push(next_idx);
          }
        }
      }
    }

    ++current_cluster_id;
  }

  // Find dominant cluster by sum of weights
  struct ClusterStats {
    double total_weight{0.0};
    size_t count{0};
  };
  std::unordered_map<int, ClusterStats> cluster_stats;

  for (size_t i = 0; i < n; ++i) {
    const int label = result.labels[i];
    if (label >= 0) {
      cluster_stats[label].total_weight += particles[i].weight;
      cluster_stats[label].count += 1;
    }
  }

  int best_cluster_id = -1;
  double max_weight = -1.0;
  for (const auto& [id, stats] : cluster_stats) {
    if (stats.total_weight > max_weight) {
      max_weight = stats.total_weight;
      best_cluster_id = id;
    }
  }

  result.dominant_cluster_id = best_cluster_id;
  if (best_cluster_id >= 0) {
    result.dominant_cluster_size = cluster_stats[best_cluster_id].count;
    result.dominant_cluster_weight = cluster_stats[best_cluster_id].total_weight;
  }

  // Compute weighted mean pose and covariance
  // If a dominant cluster exists, use its particles; otherwise fall back to all particles
  double sum_w = 0.0;
  double mean_x = 0.0;
  double mean_y = 0.0;
  double sin_sum = 0.0;
  double cos_sum = 0.0;

  for (size_t i = 0; i < n; ++i) {
    if (best_cluster_id < 0 || result.labels[i] == best_cluster_id) {
      const Particle& p = particles[i];
      const double w = (p.weight > 0.0) ? p.weight : (1.0 / static_cast<double>(n));
      sum_w += w;
      mean_x += w * p.x;
      mean_y += w * p.y;
      sin_sum += w * std::sin(p.theta);
      cos_sum += w * std::cos(p.theta);
    }
  }

  if (sum_w > 0.0) {
    const double inv_w = 1.0 / sum_w;
    mean_x *= inv_w;
    mean_y *= inv_w;
    const double mean_theta = std::atan2(sin_sum, cos_sum);

    result.estimated_pose = {mean_x, mean_y, mean_theta};

    // Calculate 3x3 covariance matrix
    double c_xx = 0.0;
    double c_xy = 0.0;
    double c_xt = 0.0;
    double c_yy = 0.0;
    double c_yt = 0.0;
    double c_tt = 0.0;

    for (size_t i = 0; i < n; ++i) {
      if (best_cluster_id < 0 || result.labels[i] == best_cluster_id) {
        const Particle& p = particles[i];
        const double w = (p.weight > 0.0) ? p.weight : (1.0 / static_cast<double>(n));
        const double dx = p.x - mean_x;
        const double dy = p.y - mean_y;
        const double dt = normalize_angle(p.theta - mean_theta);

        c_xx += w * dx * dx;
        c_xy += w * dx * dy;
        c_xt += w * dx * dt;
        c_yy += w * dy * dy;
        c_yt += w * dy * dt;
        c_tt += w * dt * dt;
      }
    }

    result.covariance = {
        c_xx * inv_w, c_xy * inv_w, c_xt * inv_w,
        c_xy * inv_w, c_yy * inv_w, c_yt * inv_w,
        c_xt * inv_w, c_yt * inv_w, c_tt * inv_w};
  }

  return result;
}

}  // namespace vapfl_core
