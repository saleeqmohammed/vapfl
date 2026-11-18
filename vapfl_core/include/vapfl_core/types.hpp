#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <span>
#include <vector>

namespace vapfl_core {

/**
 * @brief Normalize an angle into the interval [-pi, pi).
 */
[[nodiscard]] constexpr double normalize_angle(double angle) noexcept {
  constexpr double pi = std::numbers::pi;
  constexpr double two_pi = 2.0 * std::numbers::pi;
  while (angle >= pi) {
    angle -= two_pi;
  }
  while (angle < -pi) {
    angle += two_pi;
  }
  return angle;
}

/**
 * @brief 2D Pose representation in SE(2)
 */
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};  // radians in [-pi, pi)

  [[nodiscard]] constexpr bool operator==(const Pose2D& other) const noexcept {
    return x == other.x && y == other.y && theta == other.theta;
  }

  [[nodiscard]] double distance_to(const Pose2D& other) const noexcept {
    const double dx = x - other.x;
    const double dy = y - other.y;
    return std::hypot(dx, dy);
  }
};

/**
 * @brief Compact 32-byte Particle POD struct aligned for cache efficiency.
 * Exactly two particles fit within a typical 64-byte L1 cache line.
 */
struct alignas(32) Particle {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
  double weight{0.0};

  [[nodiscard]] constexpr Pose2D to_pose() const noexcept {
    return {x, y, theta};
  }
};

/**
 * @brief 2D Occupancy Grid Map representation
 */
struct MapData {
  double resolution{0.05};  // meters per cell
  uint32_t width{0};        // cells
  uint32_t height{0};       // cells
  Pose2D origin{};          // origin pose in world coordinates
  std::vector<int8_t> data{};  // occupancy: 0 = free, 100 = occupied, -1 = unknown

  [[nodiscard]] bool empty() const noexcept {
    return data.empty() || width == 0 || height == 0;
  }

  [[nodiscard]] bool is_in_bounds(int mx, int my) const noexcept {
    return mx >= 0 && static_cast<uint32_t>(mx) < width &&
           my >= 0 && static_cast<uint32_t>(my) < height;
  }

  [[nodiscard]] bool world_to_map(double wx, double wy, int& mx, int& my) const noexcept {
    if (resolution <= 0.0) {
      return false;
    }
    const double dx = wx - origin.x;
    const double dy = wy - origin.y;
    // Account for map origin orientation if rotated
    const double cos_th = std::cos(-origin.theta);
    const double sin_th = std::sin(-origin.theta);
    const double rx = cos_th * dx - sin_th * dy;
    const double ry = sin_th * dx + cos_th * dy;

    mx = static_cast<int>(std::floor(rx / resolution));
    my = static_cast<int>(std::floor(ry / resolution));
    return is_in_bounds(mx, my);
  }

  void map_to_world(int mx, int my, double& wx, double& wy) const noexcept {
    const double rx = (static_cast<double>(mx) + 0.5) * resolution;
    const double ry = (static_cast<double>(my) + 0.5) * resolution;
    const double cos_th = std::cos(origin.theta);
    const double sin_th = std::sin(origin.theta);

    wx = origin.x + (cos_th * rx - sin_th * ry);
    wy = origin.y + (sin_th * rx + cos_th * ry);
  }

  [[nodiscard]] int8_t get_cell(int mx, int my) const noexcept {
    if (!is_in_bounds(mx, my)) {
      return -1;
    }
    const size_t idx = static_cast<size_t>(my) * static_cast<size_t>(width) + static_cast<size_t>(mx);
    return data[idx];
  }

  [[nodiscard]] bool is_occupied(int mx, int my, int8_t threshold = 50) const noexcept {
    const int8_t val = get_cell(mx, my);
    return val >= threshold;
  }

  [[nodiscard]] bool is_free(int mx, int my, int8_t free_threshold = 25) const noexcept {
    const int8_t val = get_cell(mx, my);
    return val >= 0 && val < free_threshold;
  }
};

/**
 * @brief Laser Scan message container decoupled from ROS
 */
struct LaserScanData {
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.05};
  double range_max{30.0};
  std::vector<double> ranges{};
};

/**
 * @brief Output Filter State with estimate and diagnostics
 */
struct FilterState {
  Pose2D pose{};
  std::array<double, 9> covariance{0.0};  // row-major 3x3 (xx, xy, xt, yx, yy, yt, tx, ty, tt)
  double weight_variance{0.0};
  double effective_sample_size{0.0};
  size_t particle_count{0};
  bool converged{false};
};

/**
 * @brief Motion Model Noise Parameters
 */
struct MotionModelConfig {
  double alpha1{0.05};  // rot noise from rot
  double alpha2{0.05};  // rot noise from trans
  double alpha3{0.10};  // trans noise from trans
  double alpha4{0.05};  // trans noise from rot
};

/**
 * @brief Sensor Model Probabilistic Parameters
 */
struct SensorModelConfig {
  double z_hit{0.80};
  double z_short{0.05};
  double z_max{0.05};
  double z_rand{0.10};
  double sigma_hit{0.20};      // std dev in meters
  double lambda_short{0.50};   // exponential decay param
  size_t beam_step{6};         // Decimation step across scan ranges (e.g. 1 of every 6 beams)
  size_t max_beams{60};        // Maximum total beams evaluated per update
};

/**
 * @brief Adaptive Particle Sampler & Recovery Doping Parameters
 */
struct AdaptiveSamplerConfig {
  size_t min_particles{100};
  size_t max_particles{800};
  double target_variance{0.02};
  double recovery_alpha_slow{0.001};
  double recovery_alpha_fast{0.1};
  size_t doping_count{15};     // Number of random particles injected during kidnap event
  double doping_threshold{0.05}; // Fast/slow likelihood ratio threshold
};

/**
 * @brief DBSCAN Spatial Clustering Parameters
 */
struct DBSCANConfig {
  double eps{0.30};            // Spatial radius in meters
  double eps_theta{0.50};      // Orientation difference in radians
  size_t min_pts{5};           // Minimum particles to form a core cluster
};

}  // namespace vapfl_core
