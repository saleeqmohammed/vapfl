#pragma once

#include "vapfl_core/types.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <span>
#include <string>

namespace vapfl_ros::conversions {

/**
 * @brief Convert yaw angle to orientation quaternion.
 */
inline void yaw_to_quaternion(double yaw, double& qx, double& qy, double& qz, double& qw) noexcept {
  const double half_yaw = 0.5 * yaw;
  qx = 0.0;
  qy = 0.0;
  qz = std::sin(half_yaw);
  qw = std::cos(half_yaw);
}

/**
 * @brief Extract 2D yaw angle from quaternion.
 */
[[nodiscard]] inline double quaternion_to_yaw(double qx, double qy, double qz, double qw) noexcept {
  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  return std::atan2(siny_cosp, cosy_cosp);
}

/**
 * @brief Convert nav_msgs::msg::OccupancyGrid to vapfl_core::MapData.
 */
[[nodiscard]] inline vapfl_core::MapData from_ros(const nav_msgs::msg::OccupancyGrid& msg) {
  vapfl_core::MapData map;
  map.resolution = static_cast<double>(msg.info.resolution);
  map.width = msg.info.width;
  map.height = msg.info.height;
  map.origin.x = msg.info.origin.position.x;
  map.origin.y = msg.info.origin.position.y;
  map.origin.theta = quaternion_to_yaw(
      msg.info.origin.orientation.x,
      msg.info.origin.orientation.y,
      msg.info.origin.orientation.z,
      msg.info.origin.orientation.w);

  map.data.assign(msg.data.begin(), msg.data.end());
  return map;
}

/**
 * @brief Convert sensor_msgs::msg::LaserScan to vapfl_core::LaserScanData.
 */
[[nodiscard]] inline vapfl_core::LaserScanData from_ros(const sensor_msgs::msg::LaserScan& msg) {
  vapfl_core::LaserScanData scan;
  scan.angle_min = static_cast<double>(msg.angle_min);
  scan.angle_max = static_cast<double>(msg.angle_max);
  scan.angle_increment = static_cast<double>(msg.angle_increment);
  scan.range_min = static_cast<double>(msg.range_min);
  scan.range_max = static_cast<double>(msg.range_max);

  scan.ranges.reserve(msg.ranges.size());
  for (const float r : msg.ranges) {
    scan.ranges.push_back(static_cast<double>(r));
  }
  return scan;
}

/**
 * @brief Convert nav_msgs::msg::Odometry to vapfl_core::Pose2D.
 */
[[nodiscard]] inline vapfl_core::Pose2D from_ros(const nav_msgs::msg::Odometry& msg) {
  return vapfl_core::Pose2D{
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      quaternion_to_yaw(
          msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
          msg.pose.pose.orientation.w)};
}

/**
 * @brief Convert geometry_msgs::msg::PoseWithCovarianceStamped to vapfl_core::Pose2D.
 */
[[nodiscard]] inline vapfl_core::Pose2D from_ros(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
  return vapfl_core::Pose2D{
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      quaternion_to_yaw(
          msg.pose.pose.orientation.x,
          msg.pose.pose.orientation.y,
          msg.pose.pose.orientation.z,
          msg.pose.pose.orientation.w)};
}

/**
 * @brief Convert vapfl_core::FilterState to geometry_msgs::msg::PoseWithCovarianceStamped.
 */
[[nodiscard]] inline geometry_msgs::msg::PoseWithCovarianceStamped to_ros(
    const vapfl_core::FilterState& state,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  msg.pose.pose.position.x = state.pose.x;
  msg.pose.pose.position.y = state.pose.y;
  msg.pose.pose.position.z = 0.0;

  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  yaw_to_quaternion(state.pose.theta, qx, qy, qz, qw);
  msg.pose.pose.orientation.x = qx;
  msg.pose.pose.orientation.y = qy;
  msg.pose.pose.orientation.z = qz;
  msg.pose.pose.orientation.w = qw;

  // Fill 6x6 ROS covariance matrix: [x, y, z, roll, pitch, yaw]
  // 0: xx, 1: xy, 5: xt
  // 6: yx, 7: yy, 11: yt
  // 30: tx, 31: ty, 35: tt
  msg.pose.covariance[0] = state.covariance[0];   // xx
  msg.pose.covariance[1] = state.covariance[1];   // xy
  msg.pose.covariance[5] = state.covariance[2];   // xt
  msg.pose.covariance[6] = state.covariance[3];   // yx
  msg.pose.covariance[7] = state.covariance[4];   // yy
  msg.pose.covariance[11] = state.covariance[5];  // yt
  msg.pose.covariance[30] = state.covariance[6];  // tx
  msg.pose.covariance[31] = state.covariance[7];  // ty
  msg.pose.covariance[35] = state.covariance[8];  // tt

  return msg;
}

/**
 * @brief Convert particle span to geometry_msgs::msg::PoseArray.
 */
[[nodiscard]] inline geometry_msgs::msg::PoseArray to_ros(
    std::span<const vapfl_core::Particle> particles,
    const std::string& frame_id,
    const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.poses.reserve(particles.size());

  for (const auto& p : particles) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = p.x;
    pose.position.y = p.y;
    pose.position.z = 0.0;

    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    yaw_to_quaternion(p.theta, qx, qy, qz, qw);
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;

    msg.poses.push_back(pose);
  }
  return msg;
}

}  // namespace vapfl_ros::conversions
