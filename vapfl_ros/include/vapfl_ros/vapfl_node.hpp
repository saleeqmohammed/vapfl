#pragma once

#include "vapfl_core/particle_filter.hpp"
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <mutex>
#include <string>

namespace vapfl_ros {

class VAPFLNode : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit VAPFLNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~VAPFLNode() override = default;

  // Lifecycle node callbacks
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void broadcast_tf(const vapfl_core::Pose2D& map_pose, const rclcpp::Time& stamp);

  // Parameters
  std::string global_frame_id_{"map"};
  std::string odom_frame_id_{"odom"};
  std::string base_frame_id_{"base_link"};
  std::string scan_topic_{"/scan"};
  std::string odom_topic_{"/odom"};
  std::string map_topic_{"/map"};
  std::string initial_pose_topic_{"/initialpose"};
  bool publish_tf_{true};
  double transform_tolerance_{0.1};
  size_t initial_particles_{300};

  vapfl_core::ParticleFilterConfig filter_config_{};

  // State
  std::mutex mutex_;
  std::unique_ptr<vapfl_core::ParticleFilter> filter_;
  bool has_map_{false};
  bool is_active_{false};
  vapfl_core::Pose2D latest_odom_pose_{};
  rclcpp::Time latest_odom_stamp_{0, 0, RCL_ROS_TIME};

  // ROS 2 Communication
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseArray>::SharedPtr cloud_pub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace vapfl_ros
