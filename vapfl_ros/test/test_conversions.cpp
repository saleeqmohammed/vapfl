#include <gtest/gtest.h>
#include "vapfl_ros/conversions.hpp"
#include <cmath>

using namespace vapfl_ros;

TEST(ConversionsTest, YawQuaternionRoundTrip) {
  constexpr double test_yaw = 1.234;
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;

  conversions::yaw_to_quaternion(test_yaw, qx, qy, qz, qw);
  const double recovered_yaw = conversions::quaternion_to_yaw(qx, qy, qz, qw);

  EXPECT_NEAR(recovered_yaw, test_yaw, 1e-6);
}

TEST(ConversionsTest, LaserScanConversion) {
  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.angle_min = -1.57f;
  scan_msg.angle_max = 1.57f;
  scan_msg.angle_increment = 0.01f;
  scan_msg.range_min = 0.05f;
  scan_msg.range_max = 25.0f;
  scan_msg.ranges = {1.0f, 2.5f, 3.2f, 4.0f};

  const vapfl_core::LaserScanData data = conversions::from_ros(scan_msg);

  EXPECT_NEAR(data.angle_min, -1.57, 1e-4);
  EXPECT_NEAR(data.range_max, 25.0, 1e-4);
  ASSERT_EQ(data.ranges.size(), 4);
  EXPECT_NEAR(data.ranges[1], 2.5, 1e-4);
}

TEST(ConversionsTest, StateToPoseWithCovariance) {
  vapfl_core::FilterState state;
  state.pose = {3.5, 4.2, 0.785};
  state.covariance = {
      0.01, 0.002, 0.001,
      0.002, 0.02, 0.003,
      0.001, 0.003, 0.05};

  const rclcpp::Time stamp(100, 0, RCL_ROS_TIME);
  const auto msg = conversions::to_ros(state, "map", stamp);

  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_NEAR(msg.pose.pose.position.x, 3.5, 1e-6);
  EXPECT_NEAR(msg.pose.pose.position.y, 4.2, 1e-6);
  EXPECT_NEAR(msg.pose.covariance[0], 0.01, 1e-6);
  EXPECT_NEAR(msg.pose.covariance[7], 0.02, 1e-6);
  EXPECT_NEAR(msg.pose.covariance[35], 0.05, 1e-6);
}

TEST(ConversionsTest, ParticlesToPoseArray) {
  std::vector<vapfl_core::Particle> particles = {
      {1.0, 2.0, 0.5, 0.25},
      {3.0, 4.0, -0.5, 0.75}};

  const rclcpp::Time stamp(50, 0, RCL_ROS_TIME);
  const auto msg = conversions::to_ros(particles, "map", stamp);

  EXPECT_EQ(msg.header.frame_id, "map");
  ASSERT_EQ(msg.poses.size(), 2);
  EXPECT_NEAR(msg.poses[0].position.x, 1.0, 1e-6);
  EXPECT_NEAR(msg.poses[1].position.y, 4.0, 1e-6);
}
