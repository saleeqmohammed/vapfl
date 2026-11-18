#include "vapfl_ros/vapfl_node.hpp"
#include "vapfl_ros/conversions.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace vapfl_ros {

VAPFLNode::VAPFLNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("vapfl_node", options) {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
VAPFLNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Configuring VAPFL Lifecycle Node...");

  // Declare parameters
  global_frame_id_ = declare_parameter<std::string>("global_frame_id", "map");
  odom_frame_id_ = declare_parameter<std::string>("odom_frame_id", "odom");
  base_frame_id_ = declare_parameter<std::string>("base_frame_id", "base_link");
  scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
  map_topic_ = declare_parameter<std::string>("map_topic", "/map");
  initial_pose_topic_ = declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  transform_tolerance_ = declare_parameter<double>("transform_tolerance", 0.1);
  initial_particles_ = static_cast<size_t>(declare_parameter<int>("initial_particles", 300));

  // Motion model parameters
  filter_config_.motion.alpha1 = declare_parameter<double>("alpha1", 0.05);
  filter_config_.motion.alpha2 = declare_parameter<double>("alpha2", 0.05);
  filter_config_.motion.alpha3 = declare_parameter<double>("alpha3", 0.10);
  filter_config_.motion.alpha4 = declare_parameter<double>("alpha4", 0.05);

  // Sensor model parameters
  filter_config_.sensor.z_hit = declare_parameter<double>("z_hit", 0.80);
  filter_config_.sensor.z_short = declare_parameter<double>("z_short", 0.05);
  filter_config_.sensor.z_max = declare_parameter<double>("z_max", 0.05);
  filter_config_.sensor.z_rand = declare_parameter<double>("z_rand", 0.10);
  filter_config_.sensor.sigma_hit = declare_parameter<double>("sigma_hit", 0.20);
  filter_config_.sensor.lambda_short = declare_parameter<double>("lambda_short", 0.50);
  filter_config_.sensor.beam_step = static_cast<size_t>(declare_parameter<int>("beam_step", 6));
  filter_config_.sensor.max_beams = static_cast<size_t>(declare_parameter<int>("max_beams", 60));

  // Adaptive sampler parameters
  filter_config_.sampler.min_particles = static_cast<size_t>(declare_parameter<int>("min_particles", 100));
  filter_config_.sampler.max_particles = static_cast<size_t>(declare_parameter<int>("max_particles", 800));
  filter_config_.sampler.target_variance = declare_parameter<double>("target_variance", 0.02);
  filter_config_.sampler.recovery_alpha_slow = declare_parameter<double>("recovery_alpha_slow", 0.001);
  filter_config_.sampler.recovery_alpha_fast = declare_parameter<double>("recovery_alpha_fast", 0.1);
  filter_config_.sampler.doping_count = static_cast<size_t>(declare_parameter<int>("doping_count", 15));
  filter_config_.sampler.doping_threshold = declare_parameter<double>("doping_threshold", 0.05);

  // DBSCAN clustering parameters
  filter_config_.dbscan.eps = declare_parameter<double>("dbscan_eps", 0.30);
  filter_config_.dbscan.eps_theta = declare_parameter<double>("dbscan_eps_theta", 0.50);
  filter_config_.dbscan.min_pts = static_cast<size_t>(declare_parameter<int>("dbscan_min_pts", 5));

  // Filter motion thresholds
  filter_config_.update_min_d = declare_parameter<double>("update_min_d", 0.05);
  filter_config_.update_min_a = declare_parameter<double>("update_min_a", 0.05);

  // Initialize filter instance
  filter_ = std::make_unique<vapfl_core::ParticleFilter>(filter_config_);

  // Create lifecycle publishers
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10);
  cloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("/particlecloud", 10);

  // Create subscribers
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        map_callback(msg);
      });

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initial_pose_topic_,
      rclcpp::QoS(10),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        initial_pose_callback(msg);
      });

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      rclcpp::QoS(10),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_callback(msg);
      });

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_callback(msg);
      });

  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  RCLCPP_INFO(get_logger(), "VAPFL configured successfully.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
VAPFLNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Activating VAPFL Lifecycle Node...");
  pose_pub_->on_activate();
  cloud_pub_->on_activate();
  is_active_ = true;
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
VAPFLNode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating VAPFL Lifecycle Node...");
  is_active_ = false;
  pose_pub_->on_deactivate();
  cloud_pub_->on_deactivate();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
VAPFLNode::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up VAPFL Lifecycle Node...");
  is_active_ = false;
  has_map_ = false;
  pose_pub_.reset();
  cloud_pub_.reset();
  map_sub_.reset();
  initial_pose_sub_.reset();
  odom_sub_.reset();
  scan_sub_.reset();
  tf_broadcaster_.reset();
  filter_.reset();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
VAPFLNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(get_logger(), "Shutting down VAPFL Lifecycle Node...");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void VAPFLNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_INFO(get_logger(), "Received map: %u x %u @ %.3f m/cell",
              msg->info.width, msg->info.height, static_cast<double>(msg->info.resolution));

  const vapfl_core::MapData map = conversions::from_ros(*msg);
  if (filter_) {
    filter_->set_map(map);
    has_map_ = true;
  }
}

void VAPFLNode::initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  const vapfl_core::Pose2D pose = conversions::from_ros(*msg);
  const double std_x = std::sqrt(std::max(1e-4, msg->pose.covariance[0]));
  const double std_y = std::sqrt(std::max(1e-4, msg->pose.covariance[7]));
  const double std_th = std::sqrt(std::max(1e-4, msg->pose.covariance[35]));

  RCLCPP_INFO(get_logger(), "Setting initial pose: (%.2f, %.2f, %.2f rad)",
              pose.x, pose.y, pose.theta);

  if (filter_) {
    filter_->initialize_gaussian(pose, {std_x, std_y, std_th}, initial_particles_);
    if (is_active_) {
      const rclcpp::Time stamp = msg->header.stamp;
      const auto pose_msg = conversions::to_ros(filter_->state(), global_frame_id_, stamp);
      pose_pub_->publish(pose_msg);

      const auto cloud_msg = conversions::to_ros(filter_->particles(), global_frame_id_, stamp);
      cloud_pub_->publish(cloud_msg);

      if (publish_tf_) {
        broadcast_tf(filter_->state().pose, stamp);
      }
    }
  }
}

void VAPFLNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_odom_pose_ = conversions::from_ros(*msg);
  latest_odom_stamp_ = msg->header.stamp;

  if (is_active_ && filter_ && filter_->is_initialized()) {
    filter_->process_odometry(latest_odom_pose_);
  }
}

void VAPFLNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!is_active_ || !filter_ || !filter_->is_initialized() || !has_map_) {
    return;
  }

  const vapfl_core::LaserScanData scan_data = conversions::from_ros(*msg);
  const vapfl_core::FilterState& state = filter_->process_laser_scan(scan_data);

  const rclcpp::Time stamp = msg->header.stamp;

  // Publish estimated pose
  const auto pose_msg = conversions::to_ros(state, global_frame_id_, stamp);
  pose_pub_->publish(pose_msg);

  // Publish particle cloud
  const auto cloud_msg = conversions::to_ros(filter_->particles(), global_frame_id_, stamp);
  cloud_pub_->publish(cloud_msg);

  // Broadcast map -> odom transform
  if (publish_tf_) {
    broadcast_tf(state.pose, stamp);
  }
}

void VAPFLNode::broadcast_tf(const vapfl_core::Pose2D& map_pose, const rclcpp::Time& stamp) {
  if (!tf_broadcaster_) {
    return;
  }

  // T_map->odom = T_map->base * (T_odom->base)^-1
  const double xo = latest_odom_pose_.x;
  const double yo = latest_odom_pose_.y;
  const double tho = latest_odom_pose_.theta;

  const double cos_o = std::cos(tho);
  const double sin_o = std::sin(tho);

  const double x_inv = -xo * cos_o - yo * sin_o;
  const double y_inv = xo * sin_o - yo * cos_o;

  const double xm = map_pose.x;
  const double ym = map_pose.y;
  const double thm = map_pose.theta;

  const double cos_m = std::cos(thm);
  const double sin_m = std::sin(thm);

  const double tf_x = xm + x_inv * cos_m - y_inv * sin_m;
  const double tf_y = ym + x_inv * sin_m + y_inv * cos_m;
  const double tf_th = vapfl_core::normalize_angle(thm - tho);

  geometry_msgs::msg::TransformStamped tf_msg;
  // Apply transform tolerance to timestamp to prevent TF extrapolation errors
  tf_msg.header.stamp = stamp + rclcpp::Duration::from_seconds(transform_tolerance_);
  tf_msg.header.frame_id = global_frame_id_;
  tf_msg.child_frame_id = odom_frame_id_;

  tf_msg.transform.translation.x = tf_x;
  tf_msg.transform.translation.y = tf_y;
  tf_msg.transform.translation.z = 0.0;

  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  conversions::yaw_to_quaternion(tf_th, qx, qy, qz, qw);
  tf_msg.transform.rotation.x = qx;
  tf_msg.transform.rotation.y = qy;
  tf_msg.transform.rotation.z = qz;
  tf_msg.transform.rotation.w = qw;

  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace vapfl_ros
