# VAPFL: Variance-based Adaptive Particle Filter Localization

A modern, production-grade 2D Particle Filter Localization system written in Modern C++ (C++20) for **ROS 2 Jazzy Jalisco** and verified in **NVIDIA Isaac Sim**.

## Architecture Overview

- **`vapfl_core/`**: Pure C++20 algorithmic engine with zero ROS dependencies. Contains the motion model, raycasting sensor model, variance-based adaptive particle sampler, and DBSCAN spatial clusterer. Unit tested with Google Test and executed via CTest.
- **`vapfl_ros/`**: ROS 2 Jazzy wrapper implementing a managed `rclcpp_lifecycle::LifecycleNode`, parameter validation, sensor subscribers (`/scan`, `/odom`), TF broadcasting, and zero-copy data conversion.
- **`sim_isaac/`**: NVIDIA Isaac Sim USD environments, RTX LiDAR configurations, and automated benchmarking scripts for kidnapped robot recovery.

For the full architectural specification, compiler standards, and phase-by-phase implementation plan, see [ROADMAP.md](ROADMAP.md).
