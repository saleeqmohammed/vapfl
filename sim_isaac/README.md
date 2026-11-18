# NVIDIA Isaac Sim Integration & Benchmarking for VAPFL

This directory contains configuration, scene setup, and benchmarking scripts to evaluate **VAPFL (Variance-based Adaptive Particle Filter Localization)** inside **NVIDIA Isaac Sim** using high-fidelity PhysX 5 dynamics and RTX 2D LiDAR.

---

## 1. Prerequisites

- **Host System:** Ubuntu 24.04 LTS with NVIDIA RTX GPU (Driver >= 535)
- **NVIDIA Isaac Sim:** Isaac Sim 4.0 or newer
- **ROS 2:** ROS 2 Jazzy Jalisco
- **Extension:** `omni.isaac.ros2_bridge`

---

## 2. Architecture & Components

- **`configs/lidar_config.json`**: Definition for RTX 2D Flatscan LiDAR (360° FOV, 0.5° angular increment, 720 beams, 20 Hz, Gaussian range noise).
- **`scripts/spawn_robot.py`**: Standalone Isaac Sim Python application creating the warehouse scene, differential drive mobile robot, and ROS 2 Jazzy publishers (`/scan`, `/odom`, `/tf`).
- **`scripts/run_kidnap_benchmark.py`**: Automated benchmarking harness commanding robot motion, injecting teleportation kidnap events, calculating RMSE vs Ground Truth, and measuring recovery convergence time ($< 0.15\text{m}$).

---

## 3. Step-by-Step Instructions

### Step 1: Launch Isaac Sim Robot & Environment
From your Isaac Sim installation directory:
```bash
./python.sh /path/to/vapfl/sim_isaac/scripts/spawn_robot.py
```
To run headlessly for automated CI/regression testing:
```bash
./python.sh /path/to/vapfl/sim_isaac/scripts/spawn_robot.py --headless
```

### Step 2: Build and Launch VAPFL in ROS 2 Jazzy
In your ROS 2 workspace:
```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# Launch VAPFL with auto-activation
ros2 launch vapfl vapfl.launch.py
```

### Step 3: Run the Kidnapped Robot Benchmark
In a separate terminal:
```bash
source /opt/ros/jazzy/setup.bash
python3 sim_isaac/scripts/run_kidnap_benchmark.py --duration 30.0
```

To run a verification test in mock mode without Isaac Sim running:
```bash
python3 sim_isaac/scripts/run_kidnap_benchmark.py --mock --duration 15.0
```

---

## 4. Benchmark Metrics

The benchmark measures:
1. **Pre-Kidnap RMSE**: Position accuracy during standard trajectory tracking ($< 0.05\text{m}$).
2. **Kidnap Recovery Time**: Duration (in seconds) from teleportation until the filter re-converges to within $0.15\text{m}$ of ground truth (Target: $< 3.0\text{s}$).
3. **Adaptive Sizing & Free-Space Doping**: Dynamic particle population adjustment from $N_{min} = 100$ up to $N_{max} = 800$, accompanied by uniform free-space injection.
4. **Data Export**: Complete trajectory, errors, and particle counts recorded to `kidnap_benchmark_results.csv`.
