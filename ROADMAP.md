# VAPFL Production Roadmap (ROS 2 Jazzy)

This roadmap details the engineering plan for **Variance-based Adaptive Particle Filter Localization (VAPFL)** as an independent, industry-grade, safety-critical robotics package written from scratch in Modern C++ (C++20) for ROS 2 Jazzy.

**Target Stack:**
- **OS & Middleware:** Ubuntu 24.04 LTS & **ROS 2 Jazzy Jalisco**
- **Language Standard:** **Modern C++ (C++20)**
- **Build System:** Modern Target-Based CMake + `ament_cmake`
- **Testing & Quality Assurance:** Google Test (GTest), CTest, AddressSanitizer (ASan), UndefinedBehaviorSanitizer (UBSan)
- **CI/CD:** GitHub Actions with strict compiler warnings (`-Werror`, `-Wall`, `-Wextra`, `-Wpedantic`) and automated static analysis (`clang-tidy`, `clang-format`)
- **Simulation Environment:** **NVIDIA Isaac Sim** (PhysX 5, RTX 2D LiDAR, ROS 2 Bridge)

---

## 1. System Architecture: Decoupled Design

VAPFL is architected from scratch with strict separation of concerns: **core localization algorithms are completely decoupled from the ROS middleware**.

```
vapfl/
├── .github/
│   └── workflows/
│       ├── ci.yml                 # Build, test, & pedantic warnings check
│       └── lint.yml               # clang-format & clang-tidy validation
├── .clang-format                  # Standardized code formatting
├── .clang-tidy                    # Modern C++ static analysis rules
├── CMakeLists.txt                 # Root package CMake
├── package.xml                    # ROS 2 package manifest (format 3)
├── cmake/
│   └── CompilerWarnings.cmake     # Reusable pedantic warning flags target
├── vapfl_core/                    # PURE C++20 ALGORITHMIC LIBRARY (Zero ROS deps)
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── vapfl_core/
│   │       ├── types.hpp          # Particle, Pose2D, MapData, Config structs
│   │       ├── motion_model.hpp   # Odometry motion prediction with noise
│   │       ├── sensor_model.hpp   # Fast Bresenham raycasting & AMCL beam model
│   │       ├── adaptive_sampler.hpp # Variance-based dynamic particle count & doping
│   │       ├── dbscan.hpp         # C++20 spatial clustering for pose estimation
│   │       └── particle_filter.hpp# Algorithmic orchestrator
│   ├── src/
│   │   ├── motion_model.cpp
│   │   ├── sensor_model.cpp
│   │   ├── adaptive_sampler.cpp
│   │   ├── dbscan.cpp
│   │   └── particle_filter.cpp
│   └── tests/                     # Pure GTest suite executed via CTest (<1s runtime)
│       ├── CMakeLists.txt
│       ├── test_motion_model.cpp
│       ├── test_sensor_model.cpp
│       ├── test_adaptive_sampler.cpp
│       └── test_dbscan.cpp
├── vapfl_ros/                     # ROS 2 JAZZY LIFECYCLE NODE WRAPPER
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── vapfl_ros/
│   │       ├── vapfl_node.hpp     # rclcpp_lifecycle::LifecycleNode
│   │       └── conversions.hpp    # Zero-copy ROS msg <-> vapfl_core types
│   ├── src/
│   │   ├── vapfl_node.cpp
│   │   └── main.cpp
│   ├── config/
│   │   └── params.yaml            # Tunable filter and topic parameters
│   └── launch/
│       └── vapfl.launch.py        # Launch file with lifecycle manager
└── sim_isaac/                     # NVIDIA Isaac Sim Integration
    ├── scenes/
    │   └── warehouse_navigation.usd
    ├── configs/
    │   └── lidar_config.json
    └── scripts/
        ├── spawn_robot.py         # Automated scene and robot spawner
        └── run_kidnap_benchmark.py# Teleports robot and measures recovery time
```

---

## 2. Production C++ Standards & Memory Safety

1. **RAII & Smart Pointers:**
   - **Rule of Zero / Five:** Avoid manual `new`/`delete`.
   - **`std::unique_ptr`:** Enforce single ownership of heavy resources (e.g., Occupancy Grid data and Sensor Model instances).
   - **`std::shared_ptr`:** Restricted only to cross-thread or asynchronous ROS 2 resource sharing.
   - **Views over Copies:** Utilize `std::span<const double>` (C++20) for scan ranges and `std::string_view` for non-owning string references.
2. **Cache Locality:**
   - Instead of heap-allocating individual `Pose` objects on every update, store particles in contiguous memory (`std::vector<Particle>` where `Particle` is a compact 32-byte POD struct: `{double x, y, theta, weight;}`).
3. **Type Safety & Attributes:**
   - Use `[[nodiscard]]` on queries, state estimators, and calculations.
   - Strict `const` correctness on member functions and parameter inputs.

---

## 3. Build Systems: CMake & `ament_cmake` in ROS 2 Jazzy

### 3.1 Understanding CMakeLists.txt
CMake generates native build systems (like Ninja or Makefiles). Modern CMake uses **Target-Based** declarations:
- Targets (`add_library`, `add_executable`) encapsulate compile flags, definitions, and headers.
- Visibility specifiers isolate dependencies:
  - `PRIVATE`: Required only to build this target.
  - `INTERFACE`: Required only by downstream targets consuming this library.
  - `PUBLIC`: Required both to build this target and by its consumers.

### 3.2 Role of `ament_cmake`
`ament_cmake` extends CMake specifically for ROS 2:
- Integrates with `package.xml` for package metadata and dependency tracking.
- Registers packages into the Ament index (`ament_package()`) so `ros2 run` and `ros2 launch` locate executables and launch files.
- Provides standardized macros for exports (`ament_export_targets()`, `ament_export_dependencies()`).
- Provides automated test registration via `ament_add_gtest()`.

---

## 4. Pedantic Compiler Warnings & Static Analysis

To prevent undefined behavior, integer/sign conversions, and memory errors, builds will enforce strict warnings as errors.

### 4.1 Warning Flags (Scoped via Interface Target)
```cmake
# cmake/CompilerWarnings.cmake
function(vapfl_set_compiler_warnings target_name)
  set(CLANG_GCC_WARNINGS
    -Wall
    -Wextra
    -Wpedantic
    -Werror                 # Treat all warnings as fatal errors
    -Wconversion            # Warn on implicit conversions (e.g., double to float/int)
    -Wsign-conversion       # Warn on sign conversions
    -Wshadow                # Warn when a local variable shadows another
    -Wnon-virtual-dtor      # Warn if a polymorphic class has non-virtual destructor
    -Wold-style-cast        # Enforce C++ static_cast / reinterpret_cast
    -Wcast-align            # Warn on alignment-increasing pointer casts
    -Wunused                # Warn on unused variables and functions
    -Woverloaded-virtual    # Warn if an overload hides a virtual method
    -Wnull-dereference      # Warn on potential null pointer dereferences
    -Wdouble-promotion      # Warn when float is implicitly promoted to double
    -Wformat=2              # Enforce strict format string validation
  )
  target_compile_options(${target_name} PRIVATE ${CLANG_GCC_WARNINGS})
endfunction()
```

### 4.2 Sanitizers & Linters
- **AddressSanitizer (ASan) & UBSan:** Enabled during test builds via `-fsanitize=address,undefined` to catch memory leaks, out-of-bounds reads, and undefined math operations.
- **`clang-tidy`:** Configured with `modernize-*`, `bugprone-*`, `performance-*`, and `readability-*` checks.
- **`clang-format`:** Automated style enforcement across all `.cpp` and `.hpp` files.

---

## 5. Unit & Integration Testing (GTest + CTest)

Because `vapfl_core` has zero ROS dependencies:
- Core unit tests run via **CTest** in sub-seconds without ROS middleware startup.
- Fast local feedback loop during development.

### Test Matrix:
1. `test_motion_model.cpp`: Verify Gaussian dispersal of particles under odometric translation and rotation.
2. `test_sensor_model.cpp`: Validate Bresenham ray-casting and beam probability model against synthetic 2D maps.
3. `test_adaptive_sampler.cpp`: Verify particle population scaling ($N_{particles} \in [100, 800]$) and doping count ($N_{dops} \in [10, 20]$) as variance changes.
4. `test_dbscan.cpp`: Test clustering on multi-modal particle distributions and outlier rejection.
5. `test_lifecycle_node.cpp`: ROS 2 integration tests validating state transitions (`configure`, `activate`, `deactivate`) and message handling.

---

## 6. GitHub Actions CI/CD Pipeline

Automated workflow running on `push` and `pull_request` using the official **ROS 2 Jazzy** container image:

```yaml
# .github/workflows/ci.yml
name: VAPFL Production CI (ROS 2 Jazzy)

on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main ]

jobs:
  lint-and-format:
    runs-on: ubuntu-24.04
    steps:
      - uses: actions/checkout@v4
      - name: Install Linters
        run: sudo apt-get update && sudo apt-get install -y clang-format-18 clang-tidy-18
      - name: Verify Code Formatting
        run: |
          find vapfl_core vapfl_ros -name '*.cpp' -o -name '*.hpp' | xargs clang-format-18 --dry-run --Werror

  build-and-test:
    runs-on: ubuntu-24.04
    container:
      image: ros:jazzy-ros-base
    steps:
      - uses: actions/checkout@v4
      - name: Install Build Dependencies
        run: |
          apt-get update && apt-get install -y \
            build-essential \
            cmake \
            git \
            python3-colcon-common-extensions \
            libgtest-dev \
            libeigen3-dev
      - name: Install ROS Dependencies
        run: |
          rosdep update
          rosdep install --from-paths . --ignore-src -y
      - name: Build with Strict Warnings
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon build \
            --cmake-args \
              -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DVAPFL_ENABLE_PEDANTIC_WARNINGS=ON \
              -DVAPFL_ENABLE_WERROR=ON
      - name: Execute Tests via CTest
        run: |
          source /opt/ros/jazzy/setup.bash
          colcon test --event-handlers console_direct+
          colcon test-result --all --verbose
```

---

## 7. NVIDIA Isaac Sim Integration & Benchmarking

VAPFL natively integrates with **NVIDIA Isaac Sim** for photorealistic and high-fidelity physics validation:

1. **Omniverse ROS 2 Bridge:** Communicates with ROS 2 Jazzy topics using the `omni.isaac.ros2_bridge` extension.
2. **Sensors & Dynamics:**
   - RTX 2D Flatscan LiDAR publishing `sensor_msgs/msg/LaserScan` to `/scan`.
   - PhysX differential drive controller publishing `nav_msgs/msg/Odometry` and TF (`odom` $\rightarrow$ `base_link`).
3. **Automated Kidnapped Robot Benchmarking:**
   - A standalone Python script interfaces with Isaac Sim to simulate regular navigation and inject kidnap events (teleporting the robot).
   - Real-time logging of Ground Truth Pose vs `/amcl_pose` (RMSE), particle weight variance, dynamic particle count, and re-convergence time.
   - Comparative evaluation against standard `nav2_amcl`.

---

## 8. Implementation Phases

| Phase | Milestone | Deliverables | Verification Metric |
| :--- | :--- | :--- | :--- |
| **Phase 1** | **Build & Project Skeleton** | Root `CMakeLists.txt`, `package.xml` (Jazzy), `CompilerWarnings.cmake`, directory layout for `vapfl_core` and `vapfl_ros`. | Clean build under `-Wall -Wextra -Wpedantic -Werror`. |
| **Phase 2** | **Core Algorithmic Engine** | Pure C++20 `vapfl_core`: `MotionModel`, `SensorModel` (Bresenham raycaster), `AdaptiveSampler`, `DBSCAN`, and `ParticleFilter`. | Complete GTest test suite passing via `ctest`. |
| **Phase 3** | **ROS 2 Jazzy Lifecycle Node** | `vapfl_ros`: `rclcpp_lifecycle::LifecycleNode`, parameter validation, `/scan` and `/odom` subscribers, `/amcl_pose` publisher, TF broadcaster. | Dynamic configure/activate lifecycle check, RViz2 visualization. |
| **Phase 4** | **CI/CD Automation** | `.github/workflows/ci.yml` and `lint.yml` executing formatting, strict build, and CTest. | Green CI status badge on pull requests. |
| **Phase 5** | **Isaac Sim Setup & Benchmarking** | Isaac Sim USD scene, ROS 2 bridge configuration, kidnap benchmark script comparing VAPFL vs Nav2 AMCL. | RMSE convergence plots and recovery time metrics. |
