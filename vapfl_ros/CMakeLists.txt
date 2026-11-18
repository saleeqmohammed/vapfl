cmake_minimum_required(VERSION 3.20)
project(vapfl_ros LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Node Library
add_library(vapfl_ros
  src/vapfl_node.cpp
)

target_include_directories(vapfl_ros
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(vapfl_ros PUBLIC cxx_std_20)

target_link_libraries(vapfl_ros
  PUBLIC
    vapfl_core
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${tf2_TARGETS}
    ${tf2_ros_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
)

if(COMMAND vapfl_set_compiler_warnings)
  vapfl_set_compiler_warnings(vapfl_ros)
endif()

# Main Executable
add_executable(vapfl_node
  src/main.cpp
)

target_link_libraries(vapfl_node
  PRIVATE
    vapfl_ros
)

target_compile_features(vapfl_node PRIVATE cxx_std_20)

if(COMMAND vapfl_set_compiler_warnings)
  vapfl_set_compiler_warnings(vapfl_node)
endif()

# Installation
install(TARGETS vapfl_ros vapfl_node
  EXPORT export_vapfl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ros_conversions test/test_conversions.cpp)
  target_link_libraries(test_ros_conversions vapfl_ros)
  if(COMMAND vapfl_set_compiler_warnings)
    vapfl_set_compiler_warnings(test_ros_conversions)
  endif()
endif()
