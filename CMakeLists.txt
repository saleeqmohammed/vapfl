cmake_minimum_required(VERSION 3.20)
project(vapfl LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(VAPFL_ENABLE_SANITIZERS "Enable ASan and UBSan" OFF)
option(VAPFL_ENABLE_WERROR "Treat warnings as errors" ON)
option(VAPFL_ENABLE_PEDANTIC_WARNINGS "Enable pedantic compiler warnings" ON)
option(BUILD_TESTING "Build tests" ON)

if(VAPFL_ENABLE_SANITIZERS)
  message(STATUS "Enabling AddressSanitizer & UndefinedBehaviorSanitizer")
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

# Include compiler warnings helper
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/CompilerWarnings.cmake)

# Find ROS 2 build system
find_package(ament_cmake REQUIRED)

# Subdirectories
add_subdirectory(vapfl_core)
add_subdirectory(vapfl_ros)

ament_package()
