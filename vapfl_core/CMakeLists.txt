cmake_minimum_required(VERSION 3.20)
project(vapfl_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Include warnings helper if not already included
if(NOT COMMAND vapfl_set_compiler_warnings)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/CompilerWarnings.cmake OPTIONAL)
endif()

add_library(vapfl_core
  src/motion_model.cpp
  src/sensor_model.cpp
  src/adaptive_sampler.cpp
  src/dbscan.cpp
  src/particle_filter.cpp
)

target_include_directories(vapfl_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_compile_features(vapfl_core PUBLIC cxx_std_20)

if(COMMAND vapfl_set_compiler_warnings)
  vapfl_set_compiler_warnings(vapfl_core)
endif()

if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()
