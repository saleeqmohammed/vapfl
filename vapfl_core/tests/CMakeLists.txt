cmake_minimum_required(VERSION 3.20)

find_package(GTest REQUIRED)
include(GoogleTest)

function(vapfl_add_core_test test_name source_file)
  add_executable(${test_name} ${source_file})
  target_link_libraries(${test_name}
    PRIVATE
      vapfl_core
      GTest::gtest
      GTest::gtest_main
  )
  target_compile_features(${test_name} PRIVATE cxx_std_20)

  if(COMMAND vapfl_set_compiler_warnings)
    vapfl_set_compiler_warnings(${test_name})
  endif()

  gtest_discover_tests(${test_name})
endfunction()

vapfl_add_core_test(test_motion_model test_motion_model.cpp)
vapfl_add_core_test(test_sensor_model test_sensor_model.cpp)
vapfl_add_core_test(test_adaptive_sampler test_adaptive_sampler.cpp)
vapfl_add_core_test(test_dbscan test_dbscan.cpp)
vapfl_add_core_test(test_particle_filter test_particle_filter.cpp)
