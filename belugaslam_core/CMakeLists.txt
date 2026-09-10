cmake_minimum_required(VERSION 3.16)
project(belugaslam_core)

# Most SLAM code is header-only and is compiled again into the ROS executable.
# An empty build type in either package leaves these hot paths unoptimized.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
find_package(Ceres REQUIRED)

set(GRID_CONFIG_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/config/grid_config.py")
set(GRID_CONFIG_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/grid_config.hpp")
add_custom_command(
  OUTPUT "${GRID_CONFIG_HEADER}"
  COMMAND python3 "${GRID_CONFIG_SCRIPT}" "${GRID_CONFIG_HEADER}"
  DEPENDS "${GRID_CONFIG_SCRIPT}"
  COMMENT "Generating grid_config.hpp from Python script"
)

add_library(${PROJECT_NAME} SHARED
  src/fastslam_oc_grid_core.cpp
  ${GRID_CONFIG_HEADER}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME}
  Sophus::Sophus
  Eigen3::Eigen
  TBB::tbb
  Ceres::ceres
)

ament_target_dependencies(${PROJECT_NAME}
  ament_cmake
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Sophus TBB Ceres)
ament_export_include_directories(include)

# Offline replay calls the SAME BelugaSLAM class as the ROS node, with no TF,
# message queues or reference poses. It is useful for separating numerical
# regressions from delivery/timing failures at large particle counts.
add_executable(intel_accuracy_replay ../tools/intel_accuracy_replay.cpp)
target_link_libraries(intel_accuracy_replay ${PROJECT_NAME})
target_compile_features(intel_accuracy_replay PRIVATE cxx_std_17)
install(TARGETS intel_accuracy_replay DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(tracking_accuracy_test test/tracking_accuracy_test.cpp)
  target_include_directories(tracking_accuracy_test PRIVATE include)
  target_compile_features(tracking_accuracy_test PRIVATE cxx_std_17)
  add_test(NAME tracking_accuracy_test COMMAND tracking_accuracy_test)
  ament_add_gtest(accuracy_regression_test test/accuracy_regression_test.cpp)
  target_link_libraries(accuracy_regression_test ${PROJECT_NAME})
  ament_add_gtest(submap_graph_test test/submap_graph_test.cpp)
  target_include_directories(submap_graph_test PRIVATE include)
  target_link_libraries(submap_graph_test Sophus::Sophus Eigen3::Eigen TBB::tbb)
  target_compile_features(submap_graph_test PRIVATE cxx_std_17)
  ament_add_gtest(loop_refinement_test test/loop_refinement_test.cpp)
  target_include_directories(loop_refinement_test PRIVATE include)
  target_link_libraries(loop_refinement_test ${PROJECT_NAME})
  ament_add_gtest(submap_tracking_test test/submap_tracking_test.cpp)
  target_include_directories(submap_tracking_test PRIVATE include)
  target_link_libraries(submap_tracking_test ${PROJECT_NAME})
  add_executable(motion_filter_test test/motion_filter_test.cpp)
  target_include_directories(motion_filter_test PRIVATE include)
  target_compile_features(motion_filter_test PRIVATE cxx_std_17)
  add_test(NAME motion_filter_test COMMAND motion_filter_test)
  add_executable(loop_belief_test test/loop_belief_test.cpp)
  target_include_directories(loop_belief_test PRIVATE include)
  target_compile_features(loop_belief_test PRIVATE cxx_std_17)
  add_test(NAME loop_belief_test COMMAND loop_belief_test)
  add_executable(output_selection_test test/output_selection_test.cpp)
  target_include_directories(output_selection_test PRIVATE include)
  target_compile_features(output_selection_test PRIVATE cxx_std_17)
  add_test(NAME output_selection_test COMMAND output_selection_test)
  add_executable(pose_graph_residual_test test/pose_graph_residual_test.cpp)
  target_include_directories(pose_graph_residual_test PRIVATE include)
  target_compile_features(pose_graph_residual_test PRIVATE cxx_std_17)
  add_test(NAME pose_graph_residual_test COMMAND pose_graph_residual_test)
  add_executable(proposal_pose_test test/proposal_pose_test.cpp)
  target_include_directories(proposal_pose_test PRIVATE include)
  target_compile_features(proposal_pose_test PRIVATE cxx_std_17)
  add_test(NAME proposal_pose_test COMMAND proposal_pose_test)
  add_executable(robust_tracking_test test/robust_tracking_test.cpp)
  target_include_directories(robust_tracking_test PRIVATE include)
  target_compile_features(robust_tracking_test PRIVATE cxx_std_17)
  add_test(NAME robust_tracking_test COMMAND robust_tracking_test)
  find_package(Threads REQUIRED)
  add_executable(late_run_test test/late_run_test.cpp)
  target_include_directories(late_run_test PRIVATE include)
  target_compile_features(late_run_test PRIVATE cxx_std_17)
  target_link_libraries(late_run_test Threads::Threads)
  add_test(NAME late_run_test COMMAND late_run_test)
  ament_add_gtest(late_run_integration_test test/late_run_integration_test.cpp)
  target_include_directories(late_run_integration_test PRIVATE include)
  target_link_libraries(late_run_integration_test ${PROJECT_NAME})
  ament_add_gtest(quality_integration_test test/quality_integration_test.cpp)
  target_include_directories(quality_integration_test PRIVATE include)
  target_link_libraries(quality_integration_test ${PROJECT_NAME})
  add_executable(grid_update_test test/grid_update_test.cpp)
  target_include_directories(grid_update_test PRIVATE include)
  target_compile_features(grid_update_test PRIVATE cxx_std_17)
  add_test(NAME grid_update_test COMMAND grid_update_test)
  ament_add_gtest(performance_regression_test test/performance_regression_test.cpp)
  target_include_directories(performance_regression_test PRIVATE include)
  target_link_libraries(performance_regression_test ${PROJECT_NAME})
  ament_add_gtest(loop_pgo_test test/loop_pgo_test.cpp)
  target_include_directories(loop_pgo_test PRIVATE include)
  target_link_libraries(loop_pgo_test ${PROJECT_NAME})
endif()

ament_package()
