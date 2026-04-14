cmake_minimum_required(VERSION 3.16)
project(fastslam_core)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)

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
  include/${PROJECT_NAME}/fastslam_oc_grid.hpp
  include/${PROJECT_NAME}/particle.hpp
  ${GRID_CONFIG_HEADER}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
  Sophus::Sophus
  Eigen3::Eigen
  TBB::tbb
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
ament_export_dependencies(Eigen3 Sophus)
ament_export_include_directories(include)

# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)

#   ament_add_gtest(test_fastslam_core
#     test/test_fastslam_core.cpp
#   )

#   target_link_libraries(test_fastslam_core
#     ${PROJECT_NAME}
#   )

# endif()

ament_package()