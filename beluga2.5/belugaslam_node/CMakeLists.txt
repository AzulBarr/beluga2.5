cmake_minimum_required(VERSION 3.8)
project(belugaslam_node)

# Most SLAM code is header-only and is compiled again into the ROS executable.
# An empty build type in either package leaves these hot paths unoptimized.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
#################################### AGREGO ####################################
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(TBB REQUIRED)

find_package(belugaslam_core REQUIRED)

include_directories(
  include
)
################################################################################

add_executable( ################################################################
  belugaslam_node 
  src/belugaslam_node.cpp
  src/fastslam_oc_grid_node.cpp 
)

target_link_libraries(belugaslam_node ############################################
    belugaslam_core::belugaslam_core
    TBB::tbb
)

target_include_directories(belugaslam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_compile_features(belugaslam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(belugaslam_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  visualization_msgs
  belugaslam_core
)

install(
  TARGETS belugaslam_node
  DESTINATION lib/${PROJECT_NAME}
)

install( ########################################################################
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
