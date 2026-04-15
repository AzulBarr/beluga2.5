cmake_minimum_required(VERSION 3.8)
project(fastslam_example)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  example
  RVIZFiles
  DESTINATION share/${PROJECT_NAME}
)

ament_package()