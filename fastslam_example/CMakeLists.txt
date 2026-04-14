cmake_minimum_required(VERSION 3.8)
project(fastslam_example)

find_package(ament_cmake REQUIRED)

# Instalar las carpetas para que ROS las mueva al directorio install/share
install(DIRECTORY 
  example
  benchmark
  RVIZFiles
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  benchmark/profile_fastslam_with_bagfile.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME profile_fastslam_with_bagfile
)

install(PROGRAMS
  benchmark/flamegraph
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()