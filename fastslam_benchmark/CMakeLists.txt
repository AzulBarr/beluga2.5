cmake_minimum_required(VERSION 3.8)
project(fastslam_benchmark)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
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