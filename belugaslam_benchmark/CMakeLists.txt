cmake_minimum_required(VERSION 3.8)
project(belugaslam_benchmark)

find_package(ament_cmake REQUIRED)

install(DIRECTORY 
  benchmarking
  profiling
  #RVIZFiles
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  profiling/profile_belugaslam_with_bagfile.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME profile_belugaslam_with_bagfile
)

install(PROGRAMS
  profiling/flamegraph
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  benchmarking/parameterized_run.sh
  benchmarking/compare_results.py
  benchmarking/compute_rmse.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()