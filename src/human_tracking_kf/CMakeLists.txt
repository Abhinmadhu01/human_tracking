cmake_minimum_required(VERSION 3.8)
project(human_tracking_kf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/kalman_tracker.py
  src/person_tracking.py
  src/bytetrack_tracker.py
  DESTINATION lib/${PROJECT_NAME}

)


install(DIRECTORY launch config 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()