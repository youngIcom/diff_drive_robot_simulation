cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# C++ Executable: Obstacle Detector and Image Capture
add_executable(obstacle_detector_capture src/obstacle_detector_capture.cpp)
ament_target_dependencies(obstacle_detector_capture
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  cv_bridge
  OpenCV
)

# C++ Executable: Buoy Gate Detection and Visual Centering
add_executable(buoy_centering src/buoy_centering.cpp)
ament_target_dependencies(buoy_centering
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  OpenCV
)

install(TARGETS
  obstacle_detector_capture
  buoy_centering
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/waypoint_logger.py
  scripts/waypoint_follower.py
  scripts/path_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/path_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME path_visualizer
)

install(PROGRAMS scripts/waypoint_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_logger
)

install(PROGRAMS scripts/waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_follower
)

ament_package()

