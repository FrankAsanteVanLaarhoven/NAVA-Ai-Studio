cmake_minimum_required(VERSION 3.8)
project(univarm_kobuki_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

install(DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME})

add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower rclcpp nav_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS path_follower
  DESTINATION lib/${PROJECT_NAME})

ament_package()
