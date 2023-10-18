cmake_minimum_required(VERSION 3.8)
project(waypoint_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

add_executable(waypoint_nav src/waypoint_nav.cpp)
ament_target_dependencies(waypoint_nav
  rclcpp
  rclcpp_action
  geometry_msgs
  tf2_geometry_msgs
  nav2_msgs
  std_srvs
  yaml-cpp
)
target_link_libraries(waypoint_nav ${YAML_CPP_LIBRARIES})

add_executable(waypoint_saver src/waypoint_saver.cpp)
ament_target_dependencies(waypoint_saver
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  std_srvs
)


install(DIRECTORY launch waypoints_cfg rviz_cfg config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  waypoint_nav
  waypoint_saver
  DESTINATION lib/${PROJECT_NAME}/
)


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/tandem_run_manager.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
