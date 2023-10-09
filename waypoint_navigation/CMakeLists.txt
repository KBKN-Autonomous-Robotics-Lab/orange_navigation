cmake_minimum_required(VERSION 3.8)
project(waypoint_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

add_executable(waypoint_nav src/waypoint_nav.cpp)
ament_target_dependencies(waypoint_nav
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  yaml-cpp
)
target_link_libraries(waypoint_nav ${YAML_CPP_LIBRARIES})


install(DIRECTORY launch waypoints_cfg
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  waypoint_nav
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
