cmake_minimum_required(VERSION 3.0.2)
project(waypoint_navigation)


find_package(catkin REQUIRED
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  tf
  move_base
  move_base_msgs
  actionlib
  actionlib_msgs
  std_srvs
  dynamic_reconfigure
)


find_package(PkgConfig)
pkg_search_module(yaml-cpp REQUIRED yaml-cpp)
 
if(NOT ${yaml-cpp_VERSION} VERSION_LESS "0.5")
add_definitions(-DNEW_YAMLCPP)
endif()



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)



add_executable(waypoint_saver src/waypoint_saver.cpp)
target_link_libraries(waypoint_saver
  ${catkin_LIBRARIES}
)

add_executable(waypoint_nav src/waypoint_nav.cpp)
target_link_libraries(waypoint_nav
  ${catkin_LIBRARIES}
  ${yaml-cpp_LIBRARIES}
)

add_executable(velocity_controller src/velocity_controller.cpp)
target_link_libraries(velocity_controller
  ${catkin_LIBRARIES}
)


catkin_install_python(PROGRAMS
  scripts/tandem_run_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
