cmake_minimum_required(VERSION 3.8)
project(rviz2_plugins)

set(CMAKE_AUTOMOC ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(class_loader REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)


include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/state_trigger_panel.cpp
  src/set_finish_panel.cpp
  include/${PROJECT_NAME}/state_trigger_panel.h
  include/${PROJECT_NAME}/set_finish_panel.h
  ${MOC_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_default_plugins
  class_loader
  std_srvs
  Qt5
)

target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)


install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_export_libraries(${PROJECT_NAME})

ament_package()
