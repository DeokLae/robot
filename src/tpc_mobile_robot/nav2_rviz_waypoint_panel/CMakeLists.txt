cmake_minimum_required(VERSION 3.8)
project(nav2_rviz_waypoint_panel)

set(CMAKE_CXX_STANDARD 14)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(WaypointPanel SHARED
  src/waypoint_panel.cpp
)


ament_target_dependencies(WaypointPanel
  rclcpp
  tf2
  tf2_geometry_msgs
  rviz_common
  pluginlib
  nav2_msgs
  geometry_msgs
  std_msgs
  yaml-cpp
)

target_link_libraries(WaypointPanel
  ${rclcpp_LIBRARIES}
)
pluginlib_export_plugin_description_file(rviz_common plugins.xml)

install(TARGETS WaypointPanel
  LIBRARY DESTINATION lib
)

install(FILES
  plugins.xml
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
