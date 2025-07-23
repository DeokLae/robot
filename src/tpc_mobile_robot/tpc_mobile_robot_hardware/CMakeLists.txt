cmake_minimum_required(VERSION 3.8)
project(tpc_mobile_robot_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



include_directories(include)

add_executable(tpc_mobile_robot_hardware_node src/main.cpp)

ament_target_dependencies(tpc_mobile_robot_hardware_node
  rclcpp
  serial
  hardware_interface
  pluginlib
  nav_msgs
  tf2
  tf2_geometry_msgs
)


add_library(${PROJECT_NAME} SHARED
  src/zlac8015d.cpp
  src/tpc_mobile_robot_hardware.cpp
)
#ament_export_targets(zlac8015d HAS_LIBRARY_TARGET)
ament_target_dependencies(${PROJECT_NAME}
  serial
  hardware_interface
  pluginlib
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(hardware_interface tpc_mobile_robot_hardware.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  tpc_mobile_robot_hardware_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  nav_msgs
  tf2
  tf2_geometry_msgs
)

ament_package()
