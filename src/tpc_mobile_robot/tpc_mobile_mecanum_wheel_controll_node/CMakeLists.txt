cmake_minimum_required(VERSION 3.8)
project(tpc_mobile_mecanum_wheel_controll_node)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#add_compile_definitions(PLUGINLIB__DISABLE_BOOST_FUNCTIONS)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_manager REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(tpc_mobile_mecanum_wheel_controll_node
  src/tpc_mobile_mecanum_wheel_controll_node.cpp
)
include_directories(include)

ament_target_dependencies(
  tpc_mobile_mecanum_wheel_controll_node
  controller_manager
  realtime_tools
  rclcpp
)

install(TARGETS
  tpc_mobile_mecanum_wheel_controll_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
