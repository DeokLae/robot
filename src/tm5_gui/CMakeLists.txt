cmake_minimum_required(VERSION 3.8)
project(tm5_gui)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake" ${CMAKE_PREFIX_PATH})

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
#ros2 pkg
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

#tm5_moveit_config
find_package(tm5_bringup REQUIRED)
find_package(tm5_description REQUIRED)
find_package(tm5_moveit_config REQUIRED)
# Qt5
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${control_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_task_constructor_core_INCLUDE_DIRS}
  ${tm5_bringup_INCLUDE_DIRS}
  ${tm5_description_INCLUDE_DIRS}
  ${tm5_moveit_config_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

qt5_wrap_ui(UIC_FILES include/${PROJECT_NAME}/mainwindow.ui)

set(HEADERS
    include/${PROJECT_NAME}/mainwindow.hpp
    include/${PROJECT_NAME}/tpc_manipulation_moveit.hpp
    include/${PROJECT_NAME}/moveit_servo.hpp
)
set(SRCS
    src/main.cpp
    src/mainwindow.cpp
    src/tpc_manipulation_moveit.cpp
    src/moveit_servo.cpp
)

add_executable(${PROJECT_NAME} ${SRCS} ${HEADERS} ${UIC_FILES})

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tm5_bringup
  tm5_description
  tm5_moveit_config
  geometry_msgs
  nav_msgs
  control_msgs
  sensor_msgs
  std_srvs
  moveit_ros_planning_interface
  ament_index_cpp
)
target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Quick
  Qt5::Widgets
)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
