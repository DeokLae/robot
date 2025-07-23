cmake_minimum_required(VERSION 3.8)
project(single_drive)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ethercat_driver REQUIRED)
find_package(ethercat_interface REQUIRED)
find_package(ethercat_generic_cia402_drive REQUIRED)
find_package(ethercat_tmrobot_cia402_drive REQUIRED)
find_package(ethercat_generic_slave REQUIRED)

install(
  DIRECTORY config description launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
