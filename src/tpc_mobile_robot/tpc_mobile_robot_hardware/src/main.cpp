#include "rclcpp/rclcpp.hpp"
#include "tpc_mobile_robot_hardware/tpc_mobile_robot_hardware.hpp"
#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/duration.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
        // 플러그인 로더 생성
    pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
      "hardware_interface", "hardware_interface::SystemInterface");

    // 하드웨어 인터페이스 로드
    auto hardware_interface = loader.createSharedInstance("tpc_mobile_robot_hardware/TpcMobileRobotSystemHardware");
    if (!hardware_interface)
    {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to load hardware interface");
      return 1;
    }

    // ROS 2 노드 생성
    auto node = rclcpp::Node::make_shared("tpc_mobile_robot_hardware_node");

    // 하드웨어 인터페이스 초기화
    hardware_interface->on_init(hardware_interface::HardwareInfo());

    // 하드웨어 인터페이스의 상태 및 명령 인터페이스 등록
    auto state_interfaces = hardware_interface->export_state_interfaces();
    auto command_interfaces = hardware_interface->export_command_interfaces();

    // 상태 및 명령 인터페이스를 통해 데이터를 주고 받는 로직 추가

    // 하드웨어 인터페이스 활성화
    hardware_interface->on_activate(rclcpp_lifecycle::State());

    // ROS 2 노드 실행 (주기적으로 read 및 write 호출)
    rclcpp::Rate rate(10);  // 10Hz로 주기적으로 호출
    while (rclcpp::ok())
    {
      // 상태 읽기
      hardware_interface->read(rclcpp::Clock().now(), rclcpp::Duration(0, 1000000000));

      // 명령 쓰기
      hardware_interface->write(rclcpp::Clock().now(), rclcpp::Duration(0, 1000000000));

      rclcpp::spin_some(node);
      rate.sleep();
    }

    // 하드웨어 인터페이스 비활성화
    hardware_interface->on_deactivate(rclcpp_lifecycle::State());

    rclcpp::shutdown();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception caught: %s", e.what());
    return 1;
  }

  return 0;
}
