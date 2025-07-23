/*********************************************************************
 * Software License Agreement (Apache License, Version 2.0 )
 *
 * Copyright 2024 TPC Mechatronics Crop.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

 /* Author: DeokLae Kim (kdl79@tanhay.com)*/

#ifndef TPC_MOBILE_ROBOT_HARDWARE__TPC_MOBILE_ROBOT_HARDWARE_HPP_
#define TPC_MOBILE_ROBOT_HARDWARE__TPC_MOBILE_ROBOT_HARDWARE_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "tpc_mobile_robot_hardware/zlac8015d.h"
#include "tpc_mobile_robot_hardware/tpc_mobile_robot_hardware_definition.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/twist.hpp>


namespace tpc
{
namespace tpc_mobile_robot_hardware
{
  class TpcMobileRobotSystemHardware : public hardware_interface::SystemInterface
  {
    public:
    TpcMobileRobotSystemHardware() = default;
    //virtual ~TpcMobileRobotSystemHardware() = default;
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

    rclcpp::Node::SharedPtr odom_pub_node_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    //nav_msgs::msg::Odometry odom_msg_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    rclcpp::executors::SingleThreadedExecutor executor;

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //geometry_msgs::msg::TransformStamped odom_tf_;

    std::unique_ptr<ZLAC> motorFront_;
    //std::unique_ptr<ZLAC> motorRear_;

    MOT_DATA motorData_Front_;
    //MOT_DATA motorData_Rear_;

    private:
    size_t motor_cnt_ = 2;
    std::vector<double> wheel_positions_{0.0, 0.0};
    std::vector<double> wheel_velocities_{0.0, 0.0};
    std::vector<double> wheel_command_velocities_{0.0, 0.0};
    void calculate_odometry();

    //void publish_tf(const rclcpp::Time &current_time);
    geometry_msgs::msg::Quaternion imu_orientation_;
    geometry_msgs::msg::Vector3 imu_angular_velocity_;
    geometry_msgs::msg::Vector3 imu_linear_acceleration_;

    double x_pos_, y_pos_, theta_, roll_, pich_;
    double init_motor_encoder_L,init_motor_encoder_R,mean_rot_dist_old;

    double vx_;
    double vy_;
    double omega_z_;

    double delta_x, delta_y, delta_theta;
    double pre_x, pre_y, pre_theta;


    rclcpp::Time last_time_;
    rclcpp::Time current_time_;
    double dt;

    void publish_odometry();
  };

} // namespace tpc_mobile_robot_hardware end
} //namespace tpc end
#endif
