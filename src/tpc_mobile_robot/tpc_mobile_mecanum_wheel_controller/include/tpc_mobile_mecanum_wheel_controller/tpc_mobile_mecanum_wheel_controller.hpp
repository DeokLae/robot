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
#ifndef __TPC_MOBILE_MECANUM_WHEEL_CONTROLLER__TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_H__
#define __TPC_MOBILE_MECANUM_WHEEL_CONTROLLER__TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_H__

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <string>

#include "tpc_mobile_mecanum_wheel_controller/tpc_mobile_mecanum_wheel_compiler.h"
#include "tpc_mobile_mecanum_wheel_controller/mecanum_wheel.hpp"

namespace tpc
{
namespace tpc_mobile_mecanum_wheel_controller
{
    using Twist = geometry_msgs::msg::Twist;
    class TpcMobileMecanumController : public controller_interface::ControllerInterface
    {
    protected:
      std::shared_ptr<MecanumWheel> get_wheel(const std::string & wheel_joint_name);
      bool reset();

    protected:
      /* data */
      rclcpp::Subscription<Twist>::SharedPtr command_velocity_subscription_;
      realtime_tools::RealtimeBuffer<std::shared_ptr<Twist>> command_velocity_ptr_;
      std::shared_ptr<MecanumWheel> fl_wheel_;
      std::shared_ptr<MecanumWheel> fr_wheel_;
      std::shared_ptr<MecanumWheel> rl_wheel_;
      std::shared_ptr<MecanumWheel> rr_wheel_;
      std::string fl_wheel_joint_name_;
      std::string fr_wheel_joint_name_;
      std::string rl_wheel_joint_name_;
      std::string rr_wheel_joint_name_;
      double linear_scale_;
      double radial_scale_;
      double wheel_radius_;
      double wheel_distance_width_;
      double wheel_distance_length_;
      double wheel_separation_width_;
      double wheel_separation_length_;
      bool subscriber_is_actve_;

    public:
      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      TpcMobileMecanumController();
      ~TpcMobileMecanumController();

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::CallbackReturn on_init() override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;

      TPC_MOBILE_MECANUM_WHEEL_CONTROLLER_PUBLIC
      controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;

    };

}
}
#endif
