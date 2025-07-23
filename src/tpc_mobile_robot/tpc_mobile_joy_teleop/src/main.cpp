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

 #include <rclcpp/rclcpp.hpp>
 #include "tpc_mobile_joy_teleop/tpc_mobile_arm_joy_messages.hpp"
 #include "tpc_mobile_joy_teleop/tpc_mobile_arm_joy_teleop.hpp"
 #include "tpc_mobile_joy_teleop/moveit_servo.hpp"
 #include <joy/joy.hpp>


 int main(int argc, char* argv[])
 {
      rclcpp::init(argc, argv);

      std::shared_ptr<rclcpp::Executor> executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

      //Moveit_Servoa moveit_servo_node;
      auto teleop_node = std::make_shared<tpc::tm5_joy::Tm5JoyTeleop>("teleop_node");
      //auto joystick_node = std::make_shared<tpc::tm5_joy::TpcJoyMessages>("joystick_node");
      auto joy_node = std::make_shared<joy::Joy>(rclcpp::NodeOptions());

      executor->add_node(teleop_node);
      //executor->add_node(joystick_node);
      executor->add_node(joy_node);
      executor->spin();

      rclcpp::shutdown();
      return 0;
 }
