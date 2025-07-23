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

#include "tpc_mobile_joy_teleop/moveit_servo.hpp"


Moveit_Servoa::Moveit_Servoa()
{
  nh_ = std::make_shared<rclcpp::Node>("tpc_mobile_arm_servo_node");
  servo_start_client_ = nh_->create_client<std_srvs::srv::Trigger>("servo_node/start_servo");
  servo_stop_client_ = nh_->create_client<std_srvs::srv::Trigger>("servo_node/stop_servo");

  std::thread(std::bind(&Moveit_Servoa::spin, this)).detach();
  connect_moveit_servo();
  start_moveit_servo();
}

Moveit_Servoa::~Moveit_Servoa()
{
    stop_moveit_servo();
}

void Moveit_Servoa::connect_moveit_servo()
{
  for(int i =0; i <= 10; ++i)
  {
      if(servo_start_client_->wait_for_service(std::chrono::seconds(1)))
      {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "success to connect servo_start server ");
          break;
      }
      RCLCPP_WARN_STREAM(nh_->get_logger(), "wait to connect servo_start server");
      if(i == 10)
      {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "fail to connect servo_start server" << "please launch 'servo.lauch'");
      }
  }

  for(int i = 0; i <= 10; ++i)
  {
      if(servo_stop_client_->wait_for_service(std::chrono::seconds(1)))
      {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "success to connect servo_stop server");
          break;
      }
      RCLCPP_WARN_STREAM(nh_->get_logger(), "wait to connect servo_stop server");
      if(i == 10)
      {
          RCLCPP_ERROR_STREAM(nh_->get_logger(), "fail to connect servo_stop server" << "please launch 'servo.launch'");
      }
  }
}

void Moveit_Servoa::start_moveit_servo()
{
    RCLCPP_INFO_STREAM(nh_->get_logger(), "call moveit_servo start srv");
    auto future = servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    auto result = future.wait_for(std::chrono::seconds(1));
    if(result == std::future_status::ready)
    {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "success to start moveit_servo");
        future.get();
    }else
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "fail to start moveit_servo, restart with moveit_servo");
    }
}

void Moveit_Servoa::stop_moveit_servo()
{
    RCLCPP_INFO_STREAM(nh_->get_logger(), "call moveit_servo stop srv");
    auto future = servo_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    auto result = future.wait_for(std::chrono::seconds(1));
    if(result == std::future_status::ready)
    {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "success to stop moveit_servo");
        future.get();
    }
}

void Moveit_Servoa::spin()
{
    while(rclcpp::ok())
    {
        rclcpp::spin_some(nh_);
    }
}
