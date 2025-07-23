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

#include "tpc_mobile_mecanum_wheel_controll_node/tpc_mobile_mecanum_wheel_controll_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace tpc
{
namespace tpc_mobile_mecanum_wheel_controll_node
{
  TpcMobileMecanumWheelControllNode::TpcMobileMecanumWheelControllNode() : Node("TpcMobileMecanumWheelControllNode"), update_rate_(50)
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    controller_manager_node_ = std::make_shared<controller_manager::ControllerManager>(executor_, "controller_manager");

    update_rate_ = controller_manager_node_->get_update_rate();
    RCLCPP_INFO(this->get_logger(), "Controller manager update rate: %d Hz", update_rate_);

    cm_thread_ = std::thread(&TpcMobileMecanumWheelControllNode::run_controller_manager, this);

    load_and_configure_controllers();
  }

  TpcMobileMecanumWheelControllNode::~TpcMobileMecanumWheelControllNode()
  {
      if(cm_thread_.joinable())
      {
        cm_thread_.join();
      }
  }

  void TpcMobileMecanumWheelControllNode::run_controller_manager()
  {
    //check if real-time kerner is available
    if(realtime_tools::has_realtime_kernel())
    {
        if(!realtime_tools::configure_sched_fifo(kSchedPriority))
        {
          RCLCPP_WARN(this->get_logger(), "Could not enable FIFO RT scheduling policyrrrrr");
        }
    } else
    {
        RCLCPP_INFO(this->get_logger(), "Real-time kernel is recommended for better perfomance");
    }

    //Define time period for controller updates
    auto period = std::chrono::nanoseconds(1'000'000'000 / update_rate_);
    auto cm_now = std::chrono::nanoseconds(controller_manager_node_->now().nanoseconds());
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{cm_now};

    rclcpp::Time previous_time  = controller_manager_node_->now();

    while (rclcpp::ok())
    {
      /* code */
      auto current_time = controller_manager_node_->now();
      auto measured_period = current_time - previous_time;
      previous_time = current_time;

      controller_manager_node_->read(current_time, measured_period);
      controller_manager_node_->update(current_time, measured_period);
      controller_manager_node_->write(current_time, measured_period);

      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);
    }

  }

  void TpcMobileMecanumWheelControllNode::load_and_configure_controllers()
  {
    RCLCPP_WARN(this->get_logger(), "load and configure_controllers by mecanum controll node!!");
    std::vector<std::string> start_controllers;
    std::vector<std::string> stop_controllers;

    // Load controllers
    controller_manager_node_->load_controller("joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster");
    controller_manager_node_->load_controller("mecanum_drive_controller", "tpc_mobile_mecanum_wheel_controller/TpcMobileMecanumController");

    // Configure controllers
    controller_manager_node_->configure_controller("joint_state_broadcaster");
    controller_manager_node_->configure_controller("mecanum_drive_controller");

    //Start the controllers
    start_controllers.push_back("joint_state_broadcaster");
    start_controllers.push_back("mecanum_drive_controller");
    controller_manager_node_->switch_controller(start_controllers, stop_controllers, 1, controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT);
  }
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<tpc::tpc_mobile_mecanum_wheel_controll_node::TpcMobileMecanumWheelControllNode>();
  RCLCPP_ERROR(node->get_logger(), "mecanum coltroller node start?????");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
