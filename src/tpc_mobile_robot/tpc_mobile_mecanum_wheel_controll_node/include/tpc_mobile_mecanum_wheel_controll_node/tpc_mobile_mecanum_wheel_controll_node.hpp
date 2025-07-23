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


#ifndef __TPC_MOBILE_MECANUM_WHEEL_CONTROLL_NODE__TPC_MOBILE_MECANUM_WHEEL_CONTROLL_NODE_H__
#define __TPC_MOBILE_MECANUM_WHEEL_CONTROLL_NODE__TPC_MOBILE_MECANUM_WHEEL_CONTROLL_NODE_H__

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>
#include <realtime_tools/thread_priority.hpp>
#include <thread>
#include <vector>

namespace tpc
{
namespace tpc_mobile_mecanum_wheel_controll_node
{
  class TpcMobileMecanumWheelControllNode : public rclcpp::Node
  {
  private:
    void run_controller_manager();
    void load_and_configure_controllers();

    static constexpr int kSchedPriority = 50;

    std::shared_ptr<rclcpp::Executor> executor_;
    std::shared_ptr<controller_manager::ControllerManager> controller_manager_node_;
    int update_rate_;
    std::thread cm_thread_;

  public:
  TpcMobileMecanumWheelControllNode();
  ~TpcMobileMecanumWheelControllNode();
  };
}
}

#endif
