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

#ifndef __TPC_MOBILE_JOY_TELEOP__TPC_MOBILE_ARM_JOY_MESSAGES_H__
#define __TPC_MOBILE_JOY_TELEOP__TPC_MOBILE_ARM_JOY_MESSAGES_H__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <vector>


namespace tpc
{
namespace tm5_joy
{
    class TpcJoyMessages : public rclcpp::Node
    {
    public:
      TpcJoyMessages(const std::string& name);
      ~TpcJoyMessages();

    private:
      /* data */
      void update();
      void message_pub();

      rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_msg_publisher_;
      rclcpp::TimerBase::SharedPtr joy_update_timer_;
      std::vector<int> buttons_;
      std::vector<double> axes_;
      int device_handle_;
      double deadzone_;
      double scale_;
      double unscaled_deadzone_;
    };

}
}

#endif
