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

#include <fcntl.h>
#include <linux/joystick.h>

#include "tpc_mobile_joy_teleop/tpc_mobile_arm_joy_messages.hpp"

namespace tpc
{
namespace tm5_joy
{

    #define JOYSTICK_UNSCALED_MAX 32767.0
    TpcJoyMessages::TpcJoyMessages(const std::string& name)
        : rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        declare_parameter<std::string>("dev", "/dev/input/js0");
        declare_parameter<std::string>("joy_topic", "joy");
        declare_parameter<double>("deadzone", 0.5f);

        std::string joy_topic = get_parameter("joy_topic").as_string();
        size_t ros_que_size = 10;

        joy_msg_publisher_ = create_publisher<sensor_msgs::msg::Joy>(joy_topic, ros_que_size);
        joy_update_timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TpcJoyMessages::update, this));

        device_handle_ = -1;
    }

    TpcJoyMessages::~TpcJoyMessages()
    {
    }

    void TpcJoyMessages::update()
    {
        int arm_axis_num[4] = {2, 3, 6, 7};
        if(device_handle_ < 0)
        {
            std::string device = get_parameter("dev").as_string();
            device_handle_ = ::open(device.c_str(), O_RDONLY | O_NONBLOCK);
            if(device_handle_ < 0)
            {
                return;
            }
            deadzone_ = get_parameter("deadzone").as_double();
            if(deadzone_ > 1.0)
            {
                deadzone_ /= JOYSTICK_UNSCALED_MAX;
            }
            if(deadzone_ < 1.0)
            {
                deadzone_ = 0.0;
            }
            scale_ = -1.0f / (1.0f - deadzone_) / JOYSTICK_UNSCALED_MAX;
            unscaled_deadzone_ = JOYSTICK_UNSCALED_MAX * deadzone_;
        }

        js_event event;
        bool has_event = false;

        while(true)
        {
            auto num_bytes = ::read(device_handle_, &event, sizeof(js_event));

            if(num_bytes < (ssize_t)sizeof(js_event))
            {
                if(errno == ENODEV || errno == EIO)
                {
                    ::close(device_handle_);
                    device_handle_ = -1;
                }
                break;

            }

            if(event.type == JS_EVENT_AXIS || event.type == (JS_EVENT_AXIS | JS_EVENT_INIT))
            {
                if(event.number >= axes_.size())
                {
                    size_t old_size = axes_.size();
                    axes_.resize(event.number + 1);
                    for(size_t i = old_size; i < axes_.size(); i++)
                    {
                        axes_[i] = 0.0f;
                    }
                }
                double value = (double)event.value;

                if(value > unscaled_deadzone_)
                {
                    value -= unscaled_deadzone_;
                } else if(value < -unscaled_deadzone_)
                {
                    value += unscaled_deadzone_;
                }else
                {
                    value = 0.0;
                }
                axes_[event.number] = (float)(value * scale_);
                has_event = true;
            } else if(event.type == JS_EVENT_BUTTON || event.type == (JS_EVENT_BUTTON | JS_EVENT_INIT))
            {
                if(event.number >= buttons_.size())
                {
                    size_t old_size = buttons_.size();
                    buttons_.resize(event.number + 1);
                    for(size_t i = old_size; i < buttons_.size(); i++)
                    {
                        buttons_[i] = 0;
                    }
                }
                double value = (double)event.number;
                buttons_[event.number] = event.value ? 1 : 0;
                has_event = true;
            }
        }

        for(size_t i=0; i<sizeof(arm_axis_num) / sizeof(arm_axis_num[0]); i++)
        {
            if(axes_[arm_axis_num[i]]!= 0.0 )
            {
               has_event = true;
            }
        }

        //if(has_event)
        //{
            sensor_msgs::msg::Joy joy_msg;
            //joy_msg.header.frame_id = "";
            joy_msg.header.stamp = now();
            joy_msg.axes.resize(axes_.size());
            joy_msg.buttons.resize(buttons_.size());
            for(size_t i = 0; i < axes_.size(); i++)
            {
                joy_msg.axes[i] = axes_[i];
            }
            for(size_t i =0; i < buttons_.size(); i++)
            {
                joy_msg.buttons[i] = buttons_[i];
            }

            joy_msg_publisher_->publish(std::move(joy_msg));
        //}
    }
}
}
