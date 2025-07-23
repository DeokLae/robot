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


#ifndef __TPC_MOBILE_JOY_TELEOP__TPC_MOBILE_ARM_JOY_TELEOP_H__
#define __TPC_MOBILE_JOY_TELEOP__TPC_MOBILE_ARM_JOY_TELEOP_H__
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
//#include "tpc_mobile_arm_joy_teleop/tpc_manipulation_moveit.hpp"

namespace tpc
{
namespace tm5_joy
{
    class Tm5JoyTeleop : public rclcpp::Node
    {
    public:
        struct AxisConfig
        {
          uint32_t axis;
          double scale;
          double offset;
          double deadzone;
        };

        struct ArmAixsConfig
        {
          uint32_t axis;
          double vel;
        };

        struct ButtonConfig
        {
          uint32_t num;
          std::string name;
        };

        Tm5JoyTeleop(const std::string& name);
        ~Tm5JoyTeleop();

    private:
      /* data */
      void receive_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg);
      double get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy>& msg, AxisConfig& config);
      int get_button_value(std::unique_ptr<sensor_msgs::msg::Joy>& msg, ButtonConfig& config);
      bool check_L1_L2_sate(std::unique_ptr<sensor_msgs::msg::Joy>& msg);
      bool check_axis_value_not_zero(std::unique_ptr<sensor_msgs::msg::Joy>& msg);
      void declare_parameter_value();
      void set_axis_scale();
      void set_mobile_sclae();
      void system_down();
      void check_button_value(std::unique_ptr<sensor_msgs::msg::Joy>& msg);

      //TpcManipulationMoveit::Manipulation manipulator;

      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr mobile_twist_pub_;
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
      rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
      //rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr arm_joint_pub_;

      AxisConfig move_forward_config_;
      AxisConfig move_back_config_;
      AxisConfig move_left_config_;
      AxisConfig move_right_config_;
      AxisConfig turn_left_config_;
      AxisConfig turn_right_config_;

      AxisConfig arm_xaxis_plus_config_;
      AxisConfig arm_xaxis_minus_config_;
      AxisConfig arm_yaxis_plus_config_;
      AxisConfig arm_yaxis_minus_config_;
      AxisConfig arm_zaxis_plus_config_;
      AxisConfig arm_zaxis_minus_config_;


      ButtonConfig btn_X_config_;
      ButtonConfig btn_A_config_;
      ButtonConfig btn_Y_config_;
      ButtonConfig btn_B_config_;
      ButtonConfig btn_PLUS_config_;
      ButtonConfig btn_L1_config_;
      ButtonConfig btn_L2_config_;
      ButtonConfig btn_MINUS_config_;
      ButtonConfig btn_HOME_config_;

      std::vector<AxisConfig> arm_axis_configs_;
      std::vector<double> pre_Axis_value_;
      std::vector<ButtonConfig> btn_configs_;
      std::vector<bool> pre_btn_value_;

      std::string frame_id_name_;
      double cure_axis_scale_;
      double max_axis_scale_;
      double min_axis_scale_;
      double cure_mobie_scale_;
      double max_mobile_scale_;
      double min_mobile_scale_;

      int shtudown_cnt_;
      std::mutex data_mutex;
    };
}
}

#endif
