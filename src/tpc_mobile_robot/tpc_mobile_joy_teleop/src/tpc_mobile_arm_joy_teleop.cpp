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
#include "tpc_mobile_joy_teleop/tpc_mobile_arm_joy_teleop.hpp"
namespace tpc
{
namespace tm5_joy
{
    Tm5JoyTeleop::Tm5JoyTeleop(const std::string& name): rclcpp::Node(name, rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        declare_parameter_value();
        arm_axis_configs_ = {arm_xaxis_plus_config_, arm_xaxis_minus_config_,
                             arm_yaxis_plus_config_, arm_yaxis_minus_config_,
                             arm_zaxis_plus_config_, arm_zaxis_minus_config_};

        pre_Axis_value_.resize(sizeof(arm_axis_configs_));
        for(size_t i = 0; i < sizeof(arm_axis_configs_); i++)
        {
          pre_Axis_value_[i] = 0.0f;
        }

        btn_configs_ = {btn_A_config_, btn_B_config_, btn_X_config_, btn_Y_config_,
                        btn_L1_config_, btn_L2_config_, btn_HOME_config_,
                        btn_PLUS_config_, btn_MINUS_config_};
        pre_btn_value_.resize(sizeof(btn_configs_));
        for(size_t i = 0; i < sizeof(btn_configs_); i ++)
        {
            pre_btn_value_[i] = false;
        }

        std::string mobile_twist_topic = get_parameter("mobile_twist_topic").as_string();
        mobile_twist_pub_ = create_publisher<geometry_msgs::msg::Twist>(mobile_twist_topic, 10);

        std::string joy_topic =  get_parameter("joy_topic").as_string();
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            joy_topic, 10, std::bind(&Tm5JoyTeleop::receive_joy_message, this, std::placeholders::_1));

        std::string arm_twist_topic = get_parameter("arm_twist_topic").as_string();
        arm_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(arm_twist_topic, 10);

        frame_id_name_ = "link_0";
        shtudown_cnt_ = 0;
    }
    Tm5JoyTeleop::~Tm5JoyTeleop()
    {

    }

    void Tm5JoyTeleop::receive_joy_message(std::unique_ptr<sensor_msgs::msg::Joy> msg)
    {
        bool has_event = false;

        geometry_msgs::msg::Twist::UniquePtr mobile_twist_message(new geometry_msgs::msg::Twist());
        //mobile_twist_message->header.frame_id = "twist";
        //mobile_twist_message->header.stamp = now();
        mobile_twist_message->linear.x = get_axis_value(msg, move_forward_config_) + get_axis_value(msg, move_back_config_);
        mobile_twist_message->linear.y = get_axis_value(msg, move_left_config_) + get_axis_value(msg, move_right_config_);
        mobile_twist_message->linear.z = 0.0;
        mobile_twist_message->angular.x = 0.0;
        mobile_twist_message->angular.y = 0.0;
        mobile_twist_message->angular.z = (get_axis_value(msg, turn_right_config_) - get_axis_value(msg, turn_left_config_));

        /*RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "twist.linear.x value is ::: %f", mobile_twist_message->linear.x);
        RCLCPP_INFO(this->get_logger(), "twist.linear.y value is ::: %f", mobile_twist_message->linear.y);
        RCLCPP_INFO(this->get_logger(), "twist.angular.z value is ::: %f", mobile_twist_message->angular.z);
        RCLCPP_INFO(this->get_logger(), "========================================");*/
        mobile_twist_pub_->publish(std::move(mobile_twist_message));

        if (check_L1_L2_sate(msg))
        {
          if(check_axis_value_not_zero(msg))
          {
            has_event = true;
          }else
          {
            has_event = false;
          }
        }

        if(get_button_value(msg, btn_HOME_config_) == 1)
        {
            shtudown_cnt_ += 1;
            if(shtudown_cnt_ > 30)
            {
                std::thread(std::bind(&Tm5JoyTeleop::system_down, this)).detach();
            }
        } else
        {
          shtudown_cnt_ = 0;
        }

        if(get_button_value(msg, btn_X_config_) == 1)
        {
          //btnx => HOME position as joint move
          std::vector<double> tmpTarget;
          tmpTarget = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
          //manipulator.moveTargetJoint(tmpTarget);
          //std::thread moveTargetJointThread([=](){
          //    Manipulation.movetargetjointStart(tmpTarget);
          //});
          //moveTargetJointThread.detach();
        }

        if(get_button_value(msg, btn_Y_config_) == 1)
        {
            //btnY => ready position as joint move
            std::vector<double> tmpTarget;
            tmpTarget = {0.0, 0.0, 1.614, 0.0, 1.5447, 0.0};
            //manipulator.moveTargetJoint(tmpTarget);
            //std::thread moveTargetJointThread([=](){
            //    Manipulation.movetargetjointStart(tmpTarget);
            //});
            //moveTargetJointThread.detach();
        }

        if(get_button_value(msg, btn_A_config_) == 1 && get_button_value(msg, btn_B_config_) == 0)
        {
            frame_id_name_ = "link_0";
        }else if(get_button_value(msg, btn_A_config_) == 0 && get_button_value(msg, btn_B_config_) == 1)
        {
          frame_id_name_ = "link_6";
        }

        if(get_button_value(msg, btn_MINUS_config_) == 1 && pre_btn_value_[8] == false)
        {
            //RCLCPP_INFO(this->get_logger(), "check button value!!");
            cure_axis_scale_ -= ((max_axis_scale_ - min_axis_scale_) / 5);

            if(cure_axis_scale_ <= min_axis_scale_)
            {
              cure_axis_scale_ = min_axis_scale_;
            }

            cure_mobie_scale_ -=((max_mobile_scale_ - min_mobile_scale_ / 5));
            if(cure_mobie_scale_ <= min_mobile_scale_)
            {
              cure_mobie_scale_ = min_mobile_scale_;
            }

            std::thread(std::bind(&Tm5JoyTeleop::set_axis_scale, this)).detach();
        }

        if(get_button_value(msg, btn_PLUS_config_) == 1 && pre_btn_value_[7] == false)
        {
          cure_axis_scale_ += ((max_axis_scale_ - min_axis_scale_) / 5);
          if(cure_axis_scale_ >= max_axis_scale_)
          {
            cure_axis_scale_ = max_axis_scale_;
          }

          cure_mobie_scale_ += ((max_mobile_scale_ - min_mobile_scale_) / 5);
          if(cure_mobie_scale_ >= max_mobile_scale_)
          {
            cure_mobie_scale_ = max_mobile_scale_;
          }
          std::thread(std::bind(&Tm5JoyTeleop::set_axis_scale, this)).detach();
        }

        check_button_value(msg);

        if(has_event)
        {
            //RCLCPP_INFO(this->get_logger(), "frame id name is %s", frame_id_name_.c_str());
            geometry_msgs::msg::TwistStamped::UniquePtr arm_twist_message(new geometry_msgs::msg::TwistStamped());
            arm_twist_message->header.frame_id = frame_id_name_;
            arm_twist_message->header.stamp = now();
            if(get_button_value(msg, btn_L1_config_) == 1)
            {
                arm_twist_message->twist.linear.x = get_axis_value(msg, arm_xaxis_plus_config_) + get_axis_value(msg, arm_xaxis_minus_config_);
                arm_twist_message->twist.linear.y = get_axis_value(msg, arm_yaxis_plus_config_) + get_axis_value(msg, arm_yaxis_minus_config_);
                arm_twist_message->twist.linear.z = get_axis_value(msg, arm_zaxis_plus_config_) + get_axis_value(msg, arm_zaxis_minus_config_);
            } else if(get_button_value(msg,btn_L2_config_ ) == 1)
            {
                arm_twist_message->twist.angular.x = get_axis_value(msg, arm_xaxis_plus_config_) + get_axis_value(msg, arm_xaxis_minus_config_);
                arm_twist_message->twist.angular.y = get_axis_value(msg, arm_yaxis_plus_config_) + get_axis_value(msg, arm_yaxis_minus_config_);
                arm_twist_message->twist.angular.z = get_axis_value(msg, arm_zaxis_plus_config_) + get_axis_value(msg, arm_zaxis_minus_config_);
            }
            /*RCLCPP_INFO(this->get_logger(), "=========================================================");
            RCLCPP_INFO(this->get_logger(), "arm twist linear x value is ::: %f", arm_twist_message->twist.linear.x);
            RCLCPP_INFO(this->get_logger(), "arm twist linear y value is ::: %f", arm_twist_message->twist.linear.y);
            RCLCPP_INFO(this->get_logger(), "arm twist linear z value is ::: %f", arm_twist_message->twist.linear.z);
            RCLCPP_INFO(this->get_logger(), "arm twist angular x value is ::: %f", arm_twist_message->twist.angular.x);
            RCLCPP_INFO(this->get_logger(), "arm twist angular y value is ::: %f", arm_twist_message->twist.angular.y);
            RCLCPP_INFO(this->get_logger(), "arm twist angular z value is ::: %f", arm_twist_message->twist.angular.z);
            RCLCPP_INFO(this->get_logger(), "=========================================================");*/            arm_twist_pub_->publish(std::move(arm_twist_message));
        }
    }

    void Tm5JoyTeleop::check_button_value(std::unique_ptr<sensor_msgs::msg::Joy>& msg)
    {
        for(size_t i = 0; i < btn_configs_.size(); i ++)
        {
          if(get_button_value(msg, btn_configs_[i]) == 1)
          {
            pre_btn_value_[i] = true;
          }else{
            pre_btn_value_[i] = false;
          }
        }
    }

    void Tm5JoyTeleop::set_axis_scale()
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        arm_xaxis_minus_config_.scale = cure_axis_scale_;
        arm_xaxis_plus_config_.scale = cure_axis_scale_;
        arm_yaxis_minus_config_.scale = cure_axis_scale_;
        arm_yaxis_plus_config_.scale = cure_axis_scale_;
        arm_zaxis_minus_config_.scale = cure_axis_scale_;
        arm_zaxis_plus_config_.scale = cure_axis_scale_;

        move_forward_config_.scale = cure_mobie_scale_;
        move_back_config_.scale = cure_mobie_scale_;
        move_left_config_.scale = cure_mobie_scale_;
        move_right_config_.scale = cure_mobie_scale_;
        turn_left_config_.scale = cure_mobie_scale_;
        turn_right_config_.scale = cure_mobie_scale_;
    }

    void Tm5JoyTeleop::system_down()
    {
      std::system("sudo shutdown now");
    }

    bool Tm5JoyTeleop::check_axis_value_not_zero(std::unique_ptr<sensor_msgs::msg::Joy>& msg)
    {
        bool return_value = false;
        for(size_t i = 0; i < arm_axis_configs_.size(); i++)
        {
          if (get_axis_value(msg, arm_axis_configs_[i]) == 0.0)
          {
            if(pre_Axis_value_[i] != 0.0)
            {
              return_value = true;
            }
          } else{
            return_value = true;
          }
          pre_Axis_value_[i] = get_axis_value(msg, arm_axis_configs_[i]);
        }
        return return_value;
    }

    bool Tm5JoyTeleop::check_L1_L2_sate(std::unique_ptr<sensor_msgs::msg::Joy>& msg)
    {
        if(get_button_value(msg, btn_L1_config_) == 1 || get_button_value(msg, btn_L2_config_) == 1)
        {
          return true;
        } else{
          return false;
        }
    }

    double Tm5JoyTeleop::get_axis_value(std::unique_ptr<sensor_msgs::msg::Joy>& msg, AxisConfig& config)
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        if(msg->axes.size() >= (size_t)config.axis)
        {
          double value = 0.0;
          double inputValue = 0.0;
          inputValue = (double)msg->axes[config.axis] - config.offset;
          if(inputValue == 0)
          {
            value = 0.0;
          } else if (std::abs(inputValue) <= config.deadzone)
          {
              value = 0.0;
          } else if(inputValue > 0)
          {
              value = ((inputValue - config.deadzone) / (1-config.deadzone)) * config.scale;
          } else if(inputValue < 0)
          {
              value = ((inputValue + config.deadzone) / (1-config.deadzone)) * config.scale;
          }
          return value;
        }
        return 0.0;
    }

    int Tm5JoyTeleop::get_button_value(std::unique_ptr<sensor_msgs::msg::Joy>& msg, ButtonConfig& config)
    {
      if(msg->buttons.size() >= (size_t)config.num)
      {
        uint32_t value = msg->buttons[config.num];
        return value;
      }
      return 0;
    }

    void Tm5JoyTeleop::declare_parameter_value()
    {
        declare_parameter<std::string>("joy_topic", "joy");
        declare_parameter<std::string>("mobile_twist_topic", "cmd_vel");
        declare_parameter<std::string>("arm_twist_topic", "/servo_node/delta_twist_cmds");
        //declare_parameter<std::string>("arm_joint_topic", "servo_node/delta/joint_cmds");
        //declare_parameter<std::string>("arm_base_frame_id", "base");
        //declare_parameter<std::string>("arm_ee_frame_id" "link_6");

        declare_parameter<double>("mobile.scale", 0.5);
        declare_parameter<double>("mobile.max_scale", 0.5);
        declare_parameter<double>("mobile.min_scale", 0.5);
        //move_forward_config_
        declare_parameter<int>("mobile.move_forward.axis", 1);
        declare_parameter<double>("mobile.move_forward.offset", 0.0);
        declare_parameter<double>("mobile.move_forward.deadzone", 0.0);
        //move_back_config_
        declare_parameter<int>("mobile.move_back.axis", 1);
        declare_parameter<double>("mobile.move_back.offset", 0.0);
        declare_parameter<double>("mobile.move_back.deadzone", 0.0);
        //move_left_config_
        declare_parameter<int>("mobile.move_left.axis", 0);
        declare_parameter<double>("mobile.move_left.offset", 0.0);
        declare_parameter<double>("mobile.move_left.deadzone", 0.0);
        //move_right_config_
        declare_parameter<int>("mobile.move_right.axis", 0);
        declare_parameter<double>("mobile.move_right.offset", 0.0);
        declare_parameter<double>("mobile.move_right.deadzone", 0.0);
        // turn_left_config_
        declare_parameter<int>("mobile.turn_left.axis", 5);
        declare_parameter<double>("mobile.turn_left.offset", 0.0);
        declare_parameter<double>("mobile.turn_left.deadzone", 0.0);
        //turn_right_config_
        declare_parameter<int>("mobile.turn_right.axis", 4);
        declare_parameter<double>("mobile.turn_right.offset", 0.0);
        declare_parameter<double>("mobile.turn_right.deadzone", 0.0);

        declare_parameter<double>("arm.scale", 0.03);
        declare_parameter<double>("arm.max_scale", 0.1);
        declare_parameter<double>("arm.min_scale", 0.01);
        //arm_xaxis_plus_config_
        declare_parameter<int>("arm.xaxis_plus.axis", 3);
        declare_parameter<double>("arm.xaxis_plus.offset", 0.0);
        declare_parameter<double>("arm.xaxis_plus.deadzone", 0.0);
        //arm_xaxis_minus_config_
        declare_parameter<int>("arm.xaxis_minus.axis", 3);
        declare_parameter<double>("arm.xaxis_minus.offset", 0.0);
        declare_parameter<double>("arm.xaxis_minus.deadzone", 0.0);
        //arm_yaxis_plus_config_
        declare_parameter<int>("arm.yaxis_plus.axis", 2);
        declare_parameter<double>("arm.yaxis_plus.offset", 0.0);
        declare_parameter<double>("arm.yaxis_plus.deadzone", 0.0);
        //arm_yaxis_minus_config_
        declare_parameter<int>("arm.yaxis_minus.axis", 2);
        declare_parameter<double>("arm.yaxis_minus.offset", 0.0);
        declare_parameter<double>("arm.yaxis_minus.deadzone", 0.0);
        //arm_zaxis_plus_config_
        declare_parameter<int>("arm.zaxis_plus.axis", 7);
        declare_parameter<double>("arm.zaxis_plus.offset", 0.0);
        declare_parameter<double>("arm.zaxis_plus.deadzone", 0.0);
        //arm_zaxis_minus_config_
        declare_parameter<int>("arm.zaxis_minus.axis", 7);
        declare_parameter<double>("arm.zaxis_minus.offset", 0.0);
        declare_parameter<double>("arm.zaxis_minus.deadzone", 0.0);

        //btn_A_config_
        declare_parameter<std::string>("btn.A_config.name", "btn_A");
        declare_parameter<int>("btn.A_config.num", 0);
        //btn_B_config_
        declare_parameter<std::string>("btn.B_config.name", "btn_B");
        declare_parameter<int>("btn.B_config.num", 1);
        //btn_X_config_
        declare_parameter<std::string>("btn.X_config.name", "btn_X");
        declare_parameter<int>("btn.X_config.num", 3);
        //btn_Y_config_
        declare_parameter<std::string>("btn.Y_config.name", "btn_Y");
        declare_parameter<int>("btn.Y_config.num", 4);
        //btn_HOME_config_
        declare_parameter<std::string>("btn.HOME_config.name", "btn_HOME");
        declare_parameter<int>("btn.HOME_config.num", 16);
        //btn_MINUS_config_
        declare_parameter<std::string>("btn.MINUS_config.name", "btn_MINUS");
        declare_parameter<int>("btn.MINUS_config.num", 15);
        //btn_PLUS_config_
        declare_parameter<std::string>("btn.PLUS_config.name", "btn_PLUS");
        declare_parameter<int>("btn.PLUS_config.num", 11);
        //btn_L1_config_
        declare_parameter<std::string>("btn.L1_config.name", "btn_L1");
        declare_parameter<int>("btn.L1_config.num", 6);
        //btn_L2_config_
        declare_parameter<std::string>("btn.L2_config.name", "btn_L2");
        declare_parameter<int>("btn.L2_config.num", 7);

        cure_mobie_scale_ = get_parameter("mobile.scale").as_double();
        max_mobile_scale_ = get_parameter("mobile.max_scale").as_double();
        min_mobile_scale_ = get_parameter("mobile.min_scale").as_double();

        move_forward_config_.axis = (uint32_t)get_parameter("mobile.move_forward.axis").as_int();
        move_forward_config_.scale = cure_mobie_scale_;
        move_forward_config_.offset = get_parameter("mobile.move_forward.offset").as_double();
        move_forward_config_.deadzone = get_parameter("mobile.move_forward.deadzone").as_double();

        move_back_config_.axis = (uint32_t)get_parameter("mobile.move_back.axis").as_int();
        move_back_config_.scale = cure_mobie_scale_;
        move_back_config_.offset = get_parameter("mobile.move_back.offset").as_double();
        move_back_config_.deadzone = get_parameter("mobile.move_back.deadzone").as_double();

        move_left_config_.axis = (uint32_t)get_parameter("mobile.move_left.axis").as_int();
        move_left_config_.scale = cure_mobie_scale_;
        move_left_config_.offset = get_parameter("mobile.move_left.offset").as_double();
        move_left_config_.deadzone = get_parameter("mobile.move_left.deadzone").as_double();

        move_right_config_.axis = (uint32_t)get_parameter("mobile.move_right.axis").as_int();
        move_right_config_.scale = cure_mobie_scale_;
        move_right_config_.offset = get_parameter("mobile.move_right.offset").as_double();
        move_right_config_.deadzone = get_parameter("mobile.move_right.deadzone").as_double();

        turn_left_config_.axis = (uint32_t)get_parameter("mobile.turn_left.axis").as_int();
        turn_left_config_.scale = cure_mobie_scale_;
        turn_left_config_.offset = get_parameter("mobile.turn_left.offset").as_double();
        turn_left_config_.deadzone = get_parameter("mobile.turn_left.deadzone").as_double();

        turn_right_config_.axis = (uint32_t)get_parameter("mobile.turn_right.axis").as_int();
        turn_right_config_.scale = cure_mobie_scale_;
        turn_right_config_.offset = get_parameter("mobile.turn_right.offset").as_double();
        turn_right_config_.deadzone = get_parameter("mobile.turn_right.deadzone").as_double();

        cure_axis_scale_ = get_parameter("arm.scale").as_double();
        max_axis_scale_ = get_parameter("arm.max_scale").as_double();
        min_axis_scale_ = get_parameter("arm.min_scale").as_double();
        arm_xaxis_plus_config_.axis = (uint32_t)get_parameter("arm.xaxis_plus.axis").as_int();
        arm_xaxis_plus_config_.scale = cure_axis_scale_;
        arm_xaxis_plus_config_.offset = get_parameter("arm.xaxis_plus.offset").as_double();
        arm_xaxis_plus_config_.deadzone = get_parameter("arm.xaxis_plus.deadzone").as_double();

        arm_xaxis_minus_config_.axis = (uint32_t)get_parameter("arm.xaxis_minus.axis").as_int();
        arm_xaxis_minus_config_.scale = cure_axis_scale_;
        arm_xaxis_minus_config_.offset = get_parameter("arm.xaxis_minus.offset").as_double();
        arm_xaxis_minus_config_.deadzone = get_parameter("arm.xaxis_minus.deadzone").as_double();

        arm_yaxis_plus_config_.axis = (uint32_t)get_parameter("arm.yaxis_plus.axis").as_int();
        arm_yaxis_plus_config_.scale = cure_axis_scale_;
        arm_yaxis_plus_config_.offset = get_parameter("arm.yaxis_plus.offset").as_double();
        arm_yaxis_plus_config_.deadzone = get_parameter("arm.yaxis_plus.deadzone").as_double();

        arm_yaxis_minus_config_.axis = (uint32_t)get_parameter("arm.yaxis_minus.axis").as_int();
        arm_yaxis_minus_config_.scale = cure_axis_scale_;
        arm_yaxis_minus_config_.offset = get_parameter("arm.yaxis_minus.offset").as_double();
        arm_yaxis_minus_config_.deadzone = get_parameter("arm.yaxis_minus.deadzone").as_double();

        arm_zaxis_plus_config_.axis = (uint32_t)get_parameter("arm.zaxis_plus.axis").as_int();
        arm_zaxis_plus_config_.scale = cure_axis_scale_;
        arm_zaxis_plus_config_.offset = get_parameter("arm.zaxis_plus.offset").as_double();
        arm_zaxis_plus_config_.deadzone = get_parameter("arm.zaxis_plus.deadzone").as_double();

        arm_zaxis_minus_config_.axis = (uint32_t)get_parameter("arm.zaxis_minus.axis").as_int();
        arm_zaxis_minus_config_.scale = cure_axis_scale_;
        arm_zaxis_minus_config_.offset = get_parameter("arm.zaxis_minus.offset").as_double();
        arm_zaxis_minus_config_.deadzone = get_parameter("arm.zaxis_minus.deadzone").as_double();

        //A :: 0
        btn_A_config_.name = get_parameter("btn.A_config.name").as_string();
        btn_A_config_.num = (uint32_t)get_parameter("btn.A_config.num").as_int();
        //B :: 1
        btn_B_config_.name = get_parameter("btn.B_config.name").as_string();
        btn_B_config_.num = (uint32_t)get_parameter("btn.B_config.num").as_int();
        //X :: 3
        btn_X_config_.name = get_parameter("btn.X_config.name").as_string();
        btn_X_config_.num = (uint32_t)get_parameter("btn.X_config.num").as_int();
        //Y :: 4
        btn_Y_config_.name = get_parameter("btn.Y_config.name").as_string();
        btn_Y_config_.num = (uint32_t)get_parameter("btn.Y_config.num").as_int();
        //+ :: 11
        btn_PLUS_config_.name = get_parameter("btn.PLUS_config.name").as_string();
        btn_PLUS_config_.num = (uint32_t)get_parameter("btn.PLUS_config.num").as_int();
        //L1 :: 6
        btn_L1_config_.name = get_parameter("btn.L1_config.name").as_string();
        btn_L1_config_.num = (uint32_t)get_parameter("btn.L1_config.num").as_int();
        //L2 :: 7
        btn_L2_config_.name = get_parameter("btn.L2_config.name").as_string();
        btn_L2_config_.num = (uint32_t)get_parameter("btn.L2_config.num").as_int();
        //-  :: 15
        btn_MINUS_config_.name = get_parameter("btn.MINUS_config.name").as_string();
        btn_MINUS_config_.num = (uint32_t)get_parameter("btn.MINUS_config.num").as_int();
        // HOME :: 16
        btn_HOME_config_.name= get_parameter("btn.HOME_config.name").as_string();
        btn_HOME_config_.num = (uint32_t)get_parameter("btn.HOME_config.num").as_int();
    }
}
} // namespace name

