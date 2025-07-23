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


#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include "pluginlib/class_list_macros.hpp"
#include <rclcpp/rclcpp.hpp>
#include "tpc_mobile_mecanum_wheel_controller/tpc_mobile_mecanum_wheel_controller.hpp"

PLUGINLIB_EXPORT_CLASS(
  tpc::tpc_mobile_mecanum_wheel_controller::TpcMobileMecanumController,
  controller_interface::ControllerInterface
)

namespace tpc
{
namespace tpc_mobile_mecanum_wheel_controller
{
    auto logger = rclcpp::get_logger("TpcMobileMecanumController");
    TpcMobileMecanumController::TpcMobileMecanumController()
    : controller_interface::ControllerInterface(),
      command_velocity_subscription_(nullptr),
      command_velocity_ptr_(nullptr)
      {

      }

    TpcMobileMecanumController::~TpcMobileMecanumController()
    {

    }
    controller_interface::InterfaceConfiguration TpcMobileMecanumController::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration command_interface_config;
        command_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        RCLCPP_INFO(logger,"Configure command insterface Mecanum Wheel Controller");
        command_interface_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
        command_interface_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
        command_interface_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
        command_interface_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
        return command_interface_config;
    }

    controller_interface::InterfaceConfiguration TpcMobileMecanumController::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration state_interface_config;
        state_interface_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

        RCLCPP_INFO(logger,"Configure state insterface Mecanum Wheel Controller");
        state_interface_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
        state_interface_config.names.push_back(fl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
        state_interface_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
        state_interface_config.names.push_back(fr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
        state_interface_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
        state_interface_config.names.push_back(rl_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);
        state_interface_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
        state_interface_config.names.push_back(rr_wheel_joint_name_ + "/" + hardware_interface::HW_IF_VELOCITY);

        return state_interface_config;
    }

    controller_interface::return_type TpcMobileMecanumController::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        //get last command of velocity
        auto velocity_command = command_velocity_ptr_.readFromRT();
        if(! velocity_command || !(*velocity_command))
        {
            return controller_interface::return_type::OK;
        }

        // Calculate the wheel velocity
        // http://robotsforroboticists.com/drive-kinematics/

        const auto twist = (*velocity_command);
        /*RCLCPP_WARN(logger, "wheel_radius_ is %d", wheel_radius_);
        RCLCPP_WARN(logger, "twist.linear.x is %d", twist->linear.x);
        RCLCPP_WARN(logger, "twist.linear.y is %d", twist->linear.y);
        RCLCPP_WARN(logger, "twist.angular.z is %d", twist->angular.z);
        RCLCPP_WARN(logger, "wheel_separation_width_ is %d", wheel_separation_width_);
        RCLCPP_WARN(logger, "wheel_separation_length_ is %d", wheel_separation_length_);*/

        //double fl_wheel_velocity = (1 / wheel_radius_) * (twist->linear.x - twist->linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z);
        //double fr_wheel_velocity = (1 / wheel_radius_) * (twist->linear.x + twist->linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z);
        //double rl_wheel_velocity = (1 / wheel_radius_) * (twist->linear.x + twist->linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z);
        //double rr_wheel_velocity = (1 / wheel_radius_) * (twist->linear.x - twist->linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z);

        double fl_wheel_velocity = ((twist->linear.x - twist->linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z) / (2*M_PI*wheel_radius_)) * 60;
        double fr_wheel_velocity = ((twist->linear.x + twist->linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z) / (2*M_PI*wheel_radius_)) * 60;
        double rl_wheel_velocity = ((twist->linear.x + twist->linear.y - (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z) / (2*M_PI*wheel_radius_)) * 60;
        double rr_wheel_velocity = ((twist->linear.x - twist->linear.y + (wheel_separation_width_ + wheel_separation_length_) * twist->angular.z) / (2*M_PI*wheel_radius_)) * 60;

        fl_wheel_->set_velocity(fl_wheel_velocity);
        fr_wheel_->set_velocity(fr_wheel_velocity);
        rl_wheel_->set_velocity(rl_wheel_velocity);
        rr_wheel_->set_velocity(rr_wheel_velocity);

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn TpcMobileMecanumController::on_init()
    {
      RCLCPP_INFO(logger, "on configure TpcMobileMecanumControllerrrrrrrrrrrr!!!");
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TpcMobileMecanumController::on_configure(const rclcpp_lifecycle::State & previous_state)
    {
      RCLCPP_INFO(logger, "on configure TpcMobileMecanumController");

      //fl_wheel_joint_name_ = get_node()->get_parameter("fr_left_wheel_joint_name").as_string();
      fl_wheel_joint_name_ = "fr_left_wheel_joint";
      RCLCPP_WARN(logger, "fr_left_wheel_joint_name is %s", fl_wheel_joint_name_.c_str());
      //fr_wheel_joint_name_ = get_node()->get_parameter("fr_right_wheel_joint_name").as_string();
      fr_wheel_joint_name_ = "fr_right_wheel_joint";
      RCLCPP_WARN(logger, "fr_right_wheel_joint_name is %s", fr_wheel_joint_name_.c_str());
      //rl_wheel_joint_name_ = get_node()->get_parameter("rr_left_wheel_joint_name").as_string();
      rl_wheel_joint_name_ = "rr_left_wheel_joint";
      RCLCPP_WARN(logger, "rr_left_wheel_joint_name is %s", rl_wheel_joint_name_.c_str());
      //rr_wheel_joint_name_ = get_node()->get_parameter("rr_right_wheel_joint_name").as_string();
      rr_wheel_joint_name_ = "rr_right_wheel_joint";
      RCLCPP_WARN(logger, "rr_right_wheel_joint_name is %s", rr_wheel_joint_name_.c_str());

      if(fl_wheel_joint_name_.empty()){
        RCLCPP_ERROR(logger, "fl_wheel_joint_name param is empty");
        return controller_interface::CallbackReturn::ERROR;
      }
      if(fr_wheel_joint_name_.empty()){
        RCLCPP_ERROR(logger, "fr_wheel_joint_name param is empty");
        return controller_interface::CallbackReturn::ERROR;
      }
      if(rl_wheel_joint_name_.empty()){
        RCLCPP_ERROR(logger, "rl_wheel_joint_name param is empty");
        return controller_interface::CallbackReturn::ERROR;
      }
      if(rr_wheel_joint_name_.empty()){
        RCLCPP_ERROR(logger, "rr_wheel_joint_name param is empty");
        return controller_interface::CallbackReturn::ERROR;
      }

      //wheel_radius_ = get_node()->get_parameter("wheel_radius").as_double();
      wheel_radius_ = 0.09;
      //wheel_distance_width_ = get_node()->get_parameter("wheel_distance.width").as_double();
      wheel_distance_width_ = 0.6;
      wheel_distance_length_ = 0.575;
      if(wheel_radius_<= 0.0){
        RCLCPP_ERROR(logger, "wheel_radius param cannot be zero or less than zero");
        return controller_interface::CallbackReturn::ERROR;
      }
      if(wheel_distance_width_ <= 0.0){
        RCLCPP_ERROR(logger, "wheel_distance_width param cannot be zero or less than zero");
        return controller_interface::CallbackReturn::ERROR;
      }
      if(wheel_distance_length_ <= 0){
        RCLCPP_ERROR(logger, "wheel_distance_length param cannot be zero or less than zero");
        return controller_interface::CallbackReturn::ERROR;
      }

      wheel_separation_width_ = wheel_distance_width_ / 2;
      wheel_separation_length_ = wheel_distance_length_ / 2;

      if(!reset()){
        return controller_interface::CallbackReturn::ERROR;
      }

      command_velocity_subscription_ = get_node()->create_subscription<Twist>("/cmd_vel", 10, [this](const Twist::SharedPtr twist)
      {
        command_velocity_ptr_.writeFromNonRT(twist);
      });

      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TpcMobileMecanumController::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
      // Init wheels
      RCLCPP_INFO(logger, "on arctivate TpcMobileMecanumController");
      fl_wheel_ = get_wheel(fl_wheel_joint_name_);
      fr_wheel_ = get_wheel(fr_wheel_joint_name_);
      rl_wheel_ = get_wheel(rl_wheel_joint_name_);
      rr_wheel_ = get_wheel(rr_wheel_joint_name_);

      if(!fl_wheel_ || !fr_wheel_ || !rl_wheel_ || !rr_wheel_)
      {
        return controller_interface::CallbackReturn::ERROR;
      }
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TpcMobileMecanumController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TpcMobileMecanumController::on_cleanup(const rclcpp_lifecycle::State &previous_state)
    {
      if(!reset()){
        return controller_interface::CallbackReturn::ERROR;
      }
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TpcMobileMecanumController::on_error(const rclcpp_lifecycle::State &previous_state)
    {
      if(!reset()){
        return controller_interface::CallbackReturn::ERROR;
      }
      return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn TpcMobileMecanumController::on_shutdown(const rclcpp_lifecycle::State &previous_state)
    {
      return controller_interface::CallbackReturn::SUCCESS;
    }


    std::shared_ptr<MecanumWheel> TpcMobileMecanumController::get_wheel(const std::string & wheel_joint_name)
    {
      //find position state interface
      const auto position_state = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
      {
        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });

      if(position_state == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "%s position state interface not found", wheel_joint_name.c_str());
        return nullptr;
      }

      //find velocity state interface
      const auto velocity_state = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&wheel_joint_name](const hardware_interface::LoanedStateInterface & interface)
      {
        return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
      });

      if(velocity_state == state_interfaces_.cend())
      {
        RCLCPP_ERROR(logger, "%s velocity state interface not found", wheel_joint_name.c_str());
        return nullptr;
      }

      //find velocity command interface
      const auto velocity_command = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(), [&wheel_joint_name](const hardware_interface::LoanedCommandInterface & interface)
        {
          return interface.get_prefix_name() == wheel_joint_name && interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
        }
      );
      if(velocity_command == command_interfaces_.end())
      {
        RCLCPP_ERROR(logger, "%s velocity command interface not found", wheel_joint_name.c_str());
        return nullptr;
      }

      //create wheel instance
      return std::make_shared<MecanumWheel>(
        std::ref(*position_state),
        std::ref(*velocity_state),
        std::ref(*velocity_command));
    }

    bool TpcMobileMecanumController::reset()
    {
      subscriber_is_actve_ = false;
      command_velocity_subscription_.reset();

      fl_wheel_.reset();
      fr_wheel_.reset();
      rl_wheel_.reset();
      rr_wheel_.reset();

      return true;
    }

}
}
