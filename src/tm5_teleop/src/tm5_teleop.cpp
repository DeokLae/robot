// Copyright 2022 ROBOTIS CO., LTD.
//
// Copyright 2024 TPC Mechatronics Crop.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original Author: Hye-jong KIM
//
// Modified by: DeokLae Kim (kdl79@tanhay.com)
// Description of modifications : This source code is a modification of the
// turtlebot3_manipulation_teleop.cpp file from ROBOTIS, adapted to implement
// the direct teaching function for TM robots.

#include <algorithm>
#include <memory>

#include "tm5_teleop/tm5_teleop.hpp"


//KeyboardReader
KeyboardReader::KeyboardReader()
: kfd(0)
{
  //get the console in raw mode
  tcgetattr(kfd, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  //setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}

void KeyboardReader::readOne(char * c)
{
  int rc = read(kfd, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}

void KeyboardReader::shutdown()
{
  tcsetattr(kfd, TCSANOW, &cooked);
}

//keyboardServo
KeyboardServo::KeyboardServo() : publish_task_(false), publish_joint_(false)
{
  //nh_ = rclcpp::Node::make_shared("servo_keyboard_input");
  nh_ = std::make_shared<rclcpp::Node>("servo_keyboard_input");

  servo_start_client_ = nh_->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_stop_client_ = nh_->create_client<std_srvs::srv::Trigger>("/servo_node/stop_servo");

  base_twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>(BASE_TWIST_TOPIC, ROS_QUEUE_SIZE);
  arm_twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(ARM_TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(ARM_JOINT_TOPIC, ROS_QUEUE_SIZE);

  cmd_vel_ = geometry_msgs::msg::Twist();

  arm_twist_vel_= ARM_TWIST_VEL;
  arm_joint_vel_ = ARM_JOINT_VEL;

}

KeyboardServo::~KeyboardServo()
{
  stop_moveit_servo();
}

int KeyboardServo::keyLoop()
{
  char c;

  std::thread(std::bind(&KeyboardServo::spin, this)).detach();
  connect_moveit_servo();
  start_moveit_servo();

  puts("Reading from keyboard");
  puts("-----------------------");
  puts("Use 1|2|3|4|5|6|q|w|e|r|t|y keys to joint jog");
  puts("Use a|s|d|f|z|x|c|v keys to Cartesian jog");
  puts("ESC to quit");

  std::thread(std::bind(&KeyboardServo::pub, this)).detach();

  bool servoing = true;
  while (servoing)
  {
    /* code */
    try{
      input.readOne(&c);
    }catch(const std::runtime_error &){
      perror("read():");
      return -1;
    }
    RCLCPP_INFO(nh_->get_logger(), "value: 0x%02x", c);

    switch (c)
    {
    case KEYCODE_PLUS:
      arm_twist_vel_ = std::min(arm_twist_vel_ + ARM_TWIST_VEL_STEP, ARM_TWIST_VEL_MAX);
      arm_joint_vel_ = std::min(arm_joint_vel_ + ARM_JOINT_VEL_SETP, ARM_JOINT_VEL_MAX);
      manipulator.cartesian_velocity_scale_ = std::min(manipulator.cartesian_velocity_scale_ + TpcManipulationMoveit::CARTE_ACCELERATION_SCALE_STEP,
                                                       TpcManipulationMoveit::CARTE_ACCELERATION_SCALE_MAX);
      manipulator.cartesian_acceleration_scale_ = std::min(manipulator.cartesian_acceleration_scale_ + TpcManipulationMoveit::CARTE_ACCELERATION_SCALE_STEP,
                                                           TpcManipulationMoveit::CARTE_ACCELERATION_SCALE_MAX);
      manipulator.joint_velocity_scale_ = std::min(manipulator.joint_velocity_scale_ + TpcManipulationMoveit::JOINT_VELOCITY_SCALE_STEP,
                                                   TpcManipulationMoveit::JOINT_VELOCITY_SCALE_MAX);
      manipulator.joint_acceleration_scale_ = std::min(manipulator.joint_acceleration_scale_ + TpcManipulationMoveit::jOINT_ACCELERATION_SCALE_STEP,
                                                       TpcManipulationMoveit::jOINT_ACCELERATION_SCALE_MAX);
      RCLCPP_INFO(nh_->get_logger(), "Twist speed set :: %f", arm_twist_vel_);
      RCLCPP_INFO(nh_->get_logger(), "joint speed set :: %f", arm_joint_vel_);
      break;
    case KEYCODE_MINUS:
      arm_twist_vel_ = std::max(arm_twist_vel_ - ARM_TWIST_VEL_STEP, ARM_TWIST_VEL);
      arm_joint_vel_ = std::max(arm_joint_vel_ - ARM_JOINT_VEL_SETP, ARM_JOINT_VEL);
      manipulator.cartesian_velocity_scale_ = std::max(manipulator.cartesian_velocity_scale_ - TpcManipulationMoveit::CARTE_VELOCITY_SCALE_STEP,
                                                       TpcManipulationMoveit::CARTE_VELOCITY_SCALE);
      manipulator.cartesian_acceleration_scale_ = std::max(manipulator.cartesian_acceleration_scale_ - TpcManipulationMoveit::CARTE_ACCELERATION_SCALE_STEP,
                                                           TpcManipulationMoveit::CARTE_ACCELERATION_SCALE);
      manipulator.joint_velocity_scale_ = std::max(manipulator.joint_velocity_scale_ - TpcManipulationMoveit::JOINT_VELOCITY_SCALE_STEP,
                                                  TpcManipulationMoveit::JOINT_VELOCITY_SCALE);
      manipulator.joint_acceleration_scale_ = std::max(manipulator.joint_acceleration_scale_ - TpcManipulationMoveit::jOINT_ACCELERATION_SCALE_STEP,
                                                       TpcManipulationMoveit::jOINT_ACCELERATION_SCALE);
      RCLCPP_INFO(nh_->get_logger(), "Twist speed set :: %f", arm_twist_vel_);
      RCLCPP_INFO(nh_->get_logger(), "joint speed set :: %f", arm_joint_vel_);
      break;
    case KEYCODE_SPACE:
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Clicked SpaceBar!!");
     manipulator.drawArcAtToolCoordinate(0.1, 100, 360, 1, 1, 1);
     // manipulator.moveJobStart();
      break;
    case KEYCODE_ENTER:
      manipulator.moveJobStart();
      break;
    case KEYCODE_M:
      //manipulator.saveCurePoseToJob("jointvalue.tpc");
      manipulator.saveCureWapointToJob("jointvalue.tpc");
      break;
    case KEYCODE_N:
      //manipulator.saveCureJointTojob("jointvalue.tpc");
      manipulator.runWapoints();
      break;

      //amr 로봇의 전 후 좌 우 이동
    case KEYCODE_O:   //KEYCODE_UP  전진인데 왜 linear.x 일까
      /* code */
      cmd_vel_.linear.x = std::min(cmd_vel_.linear.x + BASE_LINEAR_VEL_STEP, BASE_LINEAR_VEL_MAX);
      cmd_vel_.linear.y = 0.0;
      cmd_vel_.linear.z = 0.0;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "LINEAR VEL : " << cmd_vel_.linear.x);
      break;
    case KEYCODE_K:   //KEYCODE_LEFT  왼쪽인데 왜 z 일까
      /* code */
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.linear.y = 0.0;
      cmd_vel_.angular.z = std::min(cmd_vel_.angular.z + BASE_ANGULAR_VEL_STEP, BASE_ANGULAR_VEL_MAX);
      RCLCPP_INFO_STREAM(nh_->get_logger(), "ANGULAR VEL : " << cmd_vel_.angular.z);
      break;
    case KEYCODE_L:  //KEYCODE_DOWN
      /* code */
      cmd_vel_.linear.x = std::min(cmd_vel_.linear.x + BASE_LINEAR_VEL_STEP, BASE_LINEAR_VEL_MAX);
      cmd_vel_.linear.y = 0.0;
      cmd_vel_.linear.z = 0.0;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "LINEAR VEL : " << cmd_vel_.linear.x);
      break;
    case KEYCODE_SEMICOLON:  //KEYBOARD_RIGHT
      /* code */
      cmd_vel_.linear.x = 0.0;
      cmd_vel_.linear.y = 0.0;
      cmd_vel_.angular.z = std::min(cmd_vel_.angular.z + BASE_ANGULAR_VEL_STEP, BASE_ANGULAR_VEL_MAX);
      RCLCPP_INFO_STREAM(nh_->get_logger(), "ANGULAR VEL : " << cmd_vel_.angular.z);
      break;
      //amr 로봇의 전 후 좌 우 이동 끝

//-------------------------------------------------------------------//
//------------------------------------------------------------------//
    //다관절 로봇의 cartesian jog
    case KEYCODE_A:
      /* code */
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = arm_twist_vel_;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "EEF move left");
      manipulator.saveCureWapointToJob("jointvalue.tpc");
      break;
    case KEYCODE_D:
      /* code */
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = -arm_twist_vel_;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "EEF move right");
      break;
    case KEYCODE_S:
      /* code */
      task_msg_.twist.linear.x = arm_twist_vel_;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm x Front");
      break;
    case KEYCODE_X:
      /* code */
      task_msg_.twist.linear.x = -arm_twist_vel_;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm x Back");
      break;
    case KEYCODE_Z:
      /* code */
      joint_msg_.joint_names.push_back("joint_1");  //ros controller(arm_controller)에서 사용하는 joint의 이름을 사용
      joint_msg_.velocities.push_back(arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm Turn Left");//ros controller 에서 사용하는 joint 의 이름을 사용한다.
      //publish_task_ = true;
      break;
    case KEYCODE_C:
      /* code */
      joint_msg_.joint_names.push_back("joint_1");
      joint_msg_.velocities.push_back(-arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm Turn Right");
      task_msg_.twist.angular.x = -arm_twist_vel_;
      //publish_task_ = true;
      break;
    case KEYCODE_F:
      /* code */
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = arm_twist_vel_;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm z UP");
      break;
    case KEYCODE_V:
      /* code */
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = -arm_twist_vel_;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Arm z DOWN");
      break;

    // ------------------다관절 로봇 EEPOSE---------------------------//
    case KEYCODE_AA:
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = arm_twist_vel_;
      publish_task_ = true;
      break;
    case KEYCODE_DD:
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = -arm_twist_vel_;
      publish_task_ = true;
      break;
    case KEYCODE_SS:
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = arm_twist_vel_;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      break;
    case KEYCODE_XX:
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = -arm_twist_vel_;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      break;
    case KEYCODE_ZZ:
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = arm_twist_vel_;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      break;
    case KEYCODE_CC:
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = -arm_twist_vel_;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
      publish_task_ = true;
      break;

    //------------------------------------------------------------------//
    //------------------------------------------------------------------//
    //다관절 로봇의 joint jog
    case KEYCODE_1:
      /* code */
      joint_msg_.joint_names.push_back("joint_1");
      joint_msg_.velocities.push_back(arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint1 move +");
      break;
    case KEYCODE_2:
      /* code */
      joint_msg_.joint_names.push_back("joint_2");
      joint_msg_.velocities.push_back(arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint2 move +");
      break;
    case KEYCODE_3:
      /* code */
      joint_msg_.joint_names.push_back("joint_3");
      joint_msg_.velocities.push_back(arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint3 move +");
      break;
    case KEYCODE_4:
      /* code */
      joint_msg_.joint_names.push_back("joint_4");
      joint_msg_.velocities.push_back(arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint4 move +");
      break;
    case KEYCODE_5:
      /* code */
      joint_msg_.joint_names.push_back("joint_5");
      joint_msg_.velocities.push_back(arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint5 move +");
      break;
    case KEYCODE_6:
      /* code */
      joint_msg_.joint_names.push_back("joint_6");
      joint_msg_.velocities.push_back(arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint6 move +");
      break;

    //-----------------------------------------------------------------------//
    //-----------------------------------------------------------------------//
    case KEYCODE_Q:
      /* code */
      joint_msg_.joint_names.push_back("joint_1");
      joint_msg_.velocities.push_back(-arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint1 move -");
      break;
    case KEYCODE_W:
      /* code */
      joint_msg_.joint_names.push_back("joint_2");
      joint_msg_.velocities.push_back(-arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint2 move -");
      break;
    case KEYCODE_E:
      /* code */
      joint_msg_.joint_names.push_back("joint_3");
      joint_msg_.velocities.push_back(-arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint3 move -");
      break;
    case KEYCODE_R:
      /* code */
      joint_msg_.joint_names.push_back("joint_4");
      joint_msg_.velocities.push_back(-arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint4 move -");
      break;
    case KEYCODE_T:
      /* code */
      joint_msg_.joint_names.push_back("joint_5");
      joint_msg_.velocities.push_back(-arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint5 move -");
      break;
    case KEYCODE_Y:
      /* code */
      joint_msg_.joint_names.push_back("joint_6");
      joint_msg_.velocities.push_back(-arm_joint_vel_);
      publish_joint_ = true;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint6 move -");
      break;

    //----------------------------------------------------------//
    //----------------------------------------------------------//

    case KEYCODE_I:
      manipulator.runWapoints();
      break;
    case KEYCODE_ESC:
      /* code */
      RCLCPP_INFO_STREAM(nh_->get_logger(), "quit");
      manipulator.moveJobPause();
      break;

    default:
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Unsigned input : " << c);
      break;
    }
  }
  return 0;
}

void KeyboardServo::connect_moveit_servo()
{
  for(int i = 0; i < 10; i++){
    if(servo_start_client_->wait_for_service(std::chrono::seconds(1))){
      RCLCPP_INFO_STREAM(nh_->get_logger(), "success to connect servo start servr");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "wait to connect servo start server");
    if(i == 9){
      RCLCPP_ERROR_STREAM(nh_->get_logger(),
      "fail to connect moveit_servo." << "please launch 'servo.launch' at tm5_moveit_config pkg");
    }
  }

  for(int i; i < 10; i++){
    if(servo_stop_client_->wait_for_service(std::chrono::seconds(1))){
      RCLCPP_INFO_STREAM(nh_->get_logger(), "success to connect servo stop server");
      break;
    }
    RCLCPP_WARN_STREAM(nh_->get_logger(), "wait to connect servo stop server");
    if(i == 9){
      RCLCPP_ERROR_STREAM(nh_->get_logger(), "fail to connect moveit_servo" << "please 'servo.launch' tm5_moveit_config pkg");
    }
  }
}

void KeyboardServo::start_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call 'moveit_servo' start srv");
  auto future = servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  auto result = future.wait_for(std::chrono::seconds(1));
  if(result == std::future_status::ready){
    RCLCPP_INFO_STREAM(nh_->get_logger(), "success to start moveit_servo");
    future.get();
  } else {
    RCLCPP_ERROR_STREAM(nh_->get_logger(), "fail to start moveit_servo, excute without moveit_servo");
  }
}

void KeyboardServo::stop_moveit_servo()
{
  RCLCPP_INFO_STREAM(nh_->get_logger(), "call moveit_servo END srv");
  auto futrue = servo_stop_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  auto result = futrue.wait_for(std::chrono::seconds(1));
  if(result == std::future_status::ready){
    RCLCPP_INFO_STREAM(nh_->get_logger(), "success to stop moveit_servo");
    futrue.get();
  }
}

void KeyboardServo::pub()
{
  while (rclcpp::ok())
  {
    /* code */
    if(publish_task_){
      task_msg_.header.stamp = nh_->now();
      task_msg_.header.frame_id = TpcManipulationMoveit::BASE_FRAME_ID;
      arm_twist_pub_->publish(task_msg_);
      publish_task_ = false;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "task published");
      task_msg_.twist.linear.x = 0.0;
      task_msg_.twist.linear.y = 0.0;
      task_msg_.twist.linear.z = 0.0;
      task_msg_.twist.angular.x = 0.0;
      task_msg_.twist.angular.y = 0.0;
      task_msg_.twist.angular.z = 0.0;
    } else if(publish_joint_){
      joint_msg_.header.stamp = nh_->now();
      joint_msg_.header.frame_id = TpcManipulationMoveit::BASE_FRAME_ID;
      joint_pub_->publish(joint_msg_);
      publish_joint_ = false;
      RCLCPP_INFO_STREAM(nh_->get_logger(), "joint published");
      joint_msg_.joint_names.clear();
      joint_msg_.velocities.clear();
    }
    base_twist_pub_->publish(cmd_vel_);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
}

void KeyboardServo::spin()
{
  while (rclcpp::ok())
  {
    /* code */
    rclcpp::spin_some(nh_);
  }

}
