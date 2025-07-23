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

#ifndef TM5_TELEOP__TM5_TELEOP_HPP_
#define TM5_TELEOP__TM5_TELEOP_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>

#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>

#include <string>

#include "tm5_teleop/tpc_manipulation_moveit.hpp"

#define KEYCODE_SPACE 0x20

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_X 0x78
#define KEYCODE_Z 0x7A
#define KEYCODE_C 0x63
#define KEYCODE_F 0x66
#define KEYCODE_V 0x76

#define KEYCODE_AA 0x41
#define KEYCODE_DD 0x44
#define KEYCODE_SS 0x53
#define KEYCODE_XX 0x58
#define KEYCODE_ZZ 0x5A
#define KEYCODE_CC 0x43

#define KEYCODE_I 0x69

#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36

#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_T 0x74
#define KEYCODE_Y 0x79

#define KEYCODE_O 0x6F
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_P 0x70
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_ESC 0x1B
#define KEYCODE_M 0x6D
#define KEYCODE_N 0x6E
#define KEYCODE_COMMA 0x2C
#define KEYCODE_FULLSTOP 0x2E

#define KEYCODE_PLUS 0x2b
#define KEYCODE_MINUS 0x2d
#define KEYCODE_ENTER 0x0a

const char BASE_TWIST_TOPIC[] = "cmd_vel";
const char ARM_TWIST_TOPIC[] = "/servo_node/delta_twist_cmds";
const char ARM_JOINT_TOPIC[] = "/servo_node/delta_joint_cmds";

const size_t ROS_QUEUE_SIZE = 10;

// AMR 운동시 사용될 속도 변수들
// 이 변수들은 추후 AMR URDF 생성후 시뮬레이션 구성 후 사용 예정
// 메카넘 휠 조작에 맞게 수정 되어야 한다.
const double BASE_ANGULAR_VEL_MAX = 1.8;  // m/s
const double BASE_ANGULAR_VEL_STEP = 0.1;  // m/s

const double BASE_LINEAR_VEL_MAX = 0.26;  // m/s
const double BASE_LINEAR_VEL_STEP = 0.01;  // m/s
//const char JOINT_STATE_TOPIC[] = "joint_states";

// 다관절 로봇으 조작을 위한속도 변수 들
// 위에 AMR 조작 관련 속도 변수들과 함께 동적으로 속도를 조정 가능하게 수정 해야 한다.
const double ARM_TWIST_VEL = 0.02;  //m/s
const double ARM_TWIST_VEL_MAX = 0.4;
const double ARM_TWIST_VEL_STEP = (ARM_TWIST_VEL_MAX-ARM_TWIST_VEL) / 10 ;
const double ARM_JOINT_VEL= 0.05;  //rad/s
const double ARM_JOINT_VEL_MAX = 0.5;
const double ARM_JOINT_VEL_SETP = (ARM_JOINT_VEL_MAX - ARM_JOINT_VEL) / 10;

// Class for reading the keyboard inputs
class KeyboardReader
{
public:
  KeyboardReader();
  void readOne(char * c);
  void shutdown();

private:
  int kfd;
  struct termios cooked;
};

//Converts key-presses to Twist or Jog commands for Servo. in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  ~KeyboardServo();
  int keyLoop();

  void connect_moveit_servo();
  void start_moveit_servo();
  void stop_moveit_servo();

private:
  void spin();
  void pub();

  rclcpp::Node::SharedPtr nh_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_stop_client_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr base_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;

  geometry_msgs::msg::Twist cmd_vel_;
  geometry_msgs::msg::TwistStamped task_msg_;
  control_msgs::msg::JointJog joint_msg_;

  TpcManipulationMoveit::Manipulation manipulator;

  bool publish_task_;
  bool publish_joint_;

  double arm_twist_vel_;
  double arm_twist_vel_max_;
  double arm_joint_vel_;
  double arm_joint_vel_max_;

};

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  KeyboardServo KeyboardServo;
  signal(SIGINT, quit);

  int rc = KeyboardServo.keyLoop();

  input.shutdown();
  rclcpp::shutdown();

  return rc;
}

#endif
