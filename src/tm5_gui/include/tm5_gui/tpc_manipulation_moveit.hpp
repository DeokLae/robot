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

#ifndef TM5_GUI__TPC_MANIPULATION_MOVEIT_HPP_
#define TM5_GUI__TPC_MANIPULATION_MOVEIT_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <tuple>
#include <fstream>
#include <iostream>
#include <string>
#include <cmath>
#include <stdexcept>
#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.h>  //robot state 를 받는 서브스크라이버 의 매개 변수로 사용
#include <moveit/robot_state/robot_state.h>   //robot_state_ 사용
#include <moveit/robot_model_loader/robot_model_loader.h>   //robot_model_loader_ 사용
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <moveit/move_group_interface/move_group_interface.h>  //move_group_interface_ 사용
#include <moveit/planning_scene_interface/planning_scene_interface.h>  //planning_scene_interface_ 사용

//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>   // 앤드이팩터 기준 좌표 계산을 위해 tf2 를 사용
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> // 앤드이팩터 기준 좌표 계산을 위해 tf2 를 사용
#include <Eigen/Geometry>

namespace TpcManipulationMoveit{

  const char GROUP_NAME[] = "tm_arm";
  const char BASE_FRAME_ID[] = "base";
  const char TOOL_FRAME_ID[] = "link_6";

  const char JOINT_NAME_1[] = "joint_1";
  const char JOINT_NAME_2[] = "joint_2";
  const char JOINT_NAME_3[] = "joint_3";
  const char JOINT_NAME_4[] = "joint_4";
  const char JOINT_NAME_5[] = "joint_5";
  const char JOINT_NAME_6[] = "joint_6";
  const double ARM_JOINT_VEL = 1.0;
  const double ARM_TWIST_VEL = 1.0;

  const char ARM_JOINT_TOPIC[] = "/servo_node/delta_joint_cmds";
  const char ARM_TWIST_TOPIC[] = "servo_node/delta_twist_cmds";
  const char JOINT_STATE_TOPIC[] = "joint_states";
  const size_t ROS_QEUE_SIZE = 10;

  const double CARTE_VELOCITY_SCALE = 0.05;
  const double CARTE_VELOCITY_SCALE_MAX = 1.5;
  const double CARTE_VELOCITY_SCALE_STEP = (CARTE_VELOCITY_SCALE_MAX - CARTE_VELOCITY_SCALE) / 20;
  const double CARTE_ACCELERATION_SCALE = 0.05;
  const double CARTE_ACCELERATION_SCALE_MAX = 1.0;
  const double CARTE_ACCELERATION_SCALE_STEP = (CARTE_ACCELERATION_SCALE_MAX - CARTE_ACCELERATION_SCALE) / 20;
  const double JOINT_VELOCITY_SCALE = 0.01;
  const double JOINT_VELOCITY_SCALE_MAX = 1.0;
  const double JOINT_VELOCITY_SCALE_STEP = (JOINT_VELOCITY_SCALE_MAX - JOINT_VELOCITY_SCALE) / 20;
  const double jOINT_ACCELERATION_SCALE = 0.1;
  const double jOINT_ACCELERATION_SCALE_MAX = 1.0;
  const double jOINT_ACCELERATION_SCALE_STEP = (jOINT_ACCELERATION_SCALE_MAX - jOINT_ACCELERATION_SCALE) / 20;

  class Manipulation
  {
  private:
    /* data */

    //moveit::planning_interface::MoveGroupInterface::getCurrentJointValues 를 시뮬레이션 환경에서 쓸 수 없어
    // 로봇의 현재 상태를 받아오는 subscriber 를 생성 하기 위한 변수 들
    rclcpp::Node::SharedPtr manipulation_node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr robot_state_sub_;
    sensor_msgs::msg::JointState::SharedPtr cure_robot_state_;
    rclcpp::Time last_robot_state_update_time_;
    moveit::core::RobotModelPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    //std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;  // 윗줄 코드와 같은 거

    control_msgs::msg::JointJog joint_msg_;  // joint jog 토픽 메시지 선언
    geometry_msgs::msg::TwistStamped task_msg_;  //position move 토픽 메시지 선언
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;  //jog moving 을 위한 버플리셔 선언
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr task_pub_;  // pose moveing 을 위한 퍼블리셔 선언

    std::vector<geometry_msgs::msg::Pose> tmp_waypoints_;
    std::vector<geometry_msgs::msg::PoseStamped> arc_tmp_wapoints_;
    std::string task_frame_id_;
    std::vector<uint> check_Waypoint_num;
    std::vector<uint> pre_check_Waypoint_num;

    std::thread check_waypoint_thread_;
    std::mutex waypoint_mutex_;

    std::atomic<bool> thread_running_;
    std::atomic<bool> stop_thread_;
    std::atomic<bool> pause_thread_;
    std::atomic<int> current_job_index_;
    std::atomic<bool> publish_joint_;
    std::atomic<bool> publish_task_;
    std::atomic<bool> thread_step_run_;
    std::atomic<bool> thread_forward_;
    std::atomic<bool> thread_target_pose_;
    std::atomic<bool> thread_target_joint_;
    std::atomic<bool> thread_job_;
    std::atomic<bool> thread_step_comp_;
    std::atomic<bool> wapoint_check_thread_run_;
    std::atomic<bool> thread_arc_;

    size_t tmp_job_index_;
    void getCurePose();
    void getCureJoint();
    void savePoseToFile(const geometry_msgs::msg::PoseStamped& tmpPose, const std::string& filename);
    void saveJointToFile(const std::vector<double>& tmpJoint, const std::string& filename);
    void saveWaypointToFile(const geometry_msgs::msg::PoseStamped& tmpPose, const std::string& filename);
    void subCureRobotState(const sensor_msgs::msg::JointState::SharedPtr msg);
    void spin();
    void computeCartesianPath(std::vector<geometry_msgs::msg::Pose>& waypoints);
    void initJobInfo();
    bool handleJobThreadPauseOrStop(bool pause, bool stop, bool &step_wapoint_in, size_t i);
    void moveJobThread();
    void moveJobThreadBack();
    void moveToPoseCartesian(const geometry_msgs::msg::PoseStamped pose);
    void pub();
    void moveTargetPoseThread(const geometry_msgs::msg::PoseStamped pose);
    void moveTargetJoint(const std::vector<double> joint);
    void moveTargetJointThread(const std::vector<double> joint);
    std::vector<geometry_msgs::msg::PoseStamped> computeArcWaypointAtToolCoord(
            const double radius, const int point_num, const int angle_value,
            const int x_direction, const int y_direction, const int z_direction);
    std::vector<geometry_msgs::msg::PoseStamped> computeArcWaypointAtBaseCoord(
            const double radius, const int point_num, const double angle_value,
            const int x_direction, const int y_direction, const int z_direction);
    void moveTargetArcStartThread();
    void moveTargetArcBackThread();
    std::string makeJobFolder();
    void makeTmpWaypoints(geometry_msgs::msg::PoseStamped point);
    void runWaypoints(std::vector<geometry_msgs::msg::Pose>& waypoints);
    void checkWayPoint();
    void checkWayPointThread();
    void checkWayPointThreadStart();
    void checkWayPointThreadStop();

  public:
    std::vector<geometry_msgs::msg::PoseStamped> all_poses_values_;
    std::vector<std::vector<double>> all_joints_values_;
    std::vector<std::tuple<std::vector<double>, geometry_msgs::msg::PoseStamped, geometry_msgs::msg::Pose>> all_job_info_;
    std::vector<geometry_msgs::msg::Pose> all_wayPoints_;
    geometry_msgs::msg::PoseStamped cure_pose_values_;
    std::vector<double> cure_joint_values_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
    //moveit::planning_interface::MoveGroupInterfacePtr move_group_interface_; 이 구문과 동일
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    //moveit::planning_interface::PlanningSceneInterfacePtr planning_scene_interface_;  이 구문과 동일
    double cartesian_velocity_scale_;
    double cartesian_acceleration_scale_;
    double joint_velocity_scale_;
    double joint_acceleration_scale_;
    // 위 변수 들은 다른 클래스에서 직접 접근 하기 위해 public 으로 선언 한다.

    Manipulation();
    ~Manipulation();
    void readJobValuesFromFile(const std::string& filename);
    std::vector<geometry_msgs::msg::PoseStamped> getAllPose();
    std::vector<std::vector<double>> getAllJoint();
    std::vector<std::tuple<std::vector<double>, geometry_msgs::msg::PoseStamped, geometry_msgs::msg::Pose>> get_all_job_info();
    void moveJointJog(const std::string& joint_name, const double joint_vel);
    void stopJointJog();
    void moveTaskJog(int direc_num, const double twist_vel);
    void stopTaskJog();
    void movetargetjointStart(const std::vector<double> joint);
    void moveTargetJointStop();
    void moveTargetPoseStart(const geometry_msgs::msg::PoseStamped pose);
    void moveTargetPoseStop();
    void moveJobStart();
    void moveJobStartBack();
    void moveJobPause();
    void moveJobStop();
    void moveJobStep(bool next);
    void moveJobStepStop();
    void saveCurePoseToJob(const std::string& filename);
    void saveCureJointTojob(const std::string& filename);
    void saveCureWapointToJob(const std::string& filename);
    void moveEEposeCartesian(const double x, const double y, const double z);
    void moveArcWaypoints(const double radius, const int point_num, const int angle_value,
                          const int x_direction, const int y_direction, const int z_direction,
                          const std::string& frame_name);
    void saveArcWaypoits(const std::string& filename);
    void moveArcWaypointStop();
    void setFrameidforTaskMsg(const std::string& frame_name);
    int getCurrentJobIndex();
    void setCurrentJobIndex(int index);
  };
}
#endif
