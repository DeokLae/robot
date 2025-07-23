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

#include "tm5_gui/tpc_manipulation_moveit.hpp"
namespace TpcManipulationMoveit{

  Manipulation::Manipulation()
  {
    manipulation_node_ = std::make_shared<rclcpp::Node>("manipulation_node");

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(manipulation_node_, "robot_description");
    robot_model_ = robot_model_loader_->getModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    joint_pub_ = manipulation_node_->create_publisher<control_msgs::msg::JointJog>(ARM_JOINT_TOPIC,ROS_QEUE_SIZE);
    task_pub_ = manipulation_node_->create_publisher<geometry_msgs::msg::TwistStamped>(ARM_TWIST_TOPIC, ROS_QEUE_SIZE);
    robot_state_sub_ = manipulation_node_->create_subscription<sensor_msgs::msg::JointState>(
                JOINT_STATE_TOPIC,10, std::bind(&Manipulation::subCureRobotState, this, std::placeholders::_1));
    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(manipulation_node_, GROUP_NAME);
    cartesian_velocity_scale_ = CARTE_VELOCITY_SCALE;
    cartesian_acceleration_scale_ = CARTE_ACCELERATION_SCALE;
    joint_velocity_scale_ = JOINT_VELOCITY_SCALE;
    joint_acceleration_scale_ = jOINT_ACCELERATION_SCALE;

    std::thread(std::bind(&Manipulation::getCurePose, this)).detach();
    std::thread(std::bind(&Manipulation::getCureJoint, this)).detach();
    std::thread(std::bind(&Manipulation::spin, this)).detach();
    std::thread(std::bind(&Manipulation::pub, this)).detach();

    current_job_index_ = 0;
    thread_running_ = false;
    stop_thread_ = true;
    pause_thread_ = true;

    publish_joint_ = false;
    publish_task_ = false;
    thread_forward_ = true;
    thread_target_pose_ = false;
    thread_job_ = false;
    thread_step_run_ = false;
    thread_step_comp_ = false;
    wapoint_check_thread_run_ = false;
    thread_arc_ = false;
    tmp_job_index_ = 0;
  }

  Manipulation::~Manipulation()
  {
    rclcpp::shutdown();
  }

  //private function
  void Manipulation::getCurePose()
  {
    while (rclcpp::ok())
    {
      //auto joint_model_group = robot_state_->getJointModelGroup(GROUP_NAME);
      const Eigen::Isometry3d& end_effector_state = robot_state_->getGlobalLinkTransform(TOOL_FRAME_ID);
      //cure_pose_values_.header.stamp = rclcpp::Clock().now();
      cure_pose_values_.header.stamp = manipulation_node_->now();
      cure_pose_values_.header.frame_id = BASE_FRAME_ID;
      cure_pose_values_.pose.position.x = end_effector_state.translation().x();
      cure_pose_values_.pose.position.y = end_effector_state.translation().y();
      cure_pose_values_.pose.position.z = end_effector_state.translation().z();

      Eigen::Quaterniond quat(end_effector_state.rotation());
      cure_pose_values_.pose.orientation.x = quat.x();
      cure_pose_values_.pose.orientation.y = quat.y();
      cure_pose_values_.pose.orientation.z = quat.z();
      cure_pose_values_.pose.orientation.w = quat.w();

      //checkWayPoint();
    }
  }

  void Manipulation::getCureJoint()
  {
    while(rclcpp::ok())
    {
      auto joint_model_group = robot_state_->getJointModelGroup(GROUP_NAME);
      std::vector<double> joint_group_positions;
      //robot_state_->copyJointGroupPositions(joint_model_group, joint_group_positions);
      robot_state_->copyJointGroupPositions(joint_model_group, cure_joint_values_);
      //cure_joint_values_ = joint_group_positions;
      //cure_joint_values_ = move_group_interface_->getCurrentJointValues(); 시뮬레이션 환경에서 getCurrentJointValeus() 함수가 동작 하지 않는다.
    }
  }

  void Manipulation::savePoseToFile(const geometry_msgs::msg::PoseStamped& tmpPose, const std::string& filename)
  {
      std::string filePath = makeJobFolder() + "/" + filename + ".job";

      std::ofstream outfile(filePath, std::ios::app);
    if(!outfile.is_open())
    {
      std::cerr << "\033[31m" << "savePoseToFile function in manipulation CLASS ERROR :: (Read Job File) Can not open file!!" << "\033[0m" << std::endl;
      return;
    }
    outfile << "pose ";
    outfile << tmpPose.pose.position.x << " ";
    outfile << tmpPose.pose.position.y << " ";
    outfile << tmpPose.pose.position.z << " ";
    outfile << tmpPose.pose.orientation.x << " ";
    outfile << tmpPose.pose.orientation.y << " ";
    outfile << tmpPose.pose.orientation.z << " ";
    outfile << tmpPose.pose.orientation.w << std::endl;
    outfile.close();
  }

  void Manipulation::saveJointToFile(const std::vector<double>& tmpJoint, const std::string& filename)
  {
    std::string filePath = makeJobFolder() + "/" + filename + ".job";

      std::ofstream outfile(filePath, std::ios::app);
    if(!outfile.is_open())
    {
      std::cerr << "\033[31m" << "saveJointToFile function in manipulation CLASS ERROR :: (Read Job File) Can not open file!!" << "\033[0m" << std::endl;
      return;
    }
    outfile << "joint ";
    for(size_t i = 0; i < tmpJoint.size(); ++i)
    {
      outfile << tmpJoint[i];
      if(i != tmpJoint.size() -1){
        outfile << " ";
      }
    }
    outfile << std::endl;
    outfile.close();
  }

  void Manipulation::saveWaypointToFile(const geometry_msgs::msg::PoseStamped& tmpPose, const std::string& filename)
  {
    std::string filePath = makeJobFolder() + "/" + filename + ".job";

      std::ofstream outfile(filePath, std::ios::app);
    if(!outfile.is_open())
    {
      std::cerr << "\033[31m" << "savePoseToFile function in manipulation CLASS ERROR :: (Read Job File) Can not open file!!" << "\033[0m" << std::endl;
      return;
    }
    outfile << "waypoint ";
    outfile << tmpPose.pose.position.x << " ";
    outfile << tmpPose.pose.position.y << " ";
    outfile << tmpPose.pose.position.z << " ";
    outfile << tmpPose.pose.orientation.x << " ";
    outfile << tmpPose.pose.orientation.y << " ";
    outfile << tmpPose.pose.orientation.z << " ";
    outfile << tmpPose.pose.orientation.w << std::endl;
    outfile.close();
  }

  void Manipulation::subCureRobotState(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    cure_robot_state_ = msg;
    last_robot_state_update_time_ = manipulation_node_->now();
    robot_state_->setVariablePositions(cure_robot_state_->name, cure_robot_state_->position);
    move_group_interface_->setStartState(*robot_state_);
  }

  void Manipulation::spin()
  {
    while (rclcpp::ok())
    {
      /* code */
      rclcpp::spin_some(manipulation_node_);
    }
  }

  void Manipulation::computeCartesianPath(std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    //std::vector<geometry_msgs::msg::Pose> waypoints;
    moveit_msgs::msg::RobotTrajectory trajectory;

    move_group_interface_->setPlanningTime(10.0);
    move_group_interface_->setNumPlanningAttempts(10);

    double fraction = move_group_interface_-> computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if(fraction >= 0.8)
    {
      for(auto& point : trajectory.joint_trajectory.points)
      {
        rclcpp::Duration time_from_start(point.time_from_start);
        int64_t total_nanoseconds = time_from_start.nanoseconds();
        total_nanoseconds = static_cast<int64_t>(total_nanoseconds / cartesian_velocity_scale_);
        point.time_from_start.sec = static_cast<int32_t>(total_nanoseconds / 1000000000);
        point.time_from_start.nanosec = static_cast<int32_t>(total_nanoseconds % 1000000000);

        if(!point.velocities.empty())
        {
          for(auto& velocity : point.velocities)
          {
            velocity = velocity*cartesian_velocity_scale_;
          }
        }else
        {
          RCLCPP_WARN(manipulation_node_->get_logger(), "velocity vector is empty");
          point.velocities.resize(trajectory.joint_trajectory.joint_names.size(), 0.0);
        }

        if(!point.accelerations.empty())
        {
          for(auto& acceleration : point.accelerations)
          {
            acceleration = acceleration*cartesian_acceleration_scale_;
          }
        }else
        {
          RCLCPP_WARN(manipulation_node_->get_logger(), "acceleration vector is empty");
          point.accelerations.resize(trajectory.joint_trajectory.joint_names.size(), 0.0);
        }
      }
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      plan.trajectory_ = trajectory;
      move_group_interface_->execute(plan);
      RCLCPP_INFO(manipulation_node_->get_logger(), "Catesian path executed successfully");
    } else
    {
      RCLCPP_WARN(manipulation_node_->get_logger(), "Failed to plan Cartesian path (%.2f%% achieved", fraction*100.0);
    }
  }

  void Manipulation::initJobInfo()
  {
    if(!all_job_info_.empty())
    {
      for(size_t i = 0; i < all_job_info_.size(); i++)
      {
        all_job_info_.at(i) = std::make_tuple(std::vector<double>(), geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::Pose());
      }
    }
    if(!all_joints_values_.empty())
    {
      for(size_t i = 0; i < all_joints_values_.size(); i ++)
      {
        all_joints_values_.at(i) = std::vector<double>();
      }
    }

    if(!all_poses_values_.empty())
    {
      for(size_t i = 0; i < all_poses_values_.size(); i ++)
      {
        all_poses_values_.at(i) = geometry_msgs::msg::PoseStamped();
      }
    }

    if(!all_wayPoints_.empty())
    {
      for(size_t i = 0; i < all_wayPoints_.size(); i ++)
      {
        all_wayPoints_.at(i) = geometry_msgs::msg::Pose();
      }
    }
  }

  bool Manipulation::handleJobThreadPauseOrStop(bool pause, bool stop, bool &step_wapoint_in, size_t i)
  {
      if(pause){
        if(!step_wapoint_in)
        {
            current_job_index_ = i;
        }
        thread_running_ = false;
        move_group_interface_->stop();
        return true;
      }
      if(stop){
        current_job_index_ = 0;
        check_Waypoint_num.clear();
        pre_check_Waypoint_num.clear();
        thread_running_ = false;
        move_group_interface_->stop();
        return true;
      }
      return false;
  }

  void Manipulation::moveJobThread()
  {
    thread_running_ = true;
    std::vector<double> tmpJointValue;
    geometry_msgs::msg::PoseStamped tmpPoseValue;

    size_t i = current_job_index_;
    bool step_wapoint_move = false;
    bool step_wapoint_in = false;

    for(; i < all_job_info_.size(); i++)
    {
      thread_step_comp_ = false;

      if(handleJobThreadPauseOrStop(pause_thread_, stop_thread_, step_wapoint_in, i))
      {
          return;
      }
      RCLCPP_INFO(manipulation_node_->get_logger(), "MOVE  TO %ld_ST JOB MOTION", i);

      if(!std::get<0>(all_job_info_[i]).empty())
      {
        step_wapoint_move = false;
        step_wapoint_in = false;
        tmpJointValue = std::get<0>(all_job_info_[i]);
        RCLCPP_INFO(manipulation_node_->get_logger(), "Joint Move start!!");

        moveTargetJoint(tmpJointValue);
      } else if (std::get<1>(all_job_info_[i]).pose.position.x)
      {
        step_wapoint_move = false;
        step_wapoint_in = false;
        tmpPoseValue = std::get<1>(all_job_info_[i]);

        RCLCPP_INFO(manipulation_node_->get_logger(), "Pose Move start!!");
        moveToPoseCartesian(tmpPoseValue);
      } else if (std::get<2>(all_job_info_[i]).position.x)
      {
        if(!step_wapoint_in)
        {
            current_job_index_ = i;
            RCLCPP_WARN(manipulation_node_->get_logger(), "init current_job_index to  :: %d", static_cast<int>(current_job_index_));
        }
        step_wapoint_in = true;
        if(std::get<2>(all_job_info_[i+1]).position.x)
        {
            RCLCPP_INFO(manipulation_node_->get_logger(), "next position x value of wapoint is %f", std::get<2>(all_job_info_[i+1]).position.x);
            tmpPoseValue.pose = std::get<2>(all_job_info_[i]);
            makeTmpWaypoints(tmpPoseValue);
        } else{
            RCLCPP_INFO(manipulation_node_->get_logger(), "There is no next position x value of wapoint");
            tmpPoseValue.pose = std::get<2>(all_job_info_[i]);
            makeTmpWaypoints(tmpPoseValue);
            RCLCPP_INFO(manipulation_node_->get_logger(), "waypoints move start!!");
            checkWayPointThreadStart();
            runWaypoints(tmp_waypoints_);
            tmp_waypoints_.clear();
            checkWayPointThreadStop();
            step_wapoint_move = true;
        }
      }
      if(handleJobThreadPauseOrStop(pause_thread_, stop_thread_, step_wapoint_in, i))
      {
          return;
      }

      if(thread_step_run_)
      {
        thread_running_ = false;
        thread_step_comp_ = true;
        if(!step_wapoint_in)
        {
            if(i < all_job_info_.size() - 1)
            {
                current_job_index_ = i;
            }else
            {
                current_job_index_ = 0;
            }
            return;
        } else if(step_wapoint_move)
        {
            current_job_index_ = i;
            return;
        }
      }

      if(!step_wapoint_in)
      {
          current_job_index_ = i;
      }
    }
    current_job_index_ = 0;
    check_Waypoint_num.clear();
    pre_check_Waypoint_num.clear();
    thread_running_ = false;
    thread_job_ = false;
    RCLCPP_WARN(manipulation_node_->get_logger(), "Motion Cycle Complete!!");
  }

  void Manipulation::moveJobThreadBack()
  {
      thread_running_ = true;
      thread_job_ = true;
      std::vector<double> tmpJointValue;
      geometry_msgs::msg::PoseStamped tmpPoseValue;
      bool step_wapoint_move = false;
      bool step_wapoint_in = false;

      int i = current_job_index_;

      for(; i >= 0; i--)
      {
        thread_step_comp_ = false;
        if(handleJobThreadPauseOrStop(pause_thread_, stop_thread_, step_wapoint_in, i))
        {
            return;
        }
        RCLCPP_INFO(manipulation_node_->get_logger(), "MOVE  TO %d_ST JOB MOTION", i);

        if(!std::get<0>(all_job_info_[i]).empty())
        {
          step_wapoint_move = false;
          step_wapoint_in = false;
          tmpJointValue = std::get<0>(all_job_info_[i]);
          RCLCPP_INFO(manipulation_node_->get_logger(), "Joint Move start!!");

          moveTargetJoint(tmpJointValue);
        } else if (std::get<1>(all_job_info_[i]).pose.position.x)
        {
          step_wapoint_move = false;
          step_wapoint_in = false;
          tmpPoseValue = std::get<1>(all_job_info_[i]);
          RCLCPP_INFO(manipulation_node_->get_logger(), "Pose Move start!!");

          moveToPoseCartesian(tmpPoseValue);
        } else if (std::get<2>(all_job_info_[i]).position.x)
        {
          if(!step_wapoint_in)
          {
              current_job_index_ = i;
              RCLCPP_WARN(manipulation_node_->get_logger(), "init current_job_index to  :: %d", static_cast<int>(current_job_index_));
          }
          step_wapoint_in = true;
          if(std::get<2>(all_job_info_[i-1]).position.x)
          {
              RCLCPP_INFO(manipulation_node_->get_logger(), "next position x value of wapoint is %f", std::get<2>(all_job_info_[i-1]).position.x);
              tmpPoseValue.pose = std::get<2>(all_job_info_[i]);
              makeTmpWaypoints(tmpPoseValue);
          } else{
              RCLCPP_INFO(manipulation_node_->get_logger(), "There is no next position x value of wapoint");
              tmpPoseValue.pose = std::get<2>(all_job_info_[i]);
              makeTmpWaypoints(tmpPoseValue);
              RCLCPP_INFO(manipulation_node_->get_logger(), "waypoints move start!!");
              checkWayPointThreadStart();
              runWaypoints(tmp_waypoints_);
              tmp_waypoints_.clear();
              checkWayPointThreadStop();
              step_wapoint_move = true;
          }
        }

        if(handleJobThreadPauseOrStop(pause_thread_, stop_thread_, step_wapoint_in, i))
        {
            return;
        }

        if(thread_step_run_){
          thread_running_ = false;
          thread_job_ = false;
          thread_step_comp_ = true;
          if(!step_wapoint_in)
          {
              if(i > 0)
              {
                  current_job_index_ = i;
              }else
              {
                  current_job_index_ = 0;
              }
              return;
          } else if(step_wapoint_move)
          {
              current_job_index_ = i;
              return;
          }
        }

        if(!step_wapoint_in)
        {
            current_job_index_ = i;
        }
      }
      current_job_index_ = 0;
      thread_running_ = false;
      thread_job_ = false;
      RCLCPP_WARN(manipulation_node_->get_logger(), "Motion Cycle Complete!!");
  }

  void Manipulation::moveToPoseCartesian(const geometry_msgs::msg::PoseStamped pose)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose.pose);

    computeCartesianPath(waypoints);
  }

  void Manipulation::pub()
  {
      while (rclcpp::ok())
      {
        /* code */
        if(publish_joint_)
        {
          joint_msg_.header.stamp = manipulation_node_->now();
          joint_msg_.header.frame_id = BASE_FRAME_ID;
          joint_pub_->publish(joint_msg_);
        }
        else if(publish_task_)
        {
          task_msg_.header.stamp = manipulation_node_->now();
          task_msg_.header.frame_id = task_frame_id_;
          task_pub_->publish(task_msg_);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(10));
      }
  }

  void Manipulation::moveTargetPoseThread(const geometry_msgs::msg::PoseStamped pose)
  {
      thread_running_ = true;
      thread_target_pose_ = true;
      moveToPoseCartesian(pose);
      thread_running_ = false;
      thread_target_pose_ = false;
  }

  void Manipulation::moveTargetJoint(const std::vector<double> joint)
  {
    auto joint_model_group = robot_state_->getJointModelGroup(GROUP_NAME);
    std::vector<double> joint_group_positions;
    robot_state_->copyJointGroupPositions(joint_model_group, joint_group_positions);
    //매개변수로 전달 받은 joint 값을 저장 하기 위한 백터 객체를 선언.
    //moveit::planning_interface::MoveGroupInterface::getCurrentJointValues 를 시뮬레이션 환경에서 쓸 수 없어서

    joint_group_positions = joint;

    move_group_interface_->setMaxVelocityScalingFactor(joint_velocity_scale_);
    move_group_interface_->setMaxAccelerationScalingFactor(joint_acceleration_scale_);
    move_group_interface_->setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //bool success = (move_group_interface_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    bool success = (move_group_interface_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if(success)
    {
      /*robot_trajectory::RobotTrajectory rt(robot_model_, GROUP_NAME);
      rt.setRobotTrajectoryMsg(*robot_state_, my_plan.trajectory_);
      double scale = 1.5;
      trajectory_processing::IterativeParabolicTimeParameterization time_param;
      bool success = time_param.computeTimeStamps(rt, scale);
      rt.getRobotTrajectoryMsg(my_plan.trajectory_);*/
      // joint 값으로 이동 할 때도 trajectory 를 생성해서 움직이는거 가능 한거 같다.
      // 근데 wapoint 를 쓰는건 아니다.
      move_group_interface_->execute(plan);
      //move_group_interface_->asyncExecute(my_plan);
    } else
    {
      RCLCPP_ERROR(manipulation_node_->get_logger(), "Planning failed!");
    }
  }

  void Manipulation::moveTargetJointThread(const std::vector<double> joint)
  {
      thread_running_ = true;
      thread_target_joint_ = true;
      moveTargetJoint(joint);
      thread_running_ = false;
      thread_target_joint_ = false;
  }

  std::vector<geometry_msgs::msg::PoseStamped> Manipulation::computeArcWaypointAtToolCoord(
          const double radius, const int point_num, const int angle_value, const int x_direction,
          const int y_direction, const int z_direction)
  {
    geometry_msgs::msg::PoseStamped startPose;
    startPose.header.frame_id = TOOL_FRAME_ID;
    startPose.pose = cure_pose_values_.pose;

    geometry_msgs::msg::PoseStamped targetPose;
    targetPose.header.frame_id = TOOL_FRAME_ID;
    targetPose.pose = startPose.pose;

    std::vector<geometry_msgs::msg::PoseStamped> tmpWaypoints;

    tf2::Transform currentTransform;
    tf2::fromMsg(cure_pose_values_.pose, currentTransform);

    tf2::Transform transform;

    tf2::Transform targetTransform;

    for(int i = 0; i <= point_num*(angle_value/360.0); ++i)
    {
      //double angle = 2*M_PI*i / point_num;
      double angle = 2*M_PI*i / point_num;
      if(x_direction == 0)
      {
        transform.setOrigin(tf2::Vector3(-x_direction*(radius * cos(angle) - radius),
                                     y_direction*(radius * sin(angle)), z_direction*(radius * cos(angle) - radius)));
      } else{
        transform.setOrigin(tf2::Vector3(-x_direction*(radius * cos(angle) - radius),
                                     y_direction*(radius * sin(angle)), z_direction*(radius * sin(angle))));
      }
      transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      targetTransform = currentTransform * transform;
      tf2::toMsg(targetTransform, targetPose.pose);
      tmpWaypoints.push_back(targetPose);
    }
    return tmpWaypoints;
    //computeCartesianPath(waypoints);
  }

  std::vector<geometry_msgs::msg::PoseStamped> Manipulation::computeArcWaypointAtBaseCoord(
          const double radius, const int point_num, const double angle_value, const int x_direction,
          const int y_direction, const int z_direction)
  {

    geometry_msgs::msg::PoseStamped startPose;
    startPose.header.frame_id = BASE_FRAME_ID;
    startPose.pose = cure_pose_values_.pose;
    geometry_msgs::msg::PoseStamped targetPose;
    targetPose.header.frame_id = BASE_FRAME_ID;
    targetPose.pose = startPose.pose;

    std::vector<geometry_msgs::msg::PoseStamped> tmpWaypoints;

    double angle_step = ((angle_value / 360.0) * 2 * M_PI) / point_num;
    RCLCPP_INFO(manipulation_node_->get_logger(), "angle_step value is %f", angle_step);

    for(int i = 0; i <= point_num*(angle_value/360.0); ++i)
    {
      double angle = (2*M_PI*(1.0/4)) + (2*M_PI*i / point_num);
      targetPose.pose.position.x = startPose.pose.position.x - x_direction*((radius * cos(angle)));
      targetPose.pose.position.y = startPose.pose.position.y - y_direction*((radius * sin(angle)) - radius);
      if(y_direction == 0)
      {
        targetPose.pose.position.z = startPose.pose.position.z - z_direction*((radius * sin(angle)) - radius);
      } else if(x_direction == 0)
      {
        targetPose.pose.position.z = startPose.pose.position.z - z_direction*((radius * cos(angle)));
      } else
      {
        targetPose.pose.position.z = startPose.pose.position.z - z_direction*((radius * cos(angle)));
      }
      targetPose.pose.orientation.x = startPose.pose.orientation.x;
      targetPose.pose.orientation.y = startPose.pose.orientation.y;
      targetPose.pose.orientation.z = startPose.pose.orientation.z;
      targetPose.pose.orientation.w = startPose.pose.orientation.w;
      RCLCPP_ERROR(manipulation_node_->get_logger(), "first angle value is :: %f", angle);
      tmpWaypoints.push_back(targetPose);
    }
    return tmpWaypoints;
    //computeCartesianPath(tmpWaypoints);
  }

  void Manipulation::moveTargetArcStartThread()
  {
      thread_running_ = true;
      thread_arc_ = true;
      std::vector<geometry_msgs::msg::Pose> tmpWaypoints;
      for(size_t i = 0; i < arc_tmp_wapoints_.size()-1; ++i)
      {
          tmpWaypoints.push_back(arc_tmp_wapoints_[i].pose);
      }
      computeCartesianPath(tmpWaypoints);
      thread_running_ = false;
      thread_arc_ = false;
  }

  void Manipulation::moveTargetArcBackThread()
  {
      std::vector<geometry_msgs::msg::Pose> tmpWaypoints;
      int i= arc_tmp_wapoints_.size() - 1;
      for(; i >= 0; --i)
      {
          tmpWaypoints.push_back(arc_tmp_wapoints_[i].pose);
      }
      computeCartesianPath(tmpWaypoints);
  }

  std::string Manipulation::makeJobFolder()
  {
      std::string folderPath = "jobfiles";
      if(!std::filesystem::exists(folderPath))
      {
          std::filesystem::create_directory(folderPath);
      }
      return folderPath;
  }

  void Manipulation::makeTmpWaypoints(geometry_msgs::msg::PoseStamped point)
  {
    RCLCPP_INFO(manipulation_node_->get_logger(), "add waypoint");
    tmp_waypoints_.push_back(point.pose);
  }

  void Manipulation::runWaypoints(std::vector<geometry_msgs::msg::Pose>& waypoints)
  {
    std::vector<geometry_msgs::msg::Pose> tmpWayPoints;
    tmpWayPoints = waypoints;
    RCLCPP_INFO(manipulation_node_->get_logger(), "run waypoint");
    computeCartesianPath(tmpWayPoints);
  }

  void Manipulation::checkWayPoint()
  {
      const double epsilon = 5e-3;
      std::vector<uint> tmp_check_num;
      for(size_t i = 0; i <all_job_info_.size(); i++)
      {
          if(std::get<2>(all_job_info_[i]).position.x)
          {
              std::vector<double> cureValue = {cure_pose_values_.pose.position.x, cure_pose_values_.pose.position.y,
                                               cure_pose_values_.pose.position.z, cure_pose_values_.pose.orientation.x,
                                               cure_pose_values_.pose.orientation.y, cure_pose_values_.pose.orientation.z,
                                               cure_pose_values_.pose.orientation.w};

              std::vector<double> waypointValue = {std::get<2>(all_job_info_[i]).position.x, std::get<2>(all_job_info_[i]).position.y,
                                                   std::get<2>(all_job_info_[i]).position.z, std::get<2>(all_job_info_[i]).orientation.x,
                                                   std::get<2>(all_job_info_[i]).orientation.y, std::get<2>(all_job_info_[i]).orientation.z,
                                                   std::get<2>(all_job_info_[i]).orientation.w};
              bool match = true;
              for(size_t j = 0; j < 7; j++)
              {
                  if(fabs(cureValue[j] - waypointValue[j]) > epsilon){
                      match = false;
                      break;
                  }
              }
              if (match)
              {
                  tmp_check_num.push_back(i);
              }
          }
      }
      if(tmp_check_num.empty())
      {
          return;
      }

      if(pre_check_Waypoint_num == tmp_check_num)
      {
          return;
      } else
      {
          pre_check_Waypoint_num.clear();
          pre_check_Waypoint_num = tmp_check_num;
          tmp_check_num.erase(
              std::remove_if(
                  tmp_check_num.begin(),
                  tmp_check_num.end(),
                  [&](int value) {
                      return std::find(check_Waypoint_num.begin(), check_Waypoint_num.end(), value) != check_Waypoint_num.end();
                  }
              ),
              tmp_check_num.end()
          );

          if(!tmp_check_num.empty())
          {
              RCLCPP_ERROR(manipulation_node_->get_logger(), "current_job_index_ is :: %d", static_cast<int>(current_job_index_));
              if(thread_forward_)
              {
                  for(size_t num = 0; num < tmp_check_num.size(); num ++)
                  {
                      if(current_job_index_ <= static_cast<int>(tmp_check_num[num]))
                      {
                          check_Waypoint_num.push_back(tmp_check_num[num]);
                          RCLCPP_WARN(manipulation_node_->get_logger(), "check waypoint is thread_forward_ %d", tmp_check_num[num]);
                          break;
                      }
                  }
                  current_job_index_ = check_Waypoint_num[check_Waypoint_num.size() - 1] + 1;
              } else
              {
                  for(int num = tmp_check_num.size() - 1; num >= 0; num --)
                  {
                      if(current_job_index_ >= static_cast<int>(tmp_check_num[num]))
                      {
                          check_Waypoint_num.push_back(tmp_check_num[num]);
                          RCLCPP_WARN(manipulation_node_->get_logger(), "check waypoint is back %d", tmp_check_num[num]);
                          break;
                      }
                  }
                  current_job_index_ = check_Waypoint_num[check_Waypoint_num.size() - 1] - 1;
              }
              for(size_t n = 0; n < check_Waypoint_num.size(); n ++)
              {
                  RCLCPP_ERROR(manipulation_node_->get_logger(), "checked waypont num %ld is :::: %d", n, check_Waypoint_num[n]);
              }
          }
      }
  }

  void Manipulation::checkWayPointThread()
  {
      while(wapoint_check_thread_run_)
      {
          checkWayPoint();
          //std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
  }

  void Manipulation::checkWayPointThreadStart()
  {
      if(check_waypoint_thread_.joinable())
      {
          RCLCPP_WARN(manipulation_node_->get_logger(), "already run check_waypoint_thread_!!");
      } else
      {
          wapoint_check_thread_run_ = true;
          check_waypoint_thread_ = std::thread(&Manipulation::checkWayPointThread, this);
      }
  }

  void Manipulation::checkWayPointThreadStop()
  {
      {
          std::lock_guard<std::mutex> lock(waypoint_mutex_);
          wapoint_check_thread_run_ = false;
      }
      if(check_waypoint_thread_.joinable())
      {
          //wapoint_check_thread_run_ = false;
          RCLCPP_ERROR(manipulation_node_->get_logger(), "waypoint check stop!!");
          check_waypoint_thread_.join();
      }
  }


  //public function
  void Manipulation::readJobValuesFromFile(const std::string& filename)
  {
    initJobInfo();
    all_joints_values_.clear();
    all_poses_values_.clear();
    all_job_info_.clear();
    all_wayPoints_.clear();
    std::string filePath = makeJobFolder() + "/" + filename + ".job";
    std::ifstream infile(filePath);
    if(!infile){
      std::cerr << "\033[31m" << "readJobValuesFromFile function in manipulation CLASS ERROR :: (Read Job File) Can not open file!!" << "\033[0m" << std::endl;
      return ;
    }

    std::string line;
    while (std::getline(infile, line)){
      std::istringstream iss(line);
      std::vector<double> positions;
      geometry_msgs::msg::PoseStamped pose;
      std::string firstworld;
      double value;
      iss >> firstworld;
      if(firstworld == "joint"){
        while (iss >> value)
        {
          positions.push_back(value);
        }
        if(!positions.empty())
        {
          all_joints_values_.push_back(positions);
          all_job_info_.emplace_back(positions, geometry_msgs::msg::PoseStamped(), geometry_msgs::msg::Pose());
        }
      } else if (firstworld == "pose")
      {
        //pose.header.stamp = rclcpp::Clock().now();
        pose.header.stamp = manipulation_node_->now();
        pose.header.frame_id = BASE_FRAME_ID;
        iss >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z
              >> pose.pose.orientation.x >> pose.pose.orientation.y >> pose.pose.orientation.z >> pose.pose.orientation.w;
        all_poses_values_.push_back(pose);
        all_job_info_.emplace_back(std::vector<double>(), pose, geometry_msgs::msg::Pose());
      } else if (firstworld == "waypoint")
      {
        pose.header.stamp = manipulation_node_->now();
        pose.header.frame_id = BASE_FRAME_ID;
        iss >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z
              >> pose.pose.orientation.x >> pose.pose.orientation.y >> pose.pose.orientation.z >> pose.pose.orientation.w;
        all_wayPoints_.push_back(pose.pose);
        all_job_info_.emplace_back(std::vector<double>(), geometry_msgs::msg::PoseStamped(), pose.pose);
      }
    }
  }

  std::vector<geometry_msgs::msg::PoseStamped> Manipulation::getAllPose()
  {
    return all_poses_values_;
  }

  std::vector<std::vector<double>> Manipulation::getAllJoint()
  {
    return all_joints_values_;
  }

  std::vector<std::tuple<std::vector<double>, geometry_msgs::msg::PoseStamped, geometry_msgs::msg::Pose>> Manipulation::get_all_job_info()
  {
    return all_job_info_;
  }

  void Manipulation::moveJointJog(const std::string& joint_name, const double joint_vel)
  {
    if(!thread_running_)
    {
        joint_msg_.joint_names.push_back(joint_name);
        joint_msg_.velocities.push_back(joint_vel);
        publish_joint_ = true;
    }
  }

  void Manipulation::stopJointJog()
  {
      joint_msg_.joint_names.clear();
      joint_msg_.velocities.clear();
      publish_joint_ = false;
  }

  void Manipulation::moveTaskJog(int direc_num, const double twist_vel)
  {
      switch (direc_num)
      {
        case 0:
          task_msg_.twist.linear.x = twist_vel;
          break;
        case 1:
          task_msg_.twist.linear.y = twist_vel;
          break;
        case 2:
          task_msg_.twist.linear.z = twist_vel;
          break;
        case 3:
          task_msg_.twist.angular.x = twist_vel;
          break;
        case 4:
          task_msg_.twist.angular.y = twist_vel;
          break;
        case 5:
          task_msg_.twist.angular.z = twist_vel;
          break;
      }
      publish_task_ = true;
  }

  void Manipulation::stopTaskJog()
  {
    task_msg_.twist.linear.x = 0.0;
    task_msg_.twist.linear.y = 0.0;
    task_msg_.twist.linear.z = 0.0;
    task_msg_.twist.angular.x = 0.0;
    task_msg_.twist.angular.y = 0.0;
    task_msg_.twist.angular.z = 0.0;
    publish_task_ = false;
  }

  void Manipulation::movetargetjointStart(const std::vector<double> joint)
  {
      if(!thread_running_)
      {
          moveTargetJointThread(joint);
      }
  }

  void Manipulation::moveTargetJointStop()
  {
      if(thread_target_joint_)
      {
          move_group_interface_->stop();
          thread_target_joint_ = false;
      }
  }

  void Manipulation::moveTargetPoseStart(const geometry_msgs::msg::PoseStamped pose)
  {
      if(!thread_running_)
      {
          std::thread movePoseTargetThread([=](){
              moveTargetPoseThread(pose);
          });
          movePoseTargetThread.detach();
      }
  }

  void Manipulation::moveTargetPoseStop()
  {
      if(thread_target_pose_)
      {
          move_group_interface_->stop();
          thread_target_pose_ = false;
      }
  }

  void Manipulation::moveJobStart()
  {
    RCLCPP_INFO(manipulation_node_->get_logger(),"call the thread");
    if(!thread_running_)
    {
      RCLCPP_INFO(manipulation_node_->get_logger(),"thread start!!");
      pause_thread_ = false;
      stop_thread_ = false;
      thread_step_run_ = false;
      thread_job_ = true;
      if(!thread_forward_)
      {
          current_job_index_ += 1;
          check_Waypoint_num.clear();
          pre_check_Waypoint_num.clear();
      }
      thread_forward_ = true;
      std::thread(&Manipulation::moveJobThread, this).detach();
    }
  }

  void Manipulation::moveJobStartBack()
  {
      RCLCPP_INFO(manipulation_node_->get_logger(),"call the thread");
      if(!thread_running_)
      {
        RCLCPP_INFO(manipulation_node_->get_logger(),"thread start!!");
        pause_thread_ = false;
        stop_thread_ = false;
        thread_step_run_ = false;
        if(thread_forward_)
        {
            current_job_index_ -= 1;
            check_Waypoint_num.clear();
            pre_check_Waypoint_num.clear();
        }
        thread_forward_ = false;
        std::thread(&Manipulation::moveJobThreadBack, this).detach();
      }
  }

  void Manipulation::moveJobPause()
  {
    if(thread_job_)
    {
        pause_thread_ = true;
        move_group_interface_->stop();
        thread_job_ = false;
    }
  }

  void Manipulation::moveJobStop()
  {
      if(thread_job_)
      {
          stop_thread_ = true;
          move_group_interface_->stop();
          thread_job_ = false;
      }
  }

  void Manipulation::moveJobStep(bool next)
  {
      if(!thread_running_)
      {
        pause_thread_ = false;
        stop_thread_ = false;
        thread_step_run_ = true;
        if(next)
        {
            if(!thread_forward_)
            {
                current_job_index_ += 1;
                check_Waypoint_num.clear();
                pre_check_Waypoint_num.clear();
            } else if(thread_step_comp_)
            {
                current_job_index_ += 1;
            }

            thread_forward_= true;
            std::thread(&Manipulation::moveJobThread, this).detach();
        }else
        {
            if(thread_forward_)
            {
                current_job_index_ -= 1;
                check_Waypoint_num.clear();
                pre_check_Waypoint_num.clear();
            } else if(thread_step_comp_)
            {
                current_job_index_ -= 1;
            }
            thread_forward_= false;
            std::thread(&Manipulation::moveJobThreadBack, this).detach();
        }
      }
  }

  void Manipulation::moveJobStepStop()
  {
      if(thread_step_run_)
      {
          pause_thread_ = true;
          move_group_interface_->stop();
          thread_step_run_ = false;
          thread_running_ = false;
      }
  }

  void Manipulation::saveCurePoseToJob(const std::string& filename)
  {
    RCLCPP_INFO(manipulation_node_->get_logger(), "current pose values are =============================================");
    RCLCPP_INFO(manipulation_node_->get_logger(), "Current pose: position (x: %f, y: %f, z: %f), orientation (x: %f, y: %f, z: %f, w: %f)",
                  cure_pose_values_.pose.position.x,
                  cure_pose_values_.pose.position.y,
                  cure_pose_values_.pose.position.z,
                  cure_pose_values_.pose.orientation.x,
                  cure_pose_values_.pose.orientation.y,
                  cure_pose_values_.pose.orientation.z,
                  cure_pose_values_.pose.orientation.w);
    savePoseToFile(cure_pose_values_, filename);
  }

  void Manipulation::saveCureJointTojob(const std::string& filename)
  {
    std::vector<std::string> joint_group_name;
    joint_group_name = move_group_interface_->getJointNames(); // getJointNmaes() 함수는 시뮬레이션 환경에서도 동작 한다.

    RCLCPP_INFO(manipulation_node_->get_logger(), "Planning frame %s", move_group_interface_->getPlanningFrame().c_str());
    RCLCPP_INFO(manipulation_node_->get_logger(), "End effrctor link: %s", move_group_interface_->getEndEffectorLink().c_str());

    RCLCPP_INFO(manipulation_node_->get_logger(), "Current joint values :");
    for(size_t i = 0; i < joint_group_name.size(); ++i)
    {
      RCLCPP_INFO(manipulation_node_->get_logger(), "joint_%s :: %f", joint_group_name[i].c_str(), cure_joint_values_[i]);
    }
    saveJointToFile(cure_joint_values_, filename);
  }

  void Manipulation::saveCureWapointToJob(const std::string& filename)
  {
    RCLCPP_INFO(manipulation_node_->get_logger(), "current pose values save as Waypoint");
    RCLCPP_INFO(manipulation_node_->get_logger(), "current pose values are =============================================");
    RCLCPP_INFO(manipulation_node_->get_logger(), "Current pose: position (x: %f, y: %f, z: %f), orientation (x: %f, y: %f, z: %f, w: %f)",
                  cure_pose_values_.pose.position.x,
                  cure_pose_values_.pose.position.y,
                  cure_pose_values_.pose.position.z,
                  cure_pose_values_.pose.orientation.x,
                  cure_pose_values_.pose.orientation.y,
                  cure_pose_values_.pose.orientation.z,
                  cure_pose_values_.pose.orientation.w);
    saveWaypointToFile(cure_pose_values_, filename);
  }

  void Manipulation::moveEEposeCartesian(const double x, const double y, const double z)
  {
    geometry_msgs::msg::PoseStamped startPose = cure_pose_values_;
    geometry_msgs::msg::PoseStamped targetPose = startPose;

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(x, y, z));
    transform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    tf2::Transform currentTransform;
    tf2::fromMsg(cure_pose_values_.pose, currentTransform);

    tf2::Transform targetTransform;
    targetTransform = currentTransform * transform;
    tf2::toMsg(targetTransform, targetPose.pose);

    moveToPoseCartesian(targetPose);
  }

  void Manipulation::saveArcWaypoits(const std::string& filename)
  {
    for(size_t i = 0; i<arc_tmp_wapoints_.size(); ++i)
    {
      saveWaypointToFile(arc_tmp_wapoints_[i], filename);
    }
  }

  void Manipulation::setFrameidforTaskMsg(const std::string &frame_name)
  {
      if(frame_name == BASE_FRAME_ID)
      {
          task_frame_id_ = BASE_FRAME_ID;
      }else {
          task_frame_id_ = TOOL_FRAME_ID;
      }
  }

  int Manipulation::getCurrentJobIndex()
  {
      return current_job_index_;
  }

  void Manipulation::setCurrentJobIndex(int index)
  {
      current_job_index_ = index;
  }

  void Manipulation::moveArcWaypoints(const double radius, const int point_num, const int angle_value,
                                      const int x_direction, const int y_direction, const int z_direction,
                                      const std::string &frame_name)
  {
      std::vector<geometry_msgs::msg::PoseStamped> tmpWaypoints;

      if(!thread_running_)
      {
          arc_tmp_wapoints_.clear();
          if(frame_name == BASE_FRAME_ID)
          {
              arc_tmp_wapoints_ = computeArcWaypointAtBaseCoord(radius, point_num, angle_value,x_direction, y_direction, z_direction);
          } else if(frame_name == TOOL_FRAME_ID)
          {
              arc_tmp_wapoints_ = computeArcWaypointAtToolCoord(radius, point_num, angle_value,x_direction, y_direction, z_direction);
          } else
          {
              RCLCPP_WARN(manipulation_node_->get_logger(), "Check FRAME ID NAME!!");
          }

          std::thread moveArcTargetThread([=](){
              moveTargetArcStartThread();
          });
          moveArcTargetThread.detach();
      }
  }

  void Manipulation::moveArcWaypointStop()
  {
      if(thread_arc_)
      {
          move_group_interface_->stop();
          thread_running_ = false;
          thread_arc_ = false;
      }
  }

}
