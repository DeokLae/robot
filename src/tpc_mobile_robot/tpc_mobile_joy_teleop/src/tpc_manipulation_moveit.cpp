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

#include "tpc_mobile_joy_teleop/tpc_manipulation_moveit.hpp"
namespace TpcManipulationMoveit{

  Manipulation::Manipulation(){
    manipulation_node_ = std::make_shared<rclcpp::Node>("manipulation_node");
    RCLCPP_WARN(manipulation_node_->get_logger(), "111111111111111111111111111111111111");
    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(manipulation_node_, "robot_description");
    RCLCPP_WARN(manipulation_node_->get_logger(), "2222222222222222222222222222222222222");
    robot_model_ = robot_model_loader_->getModel();
    RCLCPP_WARN(manipulation_node_->get_logger(), "333333333333333333333333333333333333333");
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    RCLCPP_WARN(manipulation_node_->get_logger(), "44444444444444444444444444444444444444444");

    move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(manipulation_node_, GROUP_NAME);
    cartesian_velocity_scale_ = CARTE_VELOCITY_SCALE;
    cartesian_acceleration_scale_ = CARTE_ACCELERATION_SCALE;
    joint_velocity_scale_ = JOINT_VELOCITY_SCALE;
    joint_acceleration_scale_ = jOINT_ACCELERATION_SCALE;
    std::thread(std::bind(&Manipulation::getCurePose, this)).detach();
    std::thread(std::bind(&Manipulation::getCureJoint, this)).detach();

    robot_state_sub_ = manipulation_node_->create_subscription<sensor_msgs::msg::JointState>(JOINT_STATE_TOPIC,10, std::bind(&Manipulation::subCureRobotState, this, std::placeholders::_1));
    std::thread(std::bind(&Manipulation::spin, this)).detach();

    current_job_index_ = 0;
    thread_running_ = false;
    stop_thread_ = true;
    pause_thread_ = true;
  }

  Manipulation::~Manipulation()
  {

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

  void Manipulation::readJobValuesFromFile(const std::string& filename)
  {
    initJobInfo();
    all_joints_values_.clear();
    all_poses_values_.clear();
    all_job_info_.clear();
    all_wayPoints_.clear();
    std::ifstream infile(filename);
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
        if(!positions.empty()){
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
    for (const auto& jobs : all_job_info_)
    {
      if(!std::get<0>(jobs).empty())
      {
        std::cout << "readJobValuesFromFiel function  [INFO]============joint vlaue is ============" << std::endl;
        std::cout << "readJobValuesFromFiel function  [INFO] pose value :: " << std::get<1>(jobs).pose.position.x << std::endl;
      } else {
        std::cout << "readJobValuesFromFiel function  [INFO]============joint vlaue is not ============" << std::endl;
        std::cout << "readJobValuesFromFiel function  [INFO] pose value :: " << std::get<1>(jobs).pose.position.x << std::endl;
      }
    }

    for(const auto& joints : all_joints_values_){
      for(size_t i = 0; i < joints.size(); ++i){
        std::cout << "joint " << i << "==" << joints[i] << std::endl;
      }
    }
    for (const auto& tmppose : all_poses_values_)
    {
      std::cout << "readJobValuesFromFiel function  [INFO] position_x :: " << tmppose.pose.position.x << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] position_y :: " << tmppose.pose.position.y << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] position_z :: " << tmppose.pose.position.z << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] orientation.x :: " << tmppose.pose.orientation.x << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] orientation.y :: " << tmppose.pose.orientation.y << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] orientation.z :: " << tmppose.pose.orientation.z << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] orientation.w :: " << tmppose.pose.orientation.w << std::endl;
    }
    for (const auto& tmppose : all_wayPoints_)
    {
      std::cout << "readJobValuesFromFiel function  [INFO] waypoint_position_x :: " << tmppose.position.x << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] waypoint_position_y :: " << tmppose.position.y << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] waypoint_position_z :: " << tmppose.position.z << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] waypoint_orientation.x :: " << tmppose.orientation.x << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] waypoint_orientation.y :: " << tmppose.orientation.y << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] waypoint_orientation.z :: " << tmppose.orientation.z << std::endl;
      std::cout << "readJobValuesFromFiel function  [INFO] waypoint_orientation.w :: " << tmppose.orientation.w << std::endl;
    }
  }

  void Manipulation::savePoseToFile(const geometry_msgs::msg::PoseStamped& tmpPose, const std::string& filename){
    std::ofstream outfile(filename, std::ios::app);
    if(!outfile.is_open()){
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

  void Manipulation::saveJointToFile(const std::vector<double>& tmpJoint, const std::string& filename){
    std::ofstream outfile(filename, std::ios::app);
    if(!outfile.is_open()){
      std::cerr << "\033[31m" << "saveJointToFile function in manipulation CLASS ERROR :: (Read Job File) Can not open file!!" << "\033[0m" << std::endl;
      return;
    }
    outfile << "joint ";
    for(size_t i = 0; i < tmpJoint.size(); ++i){
      outfile << tmpJoint[i];
      if(i != tmpJoint.size() -1){
        outfile << " ";
      }
    }
    outfile << std::endl;
    outfile.close();
  }

  void Manipulation::saveWaypointToFile(const geometry_msgs::msg::PoseStamped& tmpPose, const std::string& filename){
    std::ofstream outfile(filename, std::ios::app);
    if(!outfile.is_open()){
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

  std::vector<geometry_msgs::msg::PoseStamped> Manipulation::getAllPose(){
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

  void Manipulation::subCureRobotState(sensor_msgs::msg::JointState::SharedPtr msg)
  {
    cure_robot_state_ = msg;
    last_robot_state_update_time_ = manipulation_node_->now();
    robot_state_->setVariablePositions(cure_robot_state_->name, cure_robot_state_->position);
    move_group_interface_->setStartState(*robot_state_);
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

  void Manipulation::moveTargetPoseCartesian(const geometry_msgs::msg::PoseStamped pose)
  {
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(pose.pose);

    computeCartesianPath(waypoints);
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
      // joint 값으로 이동 할 때도 trajectory 를 생성해서 움직 이는거 가능 한거 같다.
      // 근데 wapoint 를 쓴는건 아니다.
      move_group_interface_->execute(plan);
      //move_group_interface_->asyncExecute(my_plan);
    } else
    {
      RCLCPP_ERROR(manipulation_node_->get_logger(), "Planning failed!");
    }
  }

  void Manipulation::moveTargetPose(const geometry_msgs::msg::PoseStamped pose)
  {
    move_group_interface_->setPoseTarget(pose);
    move_group_interface_->setPlanningTime(10.0);
    move_group_interface_->setNumPlanningAttempts(10);
    move_group_interface_->setMaxVelocityScalingFactor(joint_velocity_scale_);
    move_group_interface_->setMaxAccelerationScalingFactor(joint_acceleration_scale_);

    auto const [success, plan] = [&]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group_interface_->plan(msg));
      return std::make_pair(ok, msg);
    }();

    if(success)
    {
      move_group_interface_->execute(plan);
      RCLCPP_INFO(manipulation_node_->get_logger(), "Move to Target Pose");
    } else{
      RCLCPP_ERROR(manipulation_node_->get_logger(), "Planning failed!!");
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

  void Manipulation::getCurePose()
  {
    while (rclcpp::ok())
    {
      //auto joint_model_group = robot_state_->getJointModelGroup(GROUP_NAME);
      const Eigen::Isometry3d& end_effector_state = robot_state_->getGlobalLinkTransform(EE_NAME);
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

  void Manipulation::spin()
  {
    while (rclcpp::ok())
    {
      /* code */
      rclcpp::spin_some(manipulation_node_);
    }
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

    moveTargetPoseCartesian(targetPose);
  }

  void Manipulation::drawArcAtToolCoordinate(const double radius, const int point_num, const int angle_value,
                                                const int x_direction, const int y_direction, const int z_direction)
  {
    geometry_msgs::msg::PoseStamped startPose;
    startPose.header.frame_id = EE_NAME;
    startPose.pose = cure_pose_values_.pose;

    geometry_msgs::msg::PoseStamped targetPose;
    targetPose.header.frame_id = EE_NAME;
    targetPose.pose = startPose.pose;
    std::vector<geometry_msgs::msg::Pose> waypoints;

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
      waypoints.push_back(targetPose.pose);
      saveWaypointToFile(targetPose, "jointvalue.tpc");
    }
    computeCartesianPath(waypoints);
  }

  void Manipulation::drawArcAtBaseCoordinate(const double radius, const int point_num, const double angle_value,
                                             const int x_direction, const int y_direction, const int z_direction)
  {
    geometry_msgs::msg::PoseStamped startPose;
    startPose.header.frame_id = BASE_FRAME_ID;
    startPose.pose = cure_pose_values_.pose;
    geometry_msgs::msg::PoseStamped targetPose;
    targetPose.header.frame_id = BASE_FRAME_ID;
    targetPose.pose = startPose.pose;

    std::vector<geometry_msgs::msg::Pose> tmpWaypoints;

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
      tmpWaypoints.push_back(targetPose.pose);
      saveWaypointToFile(targetPose, "jointvalue.tpc");
    }
    computeCartesianPath(tmpWaypoints);
  }

  void Manipulation::makeWaypoints(geometry_msgs::msg::PoseStamped point)
  {
    RCLCPP_INFO(manipulation_node_->get_logger(), "add waypoint");
    tmp_waypoints_.push_back(point.pose);
  }

  void Manipulation::runWapoints()
  {
    std::vector<geometry_msgs::msg::Pose> tmpWayPoints;
    tmpWayPoints = tmp_waypoints_;
    RCLCPP_INFO(manipulation_node_->get_logger(), "run waypoint");
    computeCartesianPath(tmpWayPoints);
    tmp_waypoints_.clear();
  }

  void Manipulation::move_job_trajectory()
  {
    thread_running_ = true;
    readJobValuesFromFile("jointvalue.tpc");
    std::vector<double> tmpJointValue;
    geometry_msgs::msg::PoseStamped tmpPoseValue;

    size_t i = current_job_index_;
    for(; i < all_job_info_.size(); ++i)
    {
      if(pause_thread_){
        //manipulator.moveTargetPoseCartesian(manipulator.cure_pose_values_);
        //manipulator.move_group_interface_->stop();
        current_job_index_ = i;
        thread_running_ = false;
        return;
      }
      if(stop_thread_){
        current_job_index_ = 0;
        thread_running_ = false;
        return;
      }
      RCLCPP_INFO(manipulation_node_->get_logger(), "MOVE  TO %ld_ST JOB MOTION", i+1);

      if(!std::get<0>(all_job_info_[i]).empty())
      {
        tmpJointValue = std::get<0>(all_job_info_[i]);
        moveTargetJoint(tmpJointValue);
        for (size_t j = 0; j < tmpJointValue.size(); j++)
        {
          RCLCPP_INFO(manipulation_node_->get_logger(), "joint_%ld :: %f", j, tmpJointValue[j]);
        }
      } else if (std::get<1>(all_job_info_[i]).pose.position.x)
      {
        tmpPoseValue = std::get<1>(all_job_info_[i]);
        moveTargetPoseCartesian(tmpPoseValue);
      } else if (std::get<2>(all_job_info_[i]).position.x)
      {
        if (std::get<2>(all_job_info_[i+1]).position.x)
        {
          tmpPoseValue.pose = std::get<2>(all_job_info_[i]);
          makeWaypoints(tmpPoseValue);
          RCLCPP_INFO(manipulation_node_->get_logger(), "waypoints just saved!! next value x is %f", std::get<2>(all_job_info_[i+1]).position.x);
        }else
        {
          tmpPoseValue.pose = std::get<2>(all_job_info_[i]);
          makeWaypoints(tmpPoseValue);
          RCLCPP_INFO(manipulation_node_->get_logger(), "waypoints move start!!");
          runWapoints();
        }
      }
      if(pause_thread_){
        //manipulator.moveTargetPoseCartesian(manipulator.cure_pose_values_);
        //manipulator.move_group_interface_->stop();
        current_job_index_ = i;
        thread_running_ = false;
        return;
      }
      if(stop_thread_){
        current_job_index_ = 0;
        thread_running_ = false;
        return;
      }
    }
    current_job_index_ = 0;
    thread_running_ = false;
    RCLCPP_WARN(manipulation_node_->get_logger(), "Motion Cycle Complete!!");
  }

  void Manipulation::moveJobStart()
  {
    RCLCPP_INFO(manipulation_node_->get_logger(),"call the thread");
    if(!thread_running_)
    {
      RCLCPP_INFO(manipulation_node_->get_logger(),"thread start!!");
      pause_thread_ = false;
      stop_thread_ = false;
      std::thread(&Manipulation::move_job_trajectory, this).detach();
    }
  }

  void Manipulation::moveJobPause()
  {
    pause_thread_ = true;
    move_group_interface_->stop();
  }

  void Manipulation::moveJobStop()
  {
      stop_thread_ = true;
      move_group_interface_->stop();
  }
}
