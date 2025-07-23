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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/duration.hpp"
#include "tpc_mobile_robot_hardware/tpc_mobile_robot_hardware.hpp"

#include "pluginlib/class_list_macros.hpp"


PLUGINLIB_EXPORT_CLASS(
  tpc::tpc_mobile_robot_hardware::TpcMobileRobotSystemHardware,
  hardware_interface::SystemInterface
)

namespace tpc{
namespace tpc_mobile_robot_hardware{
  auto logger = rclcpp::get_logger("TpcMobileRobotSystemHardware");

  hardware_interface::CallbackReturn TpcMobileRobotSystemHardware::on_init(
    const hardware_interface::HardwareInfo & info)
  {
    try
    {
      hardware_interface::CallbackReturn result = hardware_interface::SystemInterface::on_init(info);
      if(result != hardware_interface::CallbackReturn::SUCCESS)
      {
        RCLCPP_ERROR(logger, "fail to start interface for mobile robot connection!!!");
        return result;
      }
      RCLCPP_INFO(logger, "start hardware interface for mobile robot connection!!!");
      rclcpp::QoS qos_odom = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
      rclcpp::QoS qos_nn = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
      odom_pub_node_ = std::make_shared<rclcpp::Node>("tpc_mobile_odom_publisher_node");
      odom_publisher_ = odom_pub_node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

      rclcpp::QoS qos_imu = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
      qos_imu.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
      imu_sub_ = odom_pub_node_->create_subscription<sensor_msgs::msg::Imu>("imu/data", qos_imu,
            std::bind(&TpcMobileRobotSystemHardware::imu_callback, this, std::placeholders::_1));

      velocity_sub_ = odom_pub_node_->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&TpcMobileRobotSystemHardware::velocityCallback, this, std::placeholders::_1));

      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(odom_pub_node_);

      executor.add_node(odom_pub_node_);
      std::thread([&](){executor.spin();}).detach();

      x_pos_ = 0.0;
      y_pos_ = 0.0;
      theta_ = 0.0;
      roll_ = 0.0;
      pich_=0.0;
      vx_ = 0.0;
      vy_ = 0.0;
      omega_z_ = 0.0;

      delta_x = 0.0;
      delta_y = 0.0;
      delta_theta = 0.0;

      pre_x = 0.0;
      pre_y = 0.0;
      pre_theta = 0.0;

      current_time_ = odom_pub_node_->get_clock()->now();
      last_time_ = odom_pub_node_->get_clock()->now();
      dt = 0.0;
      motorFront_ = std::make_unique<ZLAC>();
      //motorRear_ = std::make_unique<ZLAC>();
      motorFront_->init(mecanum_wheel::MOTOR_COMPORT, mecanum_wheel::BOUDRATE, mecanum_wheel::MOTOR_FRONT_ID, false);
      //motorRear_->init(mecanum_wheel::MOTOR_COMPORT, mecanum_wheel::BOUDRATE, mecanum_wheel::MOTOR_REAR_ID, false);

      motorFront_->set_vel_mode();
      //motorRear_->set_vel_mode();
      motorFront_->set_acc_time(800, "LEFT");
      motorFront_->set_acc_time(800, "RIGHT");
      motorFront_->set_decc_time(500, "LEFT");
      motorFront_->set_decc_time(500, "RIGHT");
      //motorRear_->set_acc_time(500, "LEFT");
      //motorRear_->set_acc_time(500, "RIGHT");
      //motorRear_->set_decc_time(800, "LEFT");
      //motorRear_->set_decc_time(800, "RIGHT");

      wheel_positions_.resize(4, 0.0);
      wheel_command_velocities_.resize(4,0.0);
      wheel_velocities_.resize(4, 0.0);

      motorData_Front_ = motorFront_->get_position();
      motorData_Front_ = motorFront_->get_rpm();
      init_motor_encoder_L = motorData_Front_.encoder_L;
      init_motor_encoder_R = motorData_Front_.encoder_R;

      mean_rot_dist_old = 0.0;

      RCLCPP_INFO(logger, "Initializing TpcMobileRobotSystemHardware hardware interface...");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(logger, "failed initialize TpcMobileRobotSystemHardware interface :: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  std::vector<hardware_interface::StateInterface> TpcMobileRobotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(
      hardware_interface::StateInterface{"left_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_positions_[0]}
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface{"right_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_positions_[1]}
    );
    //state_interfaces.emplace_back(
    //  hardware_interface::StateInterface{"rr_left_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_positions_[2]}
    //);
    //state_interfaces.emplace_back(
    //  hardware_interface::StateInterface{"rr_right_wheel_joint", hardware_interface::HW_IF_POSITION, &wheel_positions_[3]}
    //);

    state_interfaces.emplace_back(
      hardware_interface::StateInterface{"left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[0]}
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface{"right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[1]}
    );
    //state_interfaces.emplace_back(
    //  hardware_interface::StateInterface{"rr_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[2]}
    //);
    //state_interfaces.emplace_back(
    //  hardware_interface::StateInterface{"rr_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_velocities_[3]}
    //);

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> TpcMobileRobotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    command_interfaces.emplace_back(
      hardware_interface::CommandInterface{"left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_velocities_[0]}
    );
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface{"right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_velocities_[1]}
    );
    //command_interfaces.emplace_back(
    //  hardware_interface::CommandInterface{"rr_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_velocities_[2]}
    //);
    //command_interfaces.emplace_back(
    //  hardware_interface::CommandInterface{"rr_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &wheel_command_velocities_[3]}
    //);

    return command_interfaces;
  }

  hardware_interface::return_type TpcMobileRobotSystemHardware::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    try
    {
      motorData_Front_ = motorFront_->get_position();
      //motorData_Rear_ = motorRear_->get_position();
      //motorData_Rear_ = motorRear_->get_rpm();

      constexpr double PULSES_PER_REV = 16384.0;   // 제조사값
      constexpr double GEAR_RATIO = 1.0;            // 기어비 있으면 수정
      constexpr double TWO_PI = 2.0 * M_PI;

      double revs_L = motorData_Front_.encoder_L / PULSES_PER_REV;
      double wheel_revs_L = revs_L / GEAR_RATIO;

      wheel_positions_[0] = wheel_revs_L * TWO_PI;
      //wheel_positions_[0] = motorData_Front_.encoder_L;


      double revs_R = motorData_Front_.encoder_R / PULSES_PER_REV;
      double wheel_revs_R = revs_R / GEAR_RATIO;

      wheel_positions_[1] = wheel_revs_R * TWO_PI;

      motorData_Front_ = motorFront_->get_rpm();
      wheel_velocities_[0] = motorData_Front_.rpm_L * ((2 * M_PI)/60.0);
      wheel_velocities_[1] = motorData_Front_.rpm_R* ((2 * M_PI)/60.0);

      //RCLCPP_WARN(logger, "Get rpm in hardware interface is %f", wheel_velocities_[1]);

      //wheel_positions_[2] = motorData_Rear_.encoder_L;
      //wheel_velocities_[2] = motorData_Rear_.rpm_L;

      //wheel_positions_[3] = motorData_Rear_.encoder_R;
      //wheel_velocities_[3] = motorData_Rear_.rpm_R;

      calculate_odometry();
      //publish_odometry();
      //publish_tf(time);

      return hardware_interface::return_type::OK;
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(logger, "read error in TpcMobileRobotSystemHardware :: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  hardware_interface::return_type TpcMobileRobotSystemHardware::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    try
    {
      motorFront_->set_double_rpm(wheel_command_velocities_[0] * (60.0 / (2 * M_PI)), wheel_command_velocities_[1] * (60.0 / (2 * M_PI)));
      //motorRear_->set_double_rpm(wheel_command_velocities_[2], wheel_command_velocities_[3]);
      //RCLCPP_WARN(logger, "=============set rpm in hardware interface===================");
      //RCLCPP_WARN(logger, "set rpm in hardware interface is %f", wheel_command_velocities_[0] * 60.0 / (2 * M_PI));
      //RCLCPP_WARN(logger, "=============set rpm in hardware interface===================");
      return hardware_interface::return_type::OK;
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      RCLCPP_ERROR(logger, "wite error in TpcMobileRobotSystemHardware :: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
  }

  hardware_interface::CallbackReturn TpcMobileRobotSystemHardware::on_activate(
    const rclcpp_lifecycle::State & previous_state)
  {
    try
    {
      motorFront_->enable();
      //motorRear_->enable();

      RCLCPP_INFO(logger, "on activate hardware !!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(logger, "failed on_activate TpcMobileRobotSystemHardware interface :: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hardware_interface::CallbackReturn TpcMobileRobotSystemHardware::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
  {
    try
    {
      motorFront_->disable();
      //motorRear_->disable();

      RCLCPP_INFO(logger, "on deactivaate hardware !!");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(logger, "failed deactivaate TpcMobileRobotSystemHardware interface :: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  void TpcMobileRobotSystemHardware::calculate_odometry(){

    current_time_ = odom_pub_node_->get_clock()->now();
    dt = (current_time_ - last_time_).seconds();
    last_time_ = current_time_;
    //RCLCPP_WARN(logger, "dt is :::::::: %f", dt);

    double wheel_radius = 0.10; // 휠 반지름 (m)
    double base_width = 0.7092;  // 로봇 폭 (m)
    double base_length = 0.575; // 로봇 길이 (m)

    //double w = wheel_radius * ((2*M_1_PI) / 60);
    double w =  (2 * M_PI) / 60.0;

    // 엔코더 데이터를 m/s로 변환
    double v_fl = (wheel_velocities_[0]) * wheel_radius;
    double v_fr = (wheel_velocities_[1]) * wheel_radius;
    double v_rl = wheel_velocities_[2] * w ;
    double v_rr = wheel_velocities_[3] * w ;

    double left_endoder = wheel_positions_[0] - init_motor_encoder_L;
    double right_edcoder = wheel_positions_[1] - init_motor_encoder_R;

    double rot_L_dst = (left_endoder/16385)*2*M_PI*wheel_radius;
    double rot_R_dst = (right_edcoder/16385)*2*M_PI*wheel_radius;

    double mean_rot_dist = (rot_L_dst + rot_R_dst) / 2.0;
    double mean_rot_dist_diff = mean_rot_dist - mean_rot_dist_old;

    // 메카넘휠 kinematics 역변환
    //vx_ = (v_fl + v_fr + v_rl + v_rr) / 4.0;
    //vy_ = (-v_fl + v_fr + v_rl - v_rr) / 4.0;
    //omega_z_ = (-v_fl + v_fr - v_rl + v_rr) / (4.0 * (base_width/2 + base_length/2));

    //2 wheel
    vx_ = (v_fl + v_fr) / 2.0;
    //omega_z_ = (v_fr - v_fl) / base_width;
    omega_z_ = (v_fr - v_fl) / base_width;
    //theta_ += omega_z_ * dt;
    //theta_ = (rot_R_dst - rot_L_dst) / base_width;

    //RCLCPP_WARN(logger, "X v_fl is ::: %f", v_fl);
    //RCLCPP_WARN(logger, "X v_fr is ::: %f", v_fr);
    //RCLCPP_WARN(logger, "X omega_z_ is ::: %f", omega_z_);
    //RCLCPP_WARN(logger, "X imu_angular_velocity_ is ::: %f", imu_angular_velocity_.z);
    //RCLCPP_WARN(logger, "X wheel_velocities_ is ::: %f", wheel_velocities_[0]);
    //RCLCPP_WARN(logger, "Y vellocity is ::: %f", vy_);
    //RCLCPP_WARN(logger, "theta_ is ::: %f", theta_);

    // odometry 위치 업데이트
    //double dt = (period.nanoseconds() * 1e-9); // 시간 간격
    //x_pos_ += (vx_ * cos(theta_) - vy_ * sin(theta_)) * dt;
    //y_pos_ += (vx_ * sin(theta_) + vy_ * cos(theta_)) * dt;
    //RCLCPP_WARN(logger, "theta is ::: %f", theta_);

    x_pos_ += (vx_ * cos(theta_) ) * dt;
    y_pos_ += (vx_ * sin(theta_) ) * dt;

    //x_pos_ += mean_rot_dist_diff * cos(theta_);
    //y_pos_ += mean_rot_dist_diff * sin(theta_);

    //RCLCPP_WARN(logger, "x_pos_ is ::: %f", x_pos_);
    //RCLCPP_WARN(logger, "y_pos_ is ::: %f", y_pos_);

    nav_msgs::msg::Odometry odom_msg_;
    odom_msg_.header.stamp = current_time_;
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";

    // 위치 및 속도
    odom_msg_.pose.pose.position.x = x_pos_;
    odom_msg_.pose.pose.position.y = y_pos_;

    //odom_msg_.pose.pose.orientation.z = sin(theta_ / 2.0);
    //odom_msg_.pose.pose.orientation.w = cos(theta_ / 2.0);

    odom_msg_.twist.twist.linear.x = vx_;
    odom_msg_.twist.twist.linear.y = 0.0;
    odom_msg_.twist.twist.angular.z = omega_z_;

    tf2::Quaternion quat;
    quat.setRPY(0,0,theta_);
    odom_msg_.pose.pose.orientation = tf2::toMsg(quat);;
    odom_msg_.twist.covariance = { 0.1, 0, 0, 0, 0, 0,
                                    0, 0.1, 0, 0, 0, 0,
                                    0, 0, 0.1, 0, 0, 0,
                                    0, 0, 0, 0.1, 0, 0,
                                    0, 0, 0, 0, 0.1, 0,
                                    0, 0, 0, 0, 0, 0.1};
    odom_msg_.pose.covariance = { 0.1, 0, 0, 0, 0, 0,
                                  0, 0.1, 0, 0, 0, 0,
                                  0, 0, 0.1, 0, 0, 0,
                                  0, 0, 0, 0.1, 0, 0,
                                  0, 0, 0, 0, 0.1, 0,
                                  0, 0, 0, 0, 0, 0.1};
    // 메시지 발행
    odom_publisher_->publish(odom_msg_);

    geometry_msgs::msg::TransformStamped odom_tf_;

    odom_tf_.header.stamp = current_time_;
    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_footprint";

    odom_tf_.transform.translation.x = x_pos_;
    odom_tf_.transform.translation.y = y_pos_;
    odom_tf_.transform.translation.z = 0.0;

    //RCLCPP_WARN(logger, "theta_ :::: %f", imu_angular_velocity_.z);
    //RCLCPP_WARN(logger, "y move :::: %f", odom_tf_.transform.translation.y);

    // theta_를 이용하여 회전 쿼터니언 생성 (2D 회전)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);  // (roll, pitch, yaw) 순서로 회전 설정

    // 회전 쿼터니언을 메시지로 변환하여 할당
    odom_tf_.transform.rotation = tf2::toMsg(quat);
    //odom_tf_.transform.rotation = odom_msg_.pose.pose.orientation;

    //odom_tf_.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, theta_));

    tf_broadcaster_->sendTransform(odom_tf_);
    mean_rot_dist_old = mean_rot_dist;
    //RCLCPP_WARN(logger, "x move :::: %f", x_pos_);
    //RCLCPP_WARN(logger, "y move :::: %f", y_pos_);
    //RCLCPP_WARN(logger, "z move :::: %f", theta_);
  }

  void TpcMobileRobotSystemHardware::publish_odometry(){
    // Odometry 메시지 구성
    nav_msgs::msg::Odometry odom_msg_;
    odom_msg_.header.stamp = current_time_;
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";

    // 위치 및 속도
    odom_msg_.pose.pose.position.x = x_pos_;
    odom_msg_.pose.pose.position.y = y_pos_;

    //odom_msg_.pose.pose.orientation.z = sin(theta_ / 2.0);
    //odom_msg_.pose.pose.orientation.w = cos(theta_ / 2.0);

    odom_msg_.twist.twist.linear.x = vx_;
    odom_msg_.twist.twist.linear.y = 0.0;
    odom_msg_.twist.twist.angular.z = omega_z_;

    tf2::Quaternion quat;
    quat.setRPY(0,0,theta_);
    odom_msg_.pose.pose.orientation = tf2::toMsg(quat);;
    odom_msg_.twist.covariance = { 0.1, 0, 0, 0, 0, 0,
                                    0, 0.1, 0, 0, 0, 0,
                                    0, 0, 0.1, 0, 0, 0,
                                    0, 0, 0, 0.1, 0, 0,
                                    0, 0, 0, 0, 0.1, 0,
                                    0, 0, 0, 0, 0, 0.1};
    odom_msg_.pose.covariance = { 0.1, 0, 0, 0, 0, 0,
                                  0, 0.1, 0, 0, 0, 0,
                                  0, 0, 0.1, 0, 0, 0,
                                  0, 0, 0, 0.1, 0, 0,
                                  0, 0, 0, 0, 0.1, 0,
                                  0, 0, 0, 0, 0, 0.1};
    // 메시지 발행
    odom_publisher_->publish(odom_msg_);

    geometry_msgs::msg::TransformStamped odom_tf_;

    odom_tf_.header.stamp = current_time_;
    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_footprint";

    odom_tf_.transform.translation.x = x_pos_;
    odom_tf_.transform.translation.y = y_pos_;
    odom_tf_.transform.translation.z = 0.0;

    //RCLCPP_WARN(logger, "theta_ :::: %f", imu_angular_velocity_.z);
    //RCLCPP_WARN(logger, "y move :::: %f", odom_tf_.transform.translation.y);

    // theta_를 이용하여 회전 쿼터니언 생성 (2D 회전)
    //tf2::Quaternion q;
    //q.setRPY(0, 0, theta_);  // (roll, pitch, yaw) 순서로 회전 설정

    // 회전 쿼터니언을 메시지로 변환하여 할당
    odom_tf_.transform.rotation = tf2::toMsg(quat);
    //odom_tf_.transform.rotation = odom_msg_.pose.pose.orientation;

    //odom_tf_.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, theta_));

    tf_broadcaster_->sendTransform(odom_tf_);
  }

  void TpcMobileRobotSystemHardware::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg){
    //RCLCPP_WARN(odom_pub_node_->get_logger(), "imu callback function call!!");
    imu_orientation_ = msg->orientation;
    imu_angular_velocity_ = msg->angular_velocity;
    imu_linear_acceleration_ = msg->linear_acceleration;

    tf2::Quaternion quaternion;
    quaternion.setX(msg->orientation.x);
    quaternion.setY(msg->orientation.y);
    quaternion.setZ(msg->orientation.z);
    quaternion.setW(msg->orientation.w);

    tf2::Matrix3x3 matrix(quaternion);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    theta_ = yaw;
    //roll_ = roll;
    //pich_ = pitch;
    //rclcpp::sleep_for(std::chrono::milliseconds(10));
    //RCLCPP_WARN(logger, "roll :::: %f", roll);
    //RCLCPP_WARN(logger, "pitch :::: %f", pitch);
    //RCLCPP_WARN(logger, "yaw :::: %f", yaw);
  }

  void TpcMobileRobotSystemHardware::velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {

        // Extract velocities
        double vx = (msg->linear.x);
        double vy = (msg->linear.y);
        double omega = (msg->angular.z);
        //omega_z_ = omega;
        //RCLCPP_INFO(logger, "omega_z_ is :::: %f", omega_z_);
  }


  /*void TpcMobileRobotSystemHardware::publish_tf(const rclcpp::Time &current_time){
    odom_tf_.header.stamp = current_time;
    odom_tf_.header.frame_id = "odom";
    odom_tf_.child_frame_id = "base_footprint";

    odom_tf_.transform.translation.x = x_pos_;
    odom_tf_.transform.translation.y = y_pos_;
    odom_tf_.transform.translation.z = 0.0;

    // theta_를 이용하여 회전 쿼터니언 생성 (2D 회전)
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);  // (roll, pitch, yaw) 순서로 회전 설정

    // 회전 쿼터니언을 메시지로 변환하여 할당
    odom_tf_.transform.rotation = tf2::toMsg(q);

    //odom_tf_.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, theta_));

    tf_broadcaster_->sendTransform(odom_tf_);
  }*/

} // namespace tpc_mobile_robot_hardware end
} // namespace tpc end
