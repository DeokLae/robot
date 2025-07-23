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
 #include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MecanumIMUOdometryNode : public rclcpp::Node {
public:
    MecanumIMUOdometryNode() : Node("mecanum_imu_odometry") {
        rclcpp::QoS qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
        qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        rclcpp::QoS qos_imu = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        // Odometry publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);

        // IMU subscriber
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", qos,
            std::bind(&MecanumIMUOdometryNode::imuCallback, this, std::placeholders::_1));

        // Velocity subscriber
        velocity_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&MecanumIMUOdometryNode::velocityCallback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize variables
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        last_time_ = this->get_clock()->now();
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Update yaw angle from IMU
        //RCLCPP_WARN(this->get_logger(), "imu callback function call!");
        tf2::Quaternion quat_tf;
        tf2::fromMsg(msg->orientation, quat_tf);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
        imu_yaw_ = yaw;
        //RCLCPP_WARN(this->get_logger(), "imu yaw balue is :: %f", imu_yaw_);
    }

    void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        auto current_time = this->get_clock()->now();
        double dt = (current_time - last_time_).seconds();
        last_time_ = current_time;

        // Extract velocities
        double vx = (msg->linear.x);
        double vy = (msg->linear.y);
        double omega = (msg->angular.z);

        // Update position
        //x_ += (vx * cos(theta_) - vy * sin(theta_)) * dt;
        //y_ += (vx * sin(theta_) + vy * cos(theta_)) * dt;

        x_ += (vx * cos(theta_)) * dt;
        y_ += (vx * sin(theta_)) * dt;

        // Combine IMU and odometry yaw
        //theta_ = (0.5 * omega) + (0.5 * imu_yaw_);
        //theta_ += omega*dt;
        //theta_ = (0.5*theta_) + (0.5*imu_yaw_);
        //theta_ = atan2(sin(theta_), cos(theta_));

        RCLCPP_WARN(this->get_logger(), "imu yaw value is %f", imu_yaw_);
        RCLCPP_WARN(this->get_logger(), "omega is %f", omega);
        RCLCPP_WARN(this->get_logger(), "theta_ is %f", theta_);
        theta_ = imu_yaw_;

        // Publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.orientation.z = sin(theta_ / 2.0);
        odom_msg.pose.pose.orientation.w = cos(theta_ / 2.0);

        odom_msg.twist.twist.linear.x = vx;
        //odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.angular.z = omega;

        odom_pub_->publish(odom_msg);

        // Publish transform
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;

         // theta_를 이용하여 회전 쿼터니언 생성 (2D 회전)
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);  // (roll, pitch, yaw) 순서로 회전 설정

        // 회전 쿼터니언을 메시지로 변환하여 할당
        odom_trans.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_trans);

        //odom_trans.transform.rotation = odom_msg.pose.pose.orientation;

        //tf_broadcaster_->sendTransform(odom_trans);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x_, y_, theta_, imu_yaw_;
    rclcpp::Time last_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MecanumIMUOdometryNode>());
    rclcpp::shutdown();
    return 0;
}
