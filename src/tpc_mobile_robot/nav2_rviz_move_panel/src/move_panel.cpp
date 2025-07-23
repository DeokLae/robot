#include <QPushButton>
#include <QVBoxLayout>
#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>

#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include "rclcpp/wait_for_message.hpp"
#include <filesystem>
using namespace std::chrono_literals;

class MovePanel : public rviz_common::Panel
{
public:
  MovePanel(QWidget *parent = nullptr)
  : rviz_common::Panel(parent),
    node_(rclcpp::Node::make_shared("waypoint_rviz_panel")),
    action_client_(rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
      node_, "/follow_waypoints"))
  {
    auto layout = new QVBoxLayout;

    QPushButton *move_1_button = new QPushButton("적재위치로 이동");
    QPushButton *move_2_button = new QPushButton("작업위치_1 이동");
    QPushButton *move_3_button = new QPushButton("작업위치_2 이동");
    move_1_button->setMinimumHeight(60);
    move_2_button->setMinimumHeight(60);
    move_3_button->setMinimumHeight(60);

    layout->addWidget(move_1_button);
    layout->addWidget(move_2_button);
    layout->addWidget(move_3_button);
    setLayout(layout);

    connect(move_1_button, &QPushButton::clicked, this, &MovePanel::on_move_1_button_clicked);
    connect(move_2_button, &QPushButton::clicked, this, &MovePanel::on_move_2_button_clicked);
    connect(move_3_button, &QPushButton::clicked, this, &MovePanel::on_move_3_button_clicked);

    // Spin node in a background thread
    thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });
  }

  ~MovePanel() override {
    rclcpp::shutdown();
    if (thread_.joinable()) thread_.join();
  }

  void on_move_1_button_clicked()
  {
    if (!action_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(node_->get_logger(), "Waypoint action server not available.");
      return;
    }

    const char* home = std::getenv("HOME");
    std::string dir_path = std::string(home) + "/movepoints";
    std::string file_path = dir_path + "/saved_waypoints.yaml";

    YAML::Node root = YAML::LoadFile(file_path);
    if (!root["waypoints_1"]) return;

    auto goal = nav2_msgs::action::FollowWaypoints::Goal();

    for (const auto & wp : root["waypoints_1"]) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = node_->now();

      pose.pose.position.x = wp["x"].as<double>();
      pose.pose.position.y = wp["y"].as<double>();

      double yaw = wp["yaw"].as<double>();
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);

      goal.poses.push_back(pose);
    }

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto result) {
      RCLCPP_INFO(node_->get_logger(), "웨이포인트 순찰 완료!");
    };
    action_client_->async_send_goal(goal, send_goal_options);
    RCLCPP_INFO(node_->get_logger(), "웨이포인트 %ld개 전송됨", goal.poses.size());
    RCLCPP_INFO(node_->get_logger(), "Sent waypoints.");
  }

  void on_move_2_button_clicked()
  {
    if (!action_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(node_->get_logger(), "Waypoint action server not available.");
      return;
    }

    const char* home = std::getenv("HOME");
    std::string dir_path = std::string(home) + "/movepoints";
    std::string file_path = dir_path + "/saved_waypoints.yaml";

    YAML::Node root = YAML::LoadFile(file_path);
    if (!root["waypoints_2"]) return;

    auto goal = nav2_msgs::action::FollowWaypoints::Goal();

    for (const auto & wp : root["waypoints_2"]) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = node_->now();

      pose.pose.position.x = wp["x"].as<double>();
      pose.pose.position.y = wp["y"].as<double>();

      double yaw = wp["yaw"].as<double>();
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);

      goal.poses.push_back(pose);
    }
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto result) {
      RCLCPP_INFO(node_->get_logger(), "웨이포인트 순찰 완료!");
    };
    action_client_->async_send_goal(goal, send_goal_options);
    RCLCPP_INFO(node_->get_logger(), "웨이포인트 %ld개 전송됨", goal.poses.size());
    RCLCPP_INFO(node_->get_logger(), "Sent waypoints.");
  }

  void on_move_3_button_clicked()
  {
    if (!action_client_->wait_for_action_server(2s)) {
      RCLCPP_ERROR(node_->get_logger(), "Waypoint action server not available.");
      return;
    }

    const char* home = std::getenv("HOME");
    std::string dir_path = std::string(home) + "/movepoints";
    std::string file_path = dir_path + "/saved_waypoints.yaml";

    YAML::Node root = YAML::LoadFile(file_path);
    if (!root["waypoints_3"]) return;

    auto goal = nav2_msgs::action::FollowWaypoints::Goal();

    for (const auto & wp : root["waypoints_3"]) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.header.stamp = node_->now();

      pose.pose.position.x = wp["x"].as<double>();
      pose.pose.position.y = wp["y"].as<double>();

      double yaw = wp["yaw"].as<double>();
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);

      goal.poses.push_back(pose);
    }
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto result) {
      RCLCPP_INFO(node_->get_logger(), "웨이포인트 순찰 완료!");
    };
    action_client_->async_send_goal(goal, send_goal_options);
    RCLCPP_INFO(node_->get_logger(), "웨이포인트 %ld개 전송됨", goal.poses.size());
    RCLCPP_INFO(node_->get_logger(), "Sent waypoints.");
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::thread thread_;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr action_client_;
};

PLUGINLIB_EXPORT_CLASS(MovePanel, rviz_common::Panel)
