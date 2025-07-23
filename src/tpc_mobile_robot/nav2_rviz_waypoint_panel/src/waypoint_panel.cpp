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

class WaypointPanel : public rviz_common::Panel
{
public:
  WaypointPanel(QWidget *parent = nullptr)
  : rviz_common::Panel(parent),
    node_(rclcpp::Node::make_shared("waypoint_rviz_panel"))//,
    //action_client_(rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    //  node_, "/follow_waypoints"))
  {
    auto layout = new QVBoxLayout;

    QPushButton *save_1__button = new QPushButton("적재위치 이동경로 저장");
    QPushButton *save_2__button = new QPushButton("작업위치_1 이동경로 저장");
    QPushButton *save_3__button = new QPushButton("작업위치_2 이동경로 저장");
    save_1__button->setMinimumHeight(60);
    save_2__button->setMinimumHeight(60);
    save_3__button->setMinimumHeight(60);

    layout->addWidget(save_1__button);
    layout->addWidget(save_2__button);
    layout->addWidget(save_3__button);
    setLayout(layout);

    connect(save_1__button, &QPushButton::clicked, this, &WaypointPanel::on_save_1_clicked);
    connect(save_2__button, &QPushButton::clicked, this, &WaypointPanel::on_save_2_clicked);
    connect(save_3__button, &QPushButton::clicked, this, &WaypointPanel::on_save_3_clicked);

    acml_pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10,
            std::bind(&WaypointPanel::amclPoseCallback, this, std::placeholders::_1));

    initial_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initialpose", 10);

    rclcpp::sleep_for(5s);
    loadLastPoseAndPublish();

    // Spin node in a background thread
    thread_ = std::thread([this]() {
      rclcpp::spin(node_);
    });
  }

  ~WaypointPanel() override {
    rclcpp::shutdown();
    if (thread_.joinable()) thread_.join();
  }

  void on_save_1_clicked()
  {
    RCLCPP_INFO(node_->get_logger(), "Current Position Saving...");

    if (!last_amcl_pose_) {
    RCLCPP_ERROR(node_->get_logger(), "현재 위치를 받지 못했습니다.");
    return;
  }

    double x = last_amcl_pose_->pose.pose.position.x;
    double y = last_amcl_pose_->pose.pose.position.y;
    double yaw = tf2::getYaw(last_amcl_pose_->pose.pose.orientation);

    const char* home = std::getenv("HOME");
    std::string dir_path = std::string(home) + "/movepoints";
    std::string file_path = dir_path + "/saved_waypoints.yaml";

    // 디렉토리 생성 (없으면)
    std::filesystem::create_directories(dir_path);
    YAML::Node root;

    // 기존 파일 로딩 (있으면 append)
    if (std::ifstream fin{file_path}; fin.good()) {
      root = YAML::LoadFile(file_path);
    }

    YAML::Node wp;
    wp["x"] = x;
    wp["y"] = y;
    wp["yaw"] = yaw;
    root["waypoints_1"].push_back(wp);

    std::ofstream fout(file_path);
    fout << root;
    fout.close();

    RCLCPP_INFO(node_->get_logger(), "위치 저장됨 (%.2f, %.2f, %.2f)", x, y, yaw);
  }

  void on_save_2_clicked()
  {
    RCLCPP_INFO(node_->get_logger(), "Current Position Saving...");

    if (!last_amcl_pose_) {
    RCLCPP_ERROR(node_->get_logger(), "현재 위치를 받지 못했습니다.");
    return;
    }

    double x = last_amcl_pose_->pose.pose.position.x;
    double y = last_amcl_pose_->pose.pose.position.y;
    double yaw = tf2::getYaw(last_amcl_pose_->pose.pose.orientation);

    const char* home = std::getenv("HOME");
    std::string dir_path = std::string(home) + "/movepoints";
    std::string file_path = dir_path + "/saved_waypoints.yaml";

    // 디렉토리 생성 (없으면)
    std::filesystem::create_directories(dir_path);
    YAML::Node root;

    // 기존 파일 로딩 (있으면 append)
    if (std::ifstream fin{file_path}; fin.good()) {
      root = YAML::LoadFile(file_path);
    }

    YAML::Node wp;
    wp["x"] = x;
    wp["y"] = y;
    wp["yaw"] = yaw;
    root["waypoints_2"].push_back(wp);

    std::ofstream fout(file_path);
    fout << root;
    fout.close();

    RCLCPP_INFO(node_->get_logger(), "위치 저장됨 (%.2f, %.2f, %.2f)", x, y, yaw);
  }

  void on_save_3_clicked()
  {
    RCLCPP_INFO(node_->get_logger(), "Current Position Saving...");

    if (!last_amcl_pose_) {
    RCLCPP_ERROR(node_->get_logger(), "현재 위치를 받지 못했습니다.");
    return;
    }

    double x = last_amcl_pose_->pose.pose.position.x;
    double y = last_amcl_pose_->pose.pose.position.y;
    double yaw = tf2::getYaw(last_amcl_pose_->pose.pose.orientation);

    const char* home = std::getenv("HOME");
    std::string dir_path = std::string(home) + "/movepoints";
    std::string file_path = dir_path + "/saved_waypoints.yaml";

    // 디렉토리 생성 (없으면)
    std::filesystem::create_directories(dir_path);
    YAML::Node root;

    // 기존 파일 로딩 (있으면 append)
    if (std::ifstream fin{file_path}; fin.good()) {
      root = YAML::LoadFile(file_path);
    }

    YAML::Node wp;
    wp["x"] = x;
    wp["y"] = y;
    wp["yaw"] = yaw;
    root["waypoints_3"].push_back(wp);

    std::ofstream fout(file_path);
    fout << root;
    fout.close();

    RCLCPP_INFO(node_->get_logger(), "위치 저장됨 (%.2f, %.2f, %.2f)", x, y, yaw);
  }

  void amclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    last_amcl_pose_ = msg;
    saveLastPose(last_amcl_pose_);
    RCLCPP_DEBUG(node_->get_logger(), "Received AMCL pose: (%.2f, %.2f)",
                msg->pose.pose.position.x, msg->pose.pose.position.y);
  }

  void saveLastPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double yaw = tf2::getYaw(msg->pose.pose.orientation);

    const char* home = std::getenv("HOME");
    std::string dir_path = std::string(home) + "/movepoints";
    std::string file_path = dir_path + "/saved_waypoints.yaml";

    // 디렉토리 생성 (없으면)
    std::filesystem::create_directories(dir_path);
    YAML::Node root;

    // 기존 파일 로딩 (있으면 append)
    if (std::ifstream fin{file_path}; fin.good()) {
      root = YAML::LoadFile(file_path);
    }

    YAML::Node wp;
    wp["x"] = x;
    wp["y"] = y;
    wp["yaw"] = yaw;
    root["lastpose"]=wp;

    std::ofstream fout(file_path);
    fout << root;
    fout.close();
  }

  void loadLastPoseAndPublish()
  {
    const char* home = std::getenv("HOME");
    std::string file_path = std::string(home) + "/movepoints/saved_waypoints.yaml";

    if (!std::filesystem::exists(file_path)) {
      RCLCPP_WARN(node_->get_logger(), "No saved waypoint file found at: %s", file_path.c_str());
      return;
    }

    YAML::Node root = YAML::LoadFile(file_path);
    if (!root["lastpose"]) {
      RCLCPP_WARN(node_->get_logger(), "No 'lastpose' key found in YAML");
      return;
    }

    auto lastpose = root["lastpose"];
    double x = lastpose["x"].as<double>();
    double y = lastpose["y"].as<double>();
    double yaw = lastpose["yaw"].as<double>();

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = "map";

    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    pose_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    pose_msg.pose.covariance[0] = 0.25;
    pose_msg.pose.covariance[7] = 0.25;
    pose_msg.pose.covariance[35] = 0.0685;

    RCLCPP_INFO(node_->get_logger(),
      "Publishing initial pose: x=%.2f, y=%.2f, yaw=%.2f", x, y, yaw);

    initial_pose_pub_->publish(pose_msg);
  }



private:
  rclcpp::Node::SharedPtr node_;
  std::thread thread_;
  //rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr action_client_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr acml_pose_sub_;
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_amcl_pose_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
};

PLUGINLIB_EXPORT_CLASS(WaypointPanel, rviz_common::Panel)
