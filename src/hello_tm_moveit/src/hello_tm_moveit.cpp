#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <thread>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_tm_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_tm_moveit");

  // We spin up a SingleThreadExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;   //  싱글 스레드를 사용하기 위한 클래스 executor 라는 이름으로 정의
  executor.add_node(node);   // 위에서 정의한 싱글 스레드를 사용하기 위한 클래스에 node 를 연결
  //  spinner 라는 이름으로 스레드가 스핀하는 객체를 생성 ??
  auto spinner = std::thread([&executor](){
    executor.spin();
  });

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "tm_arm");

  // Construct and initialize MoveItVisualTools
  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
    node, "link_0", rviz_visual_tools::RVIZ_MARKER_TOPIC, move_group_interface.getRobotModel()
  };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();

  // Create a closures for visualization
  auto const draw_title = [&moveit_visual_tools](auto text){
    auto const test_pose = []{
      auto msg = Eigen::Isometry3d::Identity();
      msg.translation().z() = 1.0;
      return msg;
    }();
    moveit_visual_tools.publishText(test_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
  };
  auto const prompt = [&moveit_visual_tools](auto text){
    moveit_visual_tools.prompt(text);
  };
  auto const draw_trajectory_tool_path = [&moveit_visual_tools, jmg = move_group_interface.getRobotModel()->getJointModelGroup("tm_arm")](auto const trajectory){
    moveit_visual_tools.publishTrajectoryLine(trajectory,jmg);
  };

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = 0.4;
    msg.position.z = 0.5;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  //Create collision object for the robot to avoid
  auto const collision_object = [frame_id = move_group_interface.getPlanningFrame()]{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = frame_id;
    collision_object.id = "box1";
    shape_msgs::msg::SolidPrimitive primitive;

    //Define the size of the box in merters
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.4;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.3;

    //Define the pose of the box (relative to the frame_id)
    geometry_msgs::msg::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.2;
    box_pose.position.y = 0.2;
    box_pose.position.z = 0.15;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    return collision_object;
  }();

  //Add the collision object to the scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(collision_object);


  // Create a plan to that target pose
  prompt("Press 'NEXT' in the RvizVisualToolsGui window to execute");
  draw_title("Executing");
  moveit_visual_tools.trigger();


  auto const [success, plan] = [&move_group_interface] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if (success)
  {
    draw_trajectory_tool_path(plan.trajectory_);
    moveit_visual_tools.trigger();
    prompt("Press 'NEXT' in the RvizVisualToolsGui window to execute");
    draw_title("Executing");
    moveit_visual_tools.trigger();


    move_group_interface.execute(plan);
  }
  else
  {
    draw_title("Planning Failed");
    moveit_visual_tools.trigger();


    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spinner.join();  // <--프로그램이 종료되기 전 모든 스레드가 완료되기를 기다린다.
  return 0;
}
