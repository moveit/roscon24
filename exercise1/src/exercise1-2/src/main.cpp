#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <motion_planning/motion_planning.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  MotionPlanning motion_planning("hot_dog_with_mustard", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto spin_thread = std::thread([&executor, &motion_planning]() {
    executor.add_node(motion_planning.getNodeBaseInterface());
    executor.spin();
    executor.remove_node(motion_planning.getNodeBaseInterface());
  });

  motion_planning.setUpScene();

  // A pose just above the bun
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.707;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.707;
  start_pose.position.x = 0.38;
  start_pose.position.y = 0.0;  // TODO this will look off until we have a proper gripper
  start_pose.position.z = 0.15;

  MoveGroupInterface::Plan start_plan;
  const bool success = motion_planning.plan(start_pose, start_plan);
  if (success)
  {
    motion_planning.execute(start_plan);
    motion_planning.applyMustard();
  }

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
