#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <hot_dog_scenario/hot_dog_scenario.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
    "exercise1-2",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  HotDogScenario hot_dog_scenario(node);

  auto spin_thread = std::thread([&node]() {
    rclcpp::spin(node);
  });

  const std::string planning_group {"ur_manipulator"};

  hot_dog_scenario.placeMustardinEE();

  MoveGroupInterface move_group_interface(node, planning_group);

  // A pose just above the bun
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.707;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.707;
  start_pose.position.x = 0.38;
  start_pose.position.y = 0.0;  // TODO this will look off until we have a proper gripper
  start_pose.position.z = 0.15;

  move_group_interface.setPoseTarget(start_pose);

  MoveGroupInterface::Plan start_plan;
  const bool success = static_cast<bool> (move_group_interface.plan(start_plan));
  if (success)
  {
    move_group_interface.execute(start_plan);
    hot_dog_scenario.applyMustard();
  }

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
