#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <hot_dog_scenario/hot_dog_scenario.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the node
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "exercise1_2", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Spin the node
  auto spin_thread = std::thread([&node]() { rclcpp::spin(node); });

  // Start this scenario by placing the mustard bottle in the end effector
  HotDogScenario hot_dog_scenario(node);
  hot_dog_scenario.placeHotDog();
  hot_dog_scenario.placeMustardinEE();

  // Create a ROS logger
  const auto logger = rclcpp::get_logger("exercise1_2");

  // Create the MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  const std::string planning_group{ "ur_manipulator" };
  MoveGroupInterface move_group_interface(node, planning_group);

  // A pose just above the bun
  const auto start_pose = hot_dog_scenario.getStartPose();
  move_group_interface.setPoseTarget(start_pose);

  MoveGroupInterface::Plan start_plan;
  bool success{ false };
  uint16_t attempts{ 0 };
  while (!success && attempts < 5)
  {
    success = static_cast<bool>(move_group_interface.plan(start_plan));
    ++attempts;
  }

  if (success)
  {
    move_group_interface.execute(start_plan);

    const std::vector<geometry_msgs::msg::Pose> waypoints = hot_dog_scenario.getMustardWaypoints(start_pose);

    moveit_msgs::msg::RobotTrajectory trajectory;

    // TODO(Exercise 1-2): Plan a Cartesian path
    double fraction = 0;

    if (fraction > 0)
    {
      move_group_interface.execute(trajectory);
      hot_dog_scenario.applyMustard();
    }
    else
    {
      RCLCPP_ERROR(logger, "Mustard path planning failed!");
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Move to hot dog planning failed!");
  }

  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
