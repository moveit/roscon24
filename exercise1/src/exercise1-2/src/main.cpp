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

  // Create the MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  const std::string planning_group{ "ur_manipulator" };
  MoveGroupInterface move_group_interface(node, planning_group);

  // A pose just above the bun
  const auto start_pose = hot_dog_scenario.getStartPose();
  move_group_interface.setPoseTarget(start_pose);

  MoveGroupInterface::Plan start_plan;
  const bool success = static_cast<bool>(move_group_interface.plan(start_plan));
  if (success)
  {
    move_group_interface.execute(start_plan);

    const std::vector<geometry_msgs::msg::Pose> waypoints = hot_dog_scenario.getMustardWaypoints(start_pose);

    // Exercise 1-2 Plan a Caresian path
    const double jump_threshold = 0.0;
    const double eef_step = 0.002;

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    if (fraction > 0)
    {
      move_group_interface.execute(trajectory);
      hot_dog_scenario.applyMustard();
    }
  }

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
