#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <hot_dog_scenario/hot_dog_scenario.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the node
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "exercise1_1", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Spin the node
  auto spin_thread = std::thread([&node]() { rclcpp::spin(node); });

  HotDogScenario hot_dog_scenario(node);
  hot_dog_scenario.placeHotDog();

  // Create a ROS logger
  const auto logger = rclcpp::get_logger("exercise1_1");

  // Create the MoveGroupInterface
  using moveit::planning_interface::MoveGroupInterface;
  const std::string planning_group{ "ur_manipulator" };
  MoveGroupInterface move_group_interface(node, planning_group);

  // Get predefined poses for pick and place above the sausage and bun respectively
  const geometry_msgs::msg::Pose& pick_pose = hot_dog_scenario.getPickPose();
  const geometry_msgs::msg::Pose& place_pose = hot_dog_scenario.getPlacePose();

  // Exercise 1-1: Set the goal pose to the pick pose using MoveGroupInterface
  move_group_interface.setPoseTarget(pick_pose);

  // Create a plan to that target pose
  MoveGroupInterface::Plan pick_plan;
  uint16_t pick_planning_attempts{ 0 };
  bool pick_success{ false };
  while (!pick_success && pick_planning_attempts < 5)
  {
    // Exercise 1-1: Use MoveGroupInterface to plan to the pick pose and determine planning success
    pick_success = static_cast<bool>(move_group_interface.plan(pick_plan));
    ++pick_planning_attempts;
  }
  if (pick_success)
  {
    // Execute the plan
    move_group_interface.execute(pick_plan);

    const moveit_msgs::msg::CollisionObject& sausage = hot_dog_scenario.getSausage();

    // Exercise 1-1: Attach the hot dog to the gripper and plan to the place pose
    move_group_interface.attachObject(sausage.id);
    move_group_interface.setPoseTarget(place_pose);

    MoveGroupInterface::Plan place_plan;
    uint16_t place_planning_attempts{ 0 };
    bool place_success{ false };
    while (!place_success && place_planning_attempts < 5)
    {
      // Exercise 1-1: Use MoveGroupInterface to plan to the place pose and determine planning success
      place_success = static_cast<bool>(move_group_interface.plan(place_plan));
      ++place_planning_attempts;
    }

    // Execute the plan
    if (place_success)
    {
      move_group_interface.execute(place_plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Place planning failed!");
    }

    // Detach the hot dog
    move_group_interface.detachObject(sausage.id);
    if (place_success)
    {
      // Drop the hot dog down into the bun
      hot_dog_scenario.placeSausage();
    }
  }
  else
  {
    RCLCPP_ERROR(logger, "Pick planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
