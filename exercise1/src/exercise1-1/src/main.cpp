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
  const auto pick_pose = hot_dog_scenario.getPickPose();
  const auto place_pose = hot_dog_scenario.getPlacePose();

  // Exercise 1-1: Move to the pick pose
  // Use the MoveGroupInterface to move to the pick pose
  move_group_interface.setPoseTarget(pick_pose);

  // Create a plan to that target pose
  MoveGroupInterface::Plan pick_plan;
  uint16_t attempts = 0;
  bool pick_success = false;
  while (!pick_success && attempts < 5)
  {
    pick_success = static_cast<bool>(move_group_interface.plan(pick_plan));
    ++attempts;
  }
  if (pick_success)
  {
    // Execute the plan
    move_group_interface.execute(pick_plan);

    const auto& sausage = hot_dog_scenario.getSausage();

    // Exercise 1-1: Attach the hot dog to the gripper and plan to the place pose
    move_group_interface.attachObject(sausage.id);
    move_group_interface.setPoseTarget(place_pose);

    MoveGroupInterface::Plan place_plan;
    attempts = 0;
    bool place_success = false;
    while (!place_success && attempts < 5)
    {
      place_success = static_cast<bool>(move_group_interface.plan(place_plan));
      ++attempts;
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

    // Exercise 1-1: Detach the hot dog
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
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
