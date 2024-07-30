#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <hot_dog/hot_dog_collision_object.hpp>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "hello_roscon", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  const auto logger = rclcpp::get_logger("hello_roscon");

  using moveit::planning_interface::MoveGroupInterface;
  using moveit::planning_interface::PlanningSceneInterface;

  PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject bun = hot_dog::HotDogFactory::createBun();
  moveit_msgs::msg::CollisionObject sausage = hot_dog::HotDogFactory::createOffsetSausage();

  hot_dog::HotDogFactory::addCollisionObjectsToScene(planning_scene_interface, { bun, sausage });

  // Exercise 1-1: Create the MoveIt MoveGroup Interface and generate a motion plan
  // Create the MoveIt MoveGroup Interface and set target pose
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Set a target pose (TODO make this more fun)
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;

  move_group_interface.setPoseTarget(target_pose);
  // move_group_interface.attachObject()

  // Create a plan to that target pose
  MoveGroupInterface::Plan plan;
  auto success = static_cast<bool>(move_group_interface.plan(plan));

  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  // Exercise 1-2: Adding collision objects to the scene?
  // Exercise 1-3: Attaching / detaching objects?

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
