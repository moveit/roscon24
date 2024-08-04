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
  moveit_msgs::msg::CollisionObject sausage = hot_dog::HotDogFactory::createSausage();
  moveit_msgs::msg::CollisionObject mustard = hot_dog::HotDogFactory::createMustard();
  moveit_msgs::msg::CollisionObject mustard_bottle = hot_dog::HotDogFactory::createMustardBottle();

  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // Spawn the mustard bottle attached to the end effector
  // mustard_bottle.pose = move_group_interface.getCurrentPose().pose;
  hot_dog::HotDogFactory::addCollisionObjectsToScene(planning_scene_interface, { bun, sausage, mustard_bottle });

  // moveit_msgs::msg::AttachedCollisionObject attached_mustard_bottle;
  // attached_mustard_bottle.object = mustard_bottle;
  // attached_mustard_bottle.link_name = move_group_interface.getEndEffectorLink();

  // planning_scene_interface.applyAttachedCollisionObject(attached_mustard_bottle);

  // A pose just above the bun
  geometry_msgs::msg::Pose start_pose;
  start_pose.orientation.x = 1.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;
  start_pose.position.x = 0.25;
  start_pose.position.y = 0.0;
  start_pose.position.z = 0.3;

  // Exercise 1-2: Move to the hot dog
  // Use the MoveGroupInterface to move to the pick pose
  move_group_interface.setPoseTarget(start_pose);

  // Create a plan to that target pose
  MoveGroupInterface::Plan start_plan;
  const auto start_move_success = static_cast<bool>(move_group_interface.plan(start_plan));

  if (start_move_success)
  {
    // Execute the plan
    move_group_interface.execute(start_plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Pick planning failed!");
  }

  // TODO: consider using ACM to allow robot and sausage bun collisions
  // Teleport the hot dog
  hot_dog::HotDogFactory::addCollisionObjectsToScene(planning_scene_interface, { mustard });

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
