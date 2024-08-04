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

  // TODO fix
  // hot_dog::HotDogFactory::addCollisionObjectsToScene(planning_scene_interface, { bun, sausage });

  // TODO update with new sausage positions
  // A pose just above the sausage
  geometry_msgs::msg::Pose pick_pose;
  pick_pose.orientation.x = 1.0;
  pick_pose.orientation.y = 0.0;
  pick_pose.orientation.z = 0.0;
  pick_pose.orientation.w = 0.0;
  pick_pose.position.x = 0.3;
  pick_pose.position.y = 0.4;
  pick_pose.position.z = 0.1;

  // A pose just above the bun
  geometry_msgs::msg::Pose place_pose;
  place_pose.orientation.x = 1.0;
  place_pose.orientation.y = 0.0;
  place_pose.orientation.z = 0.0;
  place_pose.orientation.w = 0.0;
  place_pose.position.x = 0.3;
  place_pose.position.y = 0.0;
  place_pose.position.z = 0.15;

  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");
  rclcpp_action::Client<moveit_msgs::action::MoveGroup>& move_group_client = move_group_interface.getMoveGroupClient();

  // Exercise 1-1: Move to the hot dog
  // Use the MoveGroupInterface to move to the pick pose
  move_group_interface.setPoseTarget(pick_pose);

  // Create a plan to that target pose
  MoveGroupInterface::Plan pick_plan;
  const auto pick_success = static_cast<bool>(move_group_interface.plan(pick_plan));

  if (pick_success)
  {
    // Execute the plan
    move_group_interface.execute(pick_plan);

    // Wait for the execution to finish before attaching objects
    while (!move_group_client.action_server_is_ready())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // Exercise 1-1: Attach the hot dog to the gripper and plan to the place pose
    // TODO we need a gripper
    move_group_interface.attachObject(sausage.id);
    move_group_interface.setPoseTarget(place_pose);

    MoveGroupInterface::Plan place_plan;
    const auto place_success = static_cast<bool>(move_group_interface.plan(place_plan));
    // Execute the plan
    if (place_success)
    {
      move_group_interface.execute(place_plan);
    }
    else
    {
      RCLCPP_ERROR(logger, "Place planning failed!");
    }

    while (!move_group_client.action_server_is_ready())
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    // Exercise 1-1: Detach the hot dog
    move_group_interface.detachObject(sausage.id);

    // TODO: consider using ACM to allow robot and sausage bun collisions
    // Teleport the hot dog
    // hot_dog::HotDogFactory::addCollisionObjectsToScene(planning_scene_interface,
    //                                                    { hot_dog::HotDogFactory::createSausage() });
  }
  else
  {
    RCLCPP_ERROR(logger, "Pick planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
