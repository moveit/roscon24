#pragma once
#include <memory>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

class HotDogScenario
{
public:
  HotDogScenario(rclcpp::Node::SharedPtr node);

  // Place the bun and sausage in the scene
  void placeHotDog();

  // Place the bun and sausage in the scene
  void placeSausage();

  static geometry_msgs::msg::Pose getPickPose();
  static geometry_msgs::msg::Pose getPlacePose();

  moveit_msgs::msg::CollisionObject getSausage() const;

private:
  rclcpp::Node::SharedPtr node_;

  // Planning scene interface used to apply collision objects
  PlanningSceneInterface planning_scene_interface_;

  // Move Group interface used to get the current pose of the robot
  std::unique_ptr<MoveGroupInterface> move_group_interface_;

  // Collision objects and colors
  moveit_msgs::msg::CollisionObject bun_;
  moveit_msgs::msg::CollisionObject sausage_;
  moveit_msgs::msg::CollisionObject mustard_;
  moveit_msgs::msg::CollisionObject mustard_bottle_;
  std::vector<moveit_msgs::msg::ObjectColor> hot_dog_colors_;

  // Helper method to create a general collision object
  static moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& name, const std::string& mesh_path,
                                                                 const geometry_msgs::msg::Pose& pose);

  // Helper methods to create hot dog collision objects
  static moveit_msgs::msg::CollisionObject createBun();
  static moveit_msgs::msg::CollisionObject createSausage();
  static moveit_msgs::msg::CollisionObject createMustard();
  static moveit_msgs::msg::CollisionObject createMustardBottle();

  // Helper method to create colors for hot dog collision objects
  static std::vector<moveit_msgs::msg::ObjectColor> createHotDogColors();
};
