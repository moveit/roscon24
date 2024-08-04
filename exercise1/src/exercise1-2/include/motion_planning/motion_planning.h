#pragma once
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <hot_dog/hot_dog_collision_object.hpp>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

class MotionPlanning
{
public:
  MotionPlanning(const std::string& name, const rclcpp::NodeOptions& options);

  void setUpScene();

  bool plan(const geometry_msgs::msg::Pose& pose, MoveGroupInterface::Plan& plan);

  void execute(const MoveGroupInterface::Plan& plan);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void applyMustard();

private:
  rclcpp::Node::SharedPtr node_;
  PlanningSceneInterface planning_scene_interface_;
  std::unique_ptr<MoveGroupInterface> move_group_interface_;
  moveit_msgs::msg::CollisionObject bun_;
  moveit_msgs::msg::CollisionObject sausage_;
  moveit_msgs::msg::CollisionObject mustard_;
  moveit_msgs::msg::CollisionObject mustard_bottle_;
  std::vector<moveit_msgs::msg::ObjectColor> hot_dog_colors_;
};
