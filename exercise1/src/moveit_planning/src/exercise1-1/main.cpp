#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& name, const std::string& mesh_path, const geometry_msgs::msg::Pose& pose){
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = name;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
  m->scale(0.05);
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  const auto mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision_object.meshes.push_back(mesh);
  collision_object.operation = collision_object.ADD;
  collision_object.pose = pose;

  return collision_object;
}

void addCollisionObjectsToScene(moveit::planning_interface::PlanningSceneInterface& psi){

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::vector<moveit_msgs::msg::ObjectColor> colors;

  geometry_msgs::msg::Pose bun_pose;
  bun_pose.orientation.w = 1.0;
  bun_pose.position.x = 0.3;
  bun_pose.position.y = 0.0;
  bun_pose.position.z = 0.2;
  collision_objects.push_back(createCollisionObject("bun", "package://moveit_planning/hotdog/bun.dae", bun_pose));

  std_msgs::msg::ColorRGBA bun_color;
  bun_color.r = 0.907f;
  bun_color.g = 0.608f;
  bun_color.b = 0.244f;
  bun_color.a = 1.0f;
  moveit_msgs::msg::ObjectColor bun_color_moveit;
  bun_color_moveit.color = bun_color;
  bun_color_moveit.id = "bun";
  colors.push_back(bun_color_moveit);

  geometry_msgs::msg::Pose sausage_pose;
  sausage_pose.orientation.w = 1.0;
  sausage_pose.position.x = 0.25;
  sausage_pose.position.y = 0.045;
  sausage_pose.position.z = 0.04;
  collision_objects.push_back(createCollisionObject("sausage", "package://moveit_planning/hotdog/sausage.dae", sausage_pose));

  std_msgs::msg::ColorRGBA sausage_color;
  sausage_color.r = 0.906f;
  sausage_color.g = 0.302f;
  sausage_color.b = 0.233f;
  sausage_color.a = 1.0f;
  moveit_msgs::msg::ObjectColor sausage_color_moveit;
  sausage_color_moveit.color = sausage_color;
  sausage_color_moveit.id = "sausage";
  colors.push_back(sausage_color_moveit);

  geometry_msgs::msg::Pose mustard_pose;
  mustard_pose.orientation.w = 1.0;
  mustard_pose.position.x = 0.3;
  mustard_pose.position.y = 0.035;
  mustard_pose.position.z = -0.305;

 collision_objects.push_back(createCollisionObject("mustard", "package://moveit_planning/hotdog/mustard.dae", mustard_pose));

  std_msgs::msg::ColorRGBA mustard_color;
  mustard_color.r = 0.906f;
  mustard_color.g = 0.833f;
  mustard_color.b = 0.109f;
  mustard_color.a = 1.0f;
  moveit_msgs::msg::ObjectColor mustard_color_moveit;
  mustard_color_moveit.color = mustard_color;
  mustard_color_moveit.id = "mustard";
  colors.push_back(mustard_color_moveit);

  psi.applyCollisionObjects(collision_objects, colors);
}

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
      "hello_roscon", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  const auto logger = rclcpp::get_logger("hello_roscon");

  using moveit::planning_interface::MoveGroupInterface;

  // Exercise 1-1: Create the MoveIt MoveGroup Interface and generate a motion plan
  // Create the MoveIt MoveGroup Interface and set target pose
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  addCollisionObjectsToScene(planning_scene_interface);

  // Set a target pose (TODO make this more fun)
  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = 0.28;
  target_pose.position.y = -0.2;
  target_pose.position.z = 0.5;

  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  moveit::planning_interface::MoveGroupInterface::Plan plan;
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
