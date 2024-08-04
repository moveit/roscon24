
#include <hot_dog_scenario/hot_dog_scenario.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

HotDogScenario::HotDogScenario(rclcpp::Node::SharedPtr node) : node_{ node }
{
  bun_ = createBun();
  sausage_ = createSausage();
  mustard_ = createMustard();
  mustard_bottle_ = createMustardBottle();
  move_group_interface_ = std::make_unique<MoveGroupInterface>(MoveGroupInterface(node_, "ur_manipulator"));
  hot_dog_colors_ = createHotDogColors();
};

void HotDogScenario::placeHotDog()
{
  planning_scene_interface_.applyCollisionObjects({ bun_, sausage_ }, hot_dog_colors_);
}

void HotDogScenario::placeMustardinEE()
{
  // Spawn the mustard bottle attached to the end effector
  mustard_bottle_.pose = move_group_interface_->getCurrentPose().pose;
  mustard_bottle_.pose.position.y += 0.1;

  // Rotate the mustard bottle to fit into the gripper
  tf2::Quaternion relative_rotation;
  relative_rotation.setRPY(0, M_PI / 2, 0);
  tf2::Quaternion current_orientation;
  tf2::fromMsg(mustard_bottle_.pose.orientation, current_orientation);
  tf2::Quaternion new_orientation = current_orientation * relative_rotation;
  mustard_bottle_.pose.orientation = tf2::toMsg(new_orientation);

  planning_scene_interface_.applyCollisionObjects({ mustard_bottle_ }, hot_dog_colors_);
  move_group_interface_->attachObject(mustard_bottle_.id);
}

void HotDogScenario::applyMustard()
{
  planning_scene_interface_.applyCollisionObjects({ mustard_ }, hot_dog_colors_);
}

moveit_msgs::msg::CollisionObject HotDogScenario::createCollisionObject(const std::string& name,
                                                                        const std::string& mesh_path,
                                                                        const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = name;

  // Full size hot dog is much too large and must be scaled
  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path, { 0.05, 0.05, 0.05 });
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  const auto mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision_object.meshes.push_back(mesh);
  collision_object.operation = collision_object.ADD;
  collision_object.pose = pose;

  return collision_object;
}

moveit_msgs::msg::CollisionObject HotDogScenario::createBun()
{
  geometry_msgs::msg::Pose bun_pose;
  bun_pose.orientation.w = 1.0;
  bun_pose.position.x = 0.5;
  bun_pose.position.y = 0.0;
  bun_pose.position.z = 0.0;
  return createCollisionObject("bun", "package://hot_dog/hotdog/bun.dae", bun_pose);
}

moveit_msgs::msg::CollisionObject HotDogScenario::createSausage()
{
  geometry_msgs::msg::Pose sausage_pose;
  sausage_pose.orientation.w = 1.0;
  sausage_pose.position.x = 0.5;
  sausage_pose.position.y = 0.0;
  sausage_pose.position.z = 0.0;
  return createCollisionObject("sausage", "package://hot_dog/hotdog/sausage.dae", sausage_pose);
}

moveit_msgs::msg::CollisionObject HotDogScenario::createMustard()
{
  geometry_msgs::msg::Pose mustard_pose;
  mustard_pose.orientation.w = 1.0;
  mustard_pose.position.x = 0.5;
  mustard_pose.position.y = 0.0;
  mustard_pose.position.z = 0.0;
  return createCollisionObject("mustard", "package://hot_dog/hotdog/mustard.dae", mustard_pose);
}

moveit_msgs::msg::CollisionObject HotDogScenario::createMustardBottle()
{
  // Spawn at origin since the mustard bottle is really only meant to be an attached collision object
  geometry_msgs::msg::Pose mustard_bottle_pose;
  mustard_bottle_pose.orientation.w = 1.0;
  mustard_bottle_pose.position.x = 0.5;
  mustard_bottle_pose.position.y = -0.3;
  mustard_bottle_pose.position.z = 0.0;
  return createCollisionObject("mustard_bottle", "package://hot_dog/mustard-bottle/mustard-bottle.dae",
                               mustard_bottle_pose);
}

std::vector<moveit_msgs::msg::ObjectColor> HotDogScenario::createHotDogColors()
{
  moveit_msgs::msg::ObjectColor bun_color;
  bun_color.color.r = 0.907f;
  bun_color.color.g = 0.608f;
  bun_color.color.b = 0.244f;
  bun_color.color.a = 1.0f;
  bun_color.id = "bun";

  moveit_msgs::msg::ObjectColor sausage_color;
  sausage_color.color.r = 0.906f;
  sausage_color.color.g = 0.302f;
  sausage_color.color.b = 0.233f;
  sausage_color.color.a = 1.0f;
  sausage_color.id = "sausage";

  moveit_msgs::msg::ObjectColor mustard_color;
  mustard_color.color.r = 0.906f;
  mustard_color.color.g = 0.833f;
  mustard_color.color.b = 0.109f;
  mustard_color.color.a = 1.0f;
  mustard_color.id = "mustard";

  moveit_msgs::msg::ObjectColor mustard_bottle_color;
  mustard_bottle_color.color.r = 0.906f;
  mustard_bottle_color.color.g = 0.833f;
  mustard_bottle_color.color.b = 0.109f;
  mustard_bottle_color.color.a = 1.0f;
  mustard_bottle_color.id = "mustard_bottle";

  return { bun_color, sausage_color, mustard_color, mustard_bottle_color };
}
