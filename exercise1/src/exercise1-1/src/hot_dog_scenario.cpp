
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
  hot_dog_colors_ = createHotDogColors();
};

void HotDogScenario::placeHotDog()
{
  planning_scene_interface_.applyCollisionObjects({ bun_, sausage_ }, hot_dog_colors_);
}

void HotDogScenario::placeSausage()
{
  sausage_.pose.position.y = 0.0;
  planning_scene_interface_.applyCollisionObjects({ sausage_ }, hot_dog_colors_);
}

geometry_msgs::msg::Pose HotDogScenario::getPickPose()
{
  geometry_msgs::msg::Pose pick_pose;
  pick_pose.orientation.x = 1.0;
  pick_pose.orientation.y = 0.0;
  pick_pose.orientation.z = 0.0;
  pick_pose.orientation.w = 0.0;
  pick_pose.position.x = 0.3;
  pick_pose.position.y = 0.4;
  pick_pose.position.z = 0.1;
  return pick_pose;
}
geometry_msgs::msg::Pose HotDogScenario::getPlacePose()
{
  geometry_msgs::msg::Pose place_pose;
  place_pose.orientation.x = 1.0;
  place_pose.orientation.y = 0.0;
  place_pose.orientation.z = 0.0;
  place_pose.orientation.w = 0.0;
  place_pose.position.x = 0.3;
  place_pose.position.y = 0.0;
  place_pose.position.z = 0.15;
  return place_pose;
}

moveit_msgs::msg::CollisionObject HotDogScenario::getSausage() const
{
  return sausage_;
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
  sausage_pose.position.y = 0.4;  // Offset for exercise 1-1
  sausage_pose.position.z = 0.0;
  return createCollisionObject("sausage", "package://hot_dog/hotdog/sausage.dae", sausage_pose);
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

  return { bun_color, sausage_color };
}
