#include "hot_dog/hot_dog_collision_object.hpp"

namespace hot_dog
{

moveit_msgs::msg::CollisionObject HotDogFactory::createCollisionObject(const std::string& name,
                                                                       const std::string& mesh_path,
                                                                       const geometry_msgs::msg::Pose& pose)
{
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "world";
  collision_object.id = name;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
  m->scale(0.05);  // Full size hot dog is much too large and must be scaled
  shapes::ShapeMsg mesh_msg;
  shapes::constructMsgFromShape(m, mesh_msg);
  const auto mesh = boost::get<shape_msgs::msg::Mesh>(mesh_msg);

  collision_object.meshes.push_back(mesh);
  collision_object.operation = collision_object.ADD;
  collision_object.pose = pose;

  return collision_object;
}

moveit_msgs::msg::CollisionObject HotDogFactory::createBun()
{
  geometry_msgs::msg::Pose bun_pose;
  bun_pose.orientation.w = 1.0;
  bun_pose.position.x = 0.3;
  bun_pose.position.y = 0.0;
  bun_pose.position.z = 0.2;
  return HotDogFactory::createCollisionObject("bun", "package://hot_dog/hotdog/bun.dae", bun_pose);
}

moveit_msgs::msg::CollisionObject HotDogFactory::createOffsetSausage()
{
  geometry_msgs::msg::Pose offset_sausage_pose;
  offset_sausage_pose.orientation.w = 1.0;
  offset_sausage_pose.position.x = 0.25;
  offset_sausage_pose.position.y = 0.445;
  offset_sausage_pose.position.z = 0.04;
  return HotDogFactory::createCollisionObject("sausage", "package://hot_dog/hotdog/sausage.dae", offset_sausage_pose);
}

moveit_msgs::msg::CollisionObject HotDogFactory::createSausage()
{
  geometry_msgs::msg::Pose sausage_pose;
  sausage_pose.orientation.w = 1.0;
  sausage_pose.position.x = 0.25;
  sausage_pose.position.y = 0.045;
  sausage_pose.position.z = 0.04;
  return HotDogFactory::createCollisionObject("sausage", "package://hot_dog/hotdog/sausage.dae", sausage_pose);
}

moveit_msgs::msg::CollisionObject HotDogFactory::createMustard()
{
  geometry_msgs::msg::Pose mustard_pose;
  mustard_pose.orientation.w = 1.0;
  mustard_pose.position.x = 0.3;
  mustard_pose.position.y = 0.035;
  mustard_pose.position.z = -0.305;
  return HotDogFactory::createCollisionObject("mustard", "package://hot_dog/hotdog/mustard.dae", mustard_pose);
}

void HotDogFactory::addCollisionObjectsToScene(moveit::planning_interface::PlanningSceneInterface& psi,
                                               const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects)
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

  const std::vector<moveit_msgs::msg::ObjectColor> hotdog_colors{ bun_color, sausage_color, mustard_color };
  psi.applyCollisionObjects(collision_objects, hotdog_colors);
}
}  // namespace hot_dog
