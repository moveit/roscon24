#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/msg/mesh.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

namespace hot_dog
{
class HotDogFactory
{
public:
  /* @brief Creates a collision object for a hot dog bun */
  static moveit_msgs::msg::CollisionObject createBun();

  /* @brief Creates a collision object for a hot dog sausage, offset from the bun */
  static moveit_msgs::msg::CollisionObject createOffsetSausage();

  /* @brief Creates a collision object for a hot dog sausage */
  static moveit_msgs::msg::CollisionObject createSausage();

  /* @brief Creates a collision object for some mustard */
  static moveit_msgs::msg::CollisionObject createMustard();

  /* @brief Apply collision objects with some predefined colors for hot dogs */
  static void addCollisionObjectsToScene(moveit::planning_interface::PlanningSceneInterface& psi,
                                         const std::vector<moveit_msgs::msg::CollisionObject>& collision_objects);

private:
  /* @brief Common method to create collison objects */
  static moveit_msgs::msg::CollisionObject createCollisionObject(const std::string& name, const std::string& mesh_path,
                                                                 const geometry_msgs::msg::Pose& pose);
};
}  // namespace hot_dog
