#include <motion_planning/motion_planning.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace
{
// Create a ROS logger
const auto logger = rclcpp::get_logger("hello_roscon");
}  // namespace

MotionPlanning::MotionPlanning(const std::string& name, const rclcpp::NodeOptions& options)
  : node_{ std::make_shared<rclcpp::Node>(name, options) }
{
  bun_ = hot_dog::HotDogFactory::createBun();
  sausage_ = hot_dog::HotDogFactory::createSausage();
  mustard_ = hot_dog::HotDogFactory::createMustard();
  mustard_bottle_ = hot_dog::HotDogFactory::createMustardBottle();
  move_group_interface_ = std::make_unique<MoveGroupInterface>(MoveGroupInterface(node_, "ur_manipulator"));
  hot_dog_colors_ = hot_dog::HotDogFactory::createHotDogColors();
};

void MotionPlanning::setUpScene()
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

  planning_scene_interface_.applyCollisionObjects({ bun_, sausage_, mustard_bottle_ }, hot_dog_colors_);
  move_group_interface_->attachObject(mustard_bottle_.id);
}

bool MotionPlanning::plan(const geometry_msgs::msg::Pose& pose, MoveGroupInterface::Plan& plan)
{
  move_group_interface_->setPoseTarget(pose);
  bool success = static_cast<bool>(move_group_interface_->plan(plan));
  return success;
}

void MotionPlanning::execute(const MoveGroupInterface::Plan& plan)
{
  move_group_interface_->execute(plan);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MotionPlanning::getNodeBaseInterface()
{
  return node_->get_node_base_interface();
}

void MotionPlanning::applyMustard()
{
  planning_scene_interface_.applyCollisionObjects({ mustard_ }, hot_dog_colors_);
}
