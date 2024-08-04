from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    moveit_config = (
      MoveItConfigsBuilder(
        robot_name="ur", package_name="ur_moveit_config"
      )
      .robot_description_semantic(file_path="srdf/ur.srdf.xacro", mappings={"name": "ur5e"})
      .to_moveit_configs()
    )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="exercise1_2",
        package="exercise1-2",
        executable="exercise1-2_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        move_group_demo
        ])
