import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        name="moveit_planning",
        package="moveit_planning",
        executable="moveit_planning1-1",
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
