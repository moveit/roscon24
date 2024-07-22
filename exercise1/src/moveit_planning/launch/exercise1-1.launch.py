import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Define xacro mappings for the robot description file
    urdf_arguments = {
        "name": "ur",
        "ur_type": "ur5e"
    }

    # Define xacro mappings for the robot description semantic file
    srdf_arguments = {
        "name": "ur"
    }

    moveit_config = (
      MoveItConfigsBuilder(
        "ur5e", package_name="ur_moveit_config"
      )
      .robot_description(file_path="../ur_description/urdf/ur.urdf.xacro", mappings=urdf_arguments)
      .robot_description_semantic(file_path="srdf/ur.srdf.xacro", mappings=srdf_arguments)
      .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
      .trajectory_execution(file_path="config/controllers.yaml")
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
