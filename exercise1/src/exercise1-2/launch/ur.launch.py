import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    ur_driver_launch_arguments = {
        "ur_type": "ur5e",
        "robot_ip": "0.0.0.0",
        "use_mock_hardware": "true",
        "launch_rviz": "false"
    }

    launch_ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ur_robot_driver'), 'launch', 'ur_control.launch.py')]),
        launch_arguments=ur_driver_launch_arguments.items()
    )

    ur_moveit_launch_arguments = {
        "ur_type": "ur5e",
        "launch_rviz": "false" 
    }

    launch_ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ur_moveit_config'), 'launch', 'ur_moveit.launch.py')]),
        launch_arguments=ur_moveit_launch_arguments.items()
    )

    moveit_config = (
      MoveItConfigsBuilder(
        robot_name="ur", package_name="ur_moveit_config"
      )
      .robot_description_semantic(file_path="srdf/ur.srdf.xacro", mappings={"name": "ur5e"})
      .to_moveit_configs()
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("exercise1-1"), "launch", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
            ]
    )
    
    return LaunchDescription([
        launch_ur_driver,
        launch_ur_moveit,
        rviz_node
        ])
