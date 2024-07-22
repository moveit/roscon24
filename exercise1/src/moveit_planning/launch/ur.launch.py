import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ur_driver_launch_arguments = {
        "ur_type": "ur5e",
        "robot_ip": "0.0.0.0",
        "use_fake_hardware": "true",
        "launch_rviz": "false"
    }

    launch_ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ur_robot_driver'), 'launch', 'ur_control.launch.py')]),
        launch_arguments=ur_driver_launch_arguments.items()
    )

    ur_moveit_launch_arguments = {
        "ur_type": "ur5e",
        "launch_rviz": "true"
    }

    launch_ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ur_moveit_config'), 'launch', 'ur_moveit.launch.py')]),
        launch_arguments=ur_moveit_launch_arguments.items()
    )
    
    return LaunchDescription([
        launch_ur_driver,
        launch_ur_moveit
        ])
