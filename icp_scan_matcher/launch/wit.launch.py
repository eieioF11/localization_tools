import os
import sys
from glob import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    params = [
        launch.actions.DeclareLaunchArgument('port',
                                            default_value="/dev/ttyUSB0"),
        launch.actions.DeclareLaunchArgument('baud_rate',
                                            default_value="115200"),
        launch.actions.DeclareLaunchArgument('frame_id',
                                            default_value="/imu_link"),
        launch.actions.DeclareLaunchArgument('publish_hz',
                                            default_value="200.0"),
    ]
    wit_node = launch_ros.actions.Node(
        package="wit_node", executable="wit_node",
        parameters=[{
            "port": launch.substitutions.LaunchConfiguration('port'),
            "baud_rate": launch.substitutions.LaunchConfiguration('baud_rate'),
            "frame_id": launch.substitutions.LaunchConfiguration('frame_id'),
            "publish_hz": launch.substitutions.LaunchConfiguration('publish_hz')
        }],
    )
    return launch.LaunchDescription(
        params + 
        [wit_node]
    )