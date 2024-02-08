import os
import sys
from glob import glob
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():
    pkg_dir = get_package_share_directory('ndt_scan_matcher')
    rviz_config_file = os.path.join(pkg_dir, 'rviz','test.rviz')
    list = [
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [os.path.join(get_package_share_directory("data_logger"), "launch"), "/data_logger.launch.py"]
        #     ),
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            # parameters=[{'use_sim_time': True}]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "-0.2",
                "--y",
                "0.0",
                "--z",
                "0.0",
                "--yaw",
                "0.0",
                "--pitch",
                "0.0",
                "--roll",
                "0.0",
                "--frame-id",
                "base_link",
                "--child-frame-id",
                "tugbot/scan_omni/scan_omni",
            ],
            # parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ndt_scan_matcher',
            executable='ndt_scan_matcher',
            namespace='',
            output="screen",
            parameters=[os.path.join(pkg_dir, "config", "test_ndt_scan_matcher_param.yaml")],
            respawn=True,
        ),
        Node(
            package='grid_point_observer',
            executable='grid_point_observer',
            namespace='',
            output="screen",
            parameters=[os.path.join(get_package_share_directory('grid_point_observer'), "config", "test_grid_point_observer_param.yaml")],
            respawn=True,
        )
    ]

    return LaunchDescription(list)