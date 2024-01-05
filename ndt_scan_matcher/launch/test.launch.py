import os
import sys
from glob import glob
import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    share_dir = get_package_share_directory('ndt_scan_matcher')
    rviz_config_file = os.path.join(share_dir, 'rviz','default.rviz')
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
    rviz2_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
    )
    base_link_to_imu_link_node = launch_ros.actions.Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "--x",
                "0.0",
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
                "/imu_link",
            ],
    )
    ndt_node = launch_ros.actions.Node(
        package='ndt_scan_matcher',
        executable='ndt_scan_matcher',
        namespace='',
        output="screen",
        parameters=[os.path.join(share_dir, "config", "ndt_scan_matcher_param.yaml")],
        respawn=True,
    )
    gpo_node = launch_ros.actions.Node(
        package='grid_point_observer',
        executable='grid_point_observer',
        namespace='',
        output="screen",
        parameters=[os.path.join(get_package_share_directory('grid_point_observer'), "config", "grid_point_observer_param.yaml")],
        respawn=True,
    )
    return launch.LaunchDescription(
        params +
        [wit_node, base_link_to_imu_link_node,ndt_node,gpo_node, rviz2_node]
    )