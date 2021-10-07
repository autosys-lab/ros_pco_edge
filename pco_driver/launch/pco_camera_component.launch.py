import launch
from launch import LaunchDescription
from launch import LaunchIntrospector
from launch import LaunchService

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('pco_driver') + '/config/camera.yaml'),
        ComposableNodeContainer(
            name="pco_camera_driver_container",
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='pco_driver',
                    plugin='PCODriver',
                    name='pco_camera_driver_node',
                    parameters=[
                        {"camera_calibration_file": LaunchConfiguration('camera_calibration_file')}
                    ])
            ],
            output='screen'
        )
    ])
