import launch
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_calibration_file',
            default_value='file://' + get_package_share_directory('pco_driver') + '/config/camera.yaml'),
        Node(
            package='pco_driver',
            executable='pco_camera_driver_node',
            parameters=[
                {"camera_calibration_file": LaunchConfiguration('camera_calibration_file')}
            ]
        ),
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            output='screen'
        )
    ])
