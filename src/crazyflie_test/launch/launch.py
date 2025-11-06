import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

package_name = 'crazyflie_test'

def generate_launch_description():
    script = LaunchConfiguration('script')
    backend = LaunchConfiguration('backend')

    script_launch_arg = DeclareLaunchArgument(
        'script'
    )

    backend_launch_arg = DeclareLaunchArgument(
        'backend',
        default_value='cpp'
    )

    crazyflies_yaml_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'crazyflies.yaml')
    motion_capture_yaml_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'motion_capture.yaml')
    server_yaml_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'server.yaml')

    crazyflie = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('crazyflie'), 'launch'),
            '/launch.py']),
        launch_arguments={
            'crazyflies_yaml_file': crazyflies_yaml_path,
            'motion_capture_yaml_file': motion_capture_yaml_path,
            'server_yaml_file': server_yaml_path,
            'backend': backend,
        }.items()
    )

    example_node = Node(
        package=package_name,
        executable=script,
        name=script,
        parameters=[{
            'use_sim_time': PythonExpression(["'", backend, "' == 'sim'"]),
        }]
    )

    return LaunchDescription([
        script_launch_arg,
        backend_launch_arg,
        crazyflie,
        example_node
    ])
