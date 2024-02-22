import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    servo_params = os.path.join(
        get_package_share_directory('grr_bringup'),
        'config',
        'servo_params.yaml'
    )
    
    print(f"HELLO {servo_params}")
    ld = LaunchDescription()
    
    ld.add_action(
        Node(
            package='grr_hardware_python',
            executable='robot',
            name='robot',
            output='screen',
            emulate_tty=True,
            namespace='grr',
            parameters=[servo_params]
        )
    )
    
    
    return ld
    
    
