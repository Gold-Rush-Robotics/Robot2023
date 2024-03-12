import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node

import xacro

def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('grr_description'))
    xacro_file = os.path.join(pkg_path,'urdf', 'robots','bloodstone.urdf.xacro')
    grr_description_config = xacro.process_file(xacro_file)

    print(grr_description_config.toxml())
    
    # Create a robot_state_publisher node
    params = {'robot_description': grr_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Start Rviz2 with basic view
    # rviz2_config_path = os.path.join(get_package_share_directory('grr_description'), 'config/isaac.rviz')
    # run_rviz2 = ExecuteProcess(
    #     cmd=['rviz2', '-d', rviz2_config_path],
    #     output='screen'
    # )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time if true'),

        node_robot_state_publisher,
        # run_rviz2
    ])
