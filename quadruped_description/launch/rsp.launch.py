import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    # Declare the use_sim_time argument, with a default value of 'false'
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Path to the xacro file
    pkg_path = get_package_share_directory('quadruped_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'bot_urdf_hii.xacro')

    # Process the xacro file using the xacro command
    robot_description_config = Command(['xacro', xacro_file, 'use_sim_time:=', use_sim_time])

    # Define parameters for the robot_state_publisher node
    params = {'robot_description': robot_description_config}

    # Create the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        node_robot_state_publisher
    ])

