from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('quadruped_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'bot_urdf_hii.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    rviz_config_file = os.path.join(share_dir, 'config', 'display.rviz')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

        # Define initial pose parameters
    initial_x = DeclareLaunchArgument(
        name='initial_x',
        default_value='0.0'
    )
    initial_y = DeclareLaunchArgument(
        name='initial_y',
        default_value='0.0'
    )
    initial_z = DeclareLaunchArgument(
        name='initial_z',
        default_value='10.0'
    )
    initial_roll = DeclareLaunchArgument(
        name='initial_roll',
        default_value='1.57'
    )
    initial_pitch = DeclareLaunchArgument(
        name='initial_pitch',
        default_value='0.0'
    )
    initial_yaw = DeclareLaunchArgument(
        name='initial_yaw',
        default_value='0.0'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf, 
            'initial_x': LaunchConfiguration('initial_x'),  # Set initial x position
            'initial_y': LaunchConfiguration('initial_y'),  # Set initial y position
            'initial_z': LaunchConfiguration('initial_z'),  # Set initial z position
            'initial_roll': LaunchConfiguration('initial_roll'),  # Set initial roll angle
            'initial_pitch': LaunchConfiguration('initial_pitch'),  # Set initial pitch angle
            'initial_yaw': LaunchConfiguration('initial_yaw')}
            
        ]
    )

    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        gui_arg,
        initial_x,
        initial_y,
        initial_z,
        initial_roll,
        initial_pitch,
        initial_yaw,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
