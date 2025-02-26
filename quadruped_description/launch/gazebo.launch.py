from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument



def generate_launch_description():
        # Get the share directory of the quadruped_description package
    share_dir = get_package_share_directory('quadruped_description')

    # Get the path to the xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'bot_urdf_hii.xacro')

    # Process the xacro file to get the robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Create the robot_state_publisher node with the processed robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'FusionComponent',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    arm_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        joint_broad_spawner,
        arm_cont_spawner
    ])
