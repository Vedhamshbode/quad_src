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
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition



def generate_launch_description():

    # activate by default control system for simulated robot
    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false"
    )
    is_sim =   LaunchConfiguration("is_sim")
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
        parameters=[{'robot_description': robot_urdf}],
        condition = UnlessCondition(is_sim)
    )

    controller_manager= Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description":robot_urdf,
             "use_sim_time":is_sim},
             os.path.join(
                 get_package_share_directory("quadruped_description"),
                 "config",
                 "my_controllers.yaml"
             )
        ],
        condition = UnlessCondition(is_sim)
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

    lb_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lb_cont"],
    )

    lf_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lf_cont"],
    )

    rb_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rb_cont"],
    )

    rf_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rf_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name = "rviz2",
        # arguments=["-d", ""]
    )

    return LaunchDescription([
        is_sim_arg,
        robot_state_publisher_node,
        controller_manager,
        # gazebo_server,
        # gazebo_client,
        # urdf_spawn_node,
        joint_broad_spawner,
        lb_cont_spawner,
        lf_cont_spawner,
        rb_cont_spawner,
        rf_cont_spawner,
        rviz_node
    ])