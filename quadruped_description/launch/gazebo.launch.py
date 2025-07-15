from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit



def generate_launch_description():
        # Get the share directory of the quadruped_description package
    share_dir = get_package_share_directory('quadruped_description')
    gz_args = LaunchConfiguration('gz_args', default='-r -v 1 empty.sdf')

    rviz_config_dir = os.path.join(
            share_dir,
            'config',
            'lidar.rviz')

    # Get the path to the xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'bot_urdf_hii.xacro')
    # pause_simulation = DeclareLaunchArgument(
    #     'paused', default_value='false', description='Start Gazebo in paused mode'
    # )

    # Process the xacro file to get the robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()
    world_file = os.path.join(share_dir, 'worlds', 'eyrc.world')

    #  ******************LIDAR PARAMETERS *****************

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    #  ******************LIDAR PARAMETERS *****************


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
 
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'dog', '-allow_renaming', 'true'],
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_dir],
        output = "screen"
    )
    
    static_tf_node = Node(
        package='quadruped_description',
        executable='static_tf.py',
    )

    static_tf_node = Node(
        package='quadruped_description',
        executable='static_tf.py',
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

    ik_service_node = Node(
        package="ik",
        executable="ik_service",
        output = "screen"
    )
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    return LaunchDescription([
         IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', [gz_args, ' -r -v 1 empty.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_broad_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_broad_spawner,
                on_exit=[ TimerAction(
                period=1.0,  # delay in seconds
                actions=[
                    lb_cont_spawner,
                    lf_cont_spawner,
                    rb_cont_spawner,
                    rf_cont_spawner
                ]
            )],
            )
        ),
        bridge,
        robot_state_publisher_node,
        gz_spawn_entity,
        # Delay fixed frame tf
        TimerAction(period=2.0, actions=[static_tf_node]),

        # Optional visualization
        rviz2_node,
    ])
