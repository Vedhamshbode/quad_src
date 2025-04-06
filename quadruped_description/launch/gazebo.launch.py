from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration




def generate_launch_description():
        # Get the share directory of the quadruped_description package
    share_dir = get_package_share_directory('quadruped_description')

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

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            # "world": world_file,
            "pause": "true",
            }.items(),
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

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'quad_dog',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.00',
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

    ik_service_node = Node(
        package="ik",
        executable="ik_service",
        output = "screen"
    )

    return LaunchDescription([
        # DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time if true'),
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),

        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),

        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),
    
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate, 
                         'scan_mode': scan_mode}],
            output='screen'),
    
        robot_state_publisher_node,
        # pause_simulation,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        rviz2_node,
        TimerAction(period=2.0, actions=[static_tf_node]),
        joint_broad_spawner,
        lb_cont_spawner,
        lf_cont_spawner,
        rb_cont_spawner,
        rf_cont_spawner,
        ik_service_node
    ])
