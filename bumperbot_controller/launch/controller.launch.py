from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():

    wheel_radius_arg = DeclareLaunchArgument(

        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(

        "wheelbase",     
        default_value="0.17"
    )
    
    max_rpm_arg = DeclareLaunchArgument(
        "max_rpm",
        default_value="1000.0"
    )

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

    pkg_share = FindPackageShare(package='bumperbot_controller').find('bumperbot_controller')

    use_python = LaunchConfiguration("use_python")
    use_sim_time = LaunchConfiguration('use_sim_time')
    wheel_radius = LaunchConfiguration("wheel_radius") #reading value of arg
    wheel_separation = LaunchConfiguration("wheelbase") #reading value of arg
    max_rpm = LaunchConfiguration("max_rpm") #reading value of arg
    robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 


    use_simple_controller  = LaunchConfiguration("use_simple_controller")
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "simple_velocity_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    start_robot_localization_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}])



    simple_controller_cpp = Node(
        package="bumperbot_controller",
        executable="controller_cpp",
        parameters=[{"wheel_radius": wheel_radius,
                     "wheelbase":  wheel_separation,
                     "max_rpm": max_rpm
                     }])
    
    # noisy_controller_launch = OpaqueFunction(function=noisy_controller)
    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        max_rpm_arg,
        declare_use_sim_time_cmd,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        simple_controller_cpp,
        start_robot_localization_cmd,
        # noisy_controller_launch
    ])
    