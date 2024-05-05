#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    EmitEvent,
    ExecuteProcess,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression, AndSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    use_sim = LaunchConfiguration("use_sim")


    package_dir = get_package_share_directory("rl_pruning_controller")

    ur_type_arg = DeclareLaunchArgument(
        "ur_type", default_value="ur5e", description="Robot description name (consistent with ur_control.launch.py)"
    )
    robot_ip_arg = DeclareLaunchArgument("robot_ip", default_value="169.254.174.50", description="Robot IP")

    use_sim_arg = DeclareLaunchArgument(
        "use_sim", default_value="false", description="If true, uses the fake controllers"
    )

    ur_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(package_dir, "ur_startup.launch.py")
        ),
        launch_arguments=[
            ("robot_ip", robot_ip),
            ("ur_type", ur_type),
            ("use_fake_hardware", use_sim),
        ],
    )

    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    # 
    jacobian_server = Node(
        package="rl_pruning_controller",
        executable="jacobian_publisher",
        name="jacobian_publisher",
    )


    return LaunchDescription(
        [
            ur_type_arg,
            robot_ip_arg,
            use_sim_arg,
            ur_launch,
            # joy_node
            jacobian_server,
        ]
    )
