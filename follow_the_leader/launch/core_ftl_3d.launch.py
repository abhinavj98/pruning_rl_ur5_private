#!/usr/bin/env python3
import launch
from launch import LaunchDescription, LaunchContext
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetLaunchConfiguration,
    EmitEvent,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    core_params_file = LaunchConfiguration("core_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")

    # Load the YAML files
    package_dir = get_package_share_directory("follow_the_leader")
    core_params_path = os.path.join(package_dir, "config", "ftl_ur5e.yaml")
    camera_params_path = PythonExpression(["'{}'.format(r'", camera_params_file, "')"])

    params_arg = DeclareLaunchArgument(
        "core_params_file",
        default_value=core_params_path,
        description="Path to the YAML file containing node parameters",
    )
    camera_params_arg = DeclareLaunchArgument(
        "camera_params_file",
        default_value=camera_params_path,
        description="Path to the YAML file containing camera parameters",
    )

    state_manager_node = Node(
        package="follow_the_leader",
        executable="state_manager",
        output="screen",
        parameters=[core_params_file],
    )

    image_processor_node = Node(
        package="follow_the_leader",
        executable="image_processor",
        output="screen",
        parameters=[core_params_file, camera_params_file],
    )

    optical_flow_node = Node(
        package="follow_the_leader",
        executable="optical_flow",
        output="screen",
        # parameters=[core_params_file, camera_params_file],
    )

    point_tracker_node = Node(
        package="follow_the_leader",
        executable="point_tracker",
        output="screen",
        parameters=[core_params_file, camera_params_file],
    )

    modeling_node = Node(
        package="follow_the_leader",
        executable="model",
        output="screen",
        parameters=[core_params_file, camera_params_file],
    )

    controller_node = Node(
        package="follow_the_leader",
        executable="controller_3d",
        # output='screen',
        parameters=[core_params_file],
    )

    servoing_node = Node(
        package="follow_the_leader",
        executable="visual_servoing",
        output="screen",
        parameters=[core_params_file],
    )

    goal_publisher_node = Node(
        package="follow_the_leader",
        executable="goal_publisher",
        output="screen",
        # parameters=[core_params_file],
    )

    # teleop_node = Node(
    #     package='teleop_twist_keyboard',
    #     executable="teleop_twist_keyboard",
    #     output='screen',
    #     prefix='xterm -e',
    #
    #
    # )

    transform_node = Node(
        package="follow_the_leader",
        executable="run_transform",
        )
    tf_node_mount = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        prefix = ['bash -c \'sleep 7; $0 $@\''],
        arguments="-0.115 0.065 0.03 0 0 0 1 tool0 camera_mount_center".split(' '),
        output = 'screen',
        )

    tf_node_mount_to_cam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        prefix=['bash -c \'sleep 7; $0 $@\''],
        arguments="0.028 0 0.0193 0 0 0 1 camera_mount_center camera_link".split(' '),
        # Z is camera thickness (23mm) minus glass (3.7mm)
    )


    tf_node_c = Node(
        package="tf2_ros",
        executable="static_transform_publisher",

        prefix=['bash -c \'sleep 7; $0 $@\''],
        arguments="0. 0. 0. 0. 0. 0. 1. camera_link camera_color_optical_frame".split(' '),
    )

    tf_node_endpoint = Node(
        package="tf2_ros",
        executable="static_transform_publisher",

        prefix=['bash -c \'sleep 7; $0 $@\''],
        arguments="-0.0931 0. 0.128 0. 0. 0. 1. tool0 endpoint".split(' '),
    )

    tf_node_world_sim = Node(
        package="tf2_ros",
        executable="static_transform_publisher",

        prefix=['bash -c \'sleep 7; $0 $@\''],
        arguments="0.5 0 0.757 0. 0. 0. 1. world_sim world".split(' '),
    )

    tf_node_aruco = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        prefix=['bash -c \'sleep 7; $0 $@\''],
        arguments="0. 0. 0. 1. 0. 0. 0. camera_color_optical_frame aruco_frame".split(' '),
    )

    optical_flow_srv = Node(
        package="follow_the_leader",
        executable="optical_flow_srv",
        
    )

    controller_rl = Node(
        package="follow_the_leader",
        executable="controller_rl",
        prefix=['bash -c \'sleep 7; $0 $@\'']
    )

#-0.07, -0.73, 0.59
    return LaunchDescription(
        [
            params_arg,
            camera_params_arg,
            state_manager_node,
            # image_processor_node,
            # optical_flow_node,
            # goal_publisher_node,
            #point_tracker_node,
            #modeling_node,
            #controller_node,
            #servoing_node,
            #teleop_node
            tf_node_mount,
            tf_node_mount_to_cam,
            # tf_node_b,
            tf_node_c,
            tf_node_endpoint,
            tf_node_world_sim,
            optical_flow_srv,
            # controller_rl

        ]
    )
