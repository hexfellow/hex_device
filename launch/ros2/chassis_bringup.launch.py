#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Declare the launch arguments

    # Note: ROS2 launch does not provide control over node shutdown order.
    # It is recommended to launch xpkg_bridge_node separately to ensure proper
    # command forwarding during the shutdown phase and prevent premature termination
    # of the bridge node that may cause shutdown commands to fail.
    enable_bridge = DeclareLaunchArgument(
        'enable_bridge',
        default_value='false',
        description='Whether to enable the xpkg_bridge node, you can set it to false if you want to launch the node separately.'
    )

    url = DeclareLaunchArgument(
        'url',
        default_value='',
        description='The URL of the robot.'
    )

    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the chassis state.'
    )

    frame_id = DeclareLaunchArgument(
        'frame_id',
        default_value='base_link',
        description='Frame ID of the chassis.'
    )

    simple_mode = DeclareLaunchArgument(
        'simple_mode',
        default_value='true',
        description='Simple mode of the chassis.'
    )
    rate_ros = DeclareLaunchArgument(
        'rate_ros',
        default_value='100.0',
        description='Rate of the ROS2 node.'
    )

    # Define the node
    xpkg_bridge_node = Node(
        package='xpkg_bridge',
        executable='xnode_bridge',
        name='xnode_bridge',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
        parameters=[{
            'url': LaunchConfiguration('url'),
            'read_only': LaunchConfiguration('read_only'),
        }],
        remappings=[
            # subscribe
            ('/ws_down', '/ws_down'),
            # publish
            ('/ws_up', '/ws_up')
        ]
    )

    hex_chassis_node = Node(
        package='hex_device',
        executable='chassis_trans',
        name='hex_chassis',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'simple_mode': LaunchConfiguration('simple_mode'),
            'rate_ros': LaunchConfiguration('rate_ros'),
        }],
        remappings=[
            # subscribe
            ('/xtopic_chassis/motor_states', '/xtopic_chassis/motor_states'),
            ('/xtopic_chassis/odom', '/xtopic_chassis/odom'),
            # publish
            ('/xtopic_chassis/joint_cmd', '/xtopic_chassis/joint_cmd'),
            ('/xtopic_chassis/cmd_vel', '/cmd_vel'),
            ('/xtopic_chassis/clear_err', '/xtopic_chassis/clear_err')
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        enable_bridge,
        url,
        read_only,
        frame_id,
        simple_mode,
        rate_ros,
        xpkg_bridge_node,
        hex_chassis_node
    ])