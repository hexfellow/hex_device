#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Declare the launch arguments
    url = DeclareLaunchArgument(
        'url',
        default_value='ws://0.0.0.0:8439',
        description='The URL of the robot.'
    )

    read_only = DeclareLaunchArgument(
        'read_only',
        default_value='false',
        description='Whether to read only the arm state.'
    )


    rate_ros = DeclareLaunchArgument(
        'rate_ros',
        default_value='300',
        description='The rate of the node.'
    )

    joint_config = FindPackageShare('hex_device').find(
        'hex_device') + '/config/joints.json'
    joint_config_path = DeclareLaunchArgument(
        'joint_config_path',
        default_value=joint_config,
        description='The path to the joint config file.'
    )
    
    init_pose_file_path = FindPackageShare('hex_device').find(
        'hex_device') + '/config/init_pose.json'
    init_pose_path = DeclareLaunchArgument(
        'init_pose_path',
        default_value=init_pose_file_path,
        description='The path to the init pose file.'
    )
    
    gripper_type = DeclareLaunchArgument(
        'gripper_type',
        default_value='0',
        description='The type of the Gripper (integer).'
    )
    
    arm_series = DeclareLaunchArgument(
        'arm_series',
        default_value='16',
        description='The series of the Archer (integer).'
    )

    # Define the node
    xpkg_bridge_node = Node(
        package='xpkg_bridge',
        executable='xnode_bridge',
        name='xnode_bridge',
        output='screen',
        emulate_tty=True,
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

    hex_device_node = Node(
        package='hex_device',
        executable='arm_trans',
        name='hex_device',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'joint_config_path': LaunchConfiguration('joint_config_path'),
            'init_pose_path': LaunchConfiguration('init_pose_path'),
            'gripper_type': LaunchConfiguration('gripper_type'),
            'arm_series': LaunchConfiguration('arm_series'),
        }],
        remappings=[
            ('/xtopic_arm/joints_cmd', '/xtopic_arm/joints_cmd'),
            ('/xtopic_arm/joint_states', '/xtopic_arm/joint_states'),
            ('/xtopic_arm/gripper_cmd', '/xtopic_arm/gripper_cmd'),
            ('/xtopic_arm/gripper_states', '/xtopic_arm/gripper_states'),
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        # arguments
        url,
        read_only,
        rate_ros,
        joint_config_path,
        init_pose_path,
        gripper_type,
        arm_series,
        # nodes
        xpkg_bridge_node,
        hex_device_node
    ])