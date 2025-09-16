#!/usr/bin/env python3
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Hard-coded URDF path
    urdf_path = "/home/effica/myagv_ros/src/my_agv_description/urdf/myAGV.urdf"
    robot_description = Path(urdf_path).read_text()

    # Hard-coded RViz config path
    #my_agv_examples
    rviz_config = "/home/effica/myagv_ros/src/my_agv_examples/rviz/rviz.rviz"

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False,
            'publish_frequency': 50.0
        }]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'rate': 50.0}]
    )

    fake_odometry = Node(
        package='my_agv_examples',
        executable='fake_odometry',
        name='fake_odometry',
        output='screen'
    )

    square_path = Node(
        package='my_agv_examples',
        executable='square_path',
        name='square_path',
        output='screen',
        parameters=[{
            'side_length': 1.0,
            'linear_speed': 0.2,
            'angular_speed': 0.3
        }]
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        fake_odometry,
        square_path,
        rviz2
    ])
