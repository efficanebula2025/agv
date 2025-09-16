#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the URDF from my_agv_description package
    description_pkg = get_package_share_directory('my_agv_description')
    urdf_path = os.path.join(description_pkg, 'urdf', 'myAGV.urdf')
    
    # Read URDF file
    with open(urdf_path, 'r') as file:
        robot_description = file.read()
    
    # Get RViz config path
    examples_pkg = get_package_share_directory('my_agv_examples')
    rviz_config = os.path.join(examples_pkg, 'rviz', 'myagv_config.rviz')
    
    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        
        # Floor Marker Node (always included)
        Node(
            package='my_agv_examples',
            executable='floor_marker',
            name='floor_marker',
            output='screen'
        ),
        
        # Fake Odometry Node
        Node(
            package='my_agv_examples',
            executable='fake_odometry',
            name='fake_odometry',
            output='screen'
        ),
        
        # Keyboard Teleop Node
        Node(
            package='my_agv_examples',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            output='screen',
            parameters=[{
                'speed': 0.3,
                'turn': 0.5,
                'repeat_rate': 20.0
            }],
            prefix='xterm -e'  # Opens in new terminal for keyboard input
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
            output='screen'
        )
    ])