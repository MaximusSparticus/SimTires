#!/usr/bin/env python3
"""
Launch file for SimTires teleoperation with joystick control.
Starts both the joy_node and teleop_driver together.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments for configuration
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'joy_config',
            default_value='xbox',
            description='Joystick configuration (xbox, ps4, etc.)'
        ),
        DeclareLaunchArgument(
            'autorepeat_rate',
            default_value='20.0',
            description='Autorepeat rate for joystick messages (Hz)'
        ),
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.1',
            description='Deadzone for joystick axes'
        ),
        DeclareLaunchArgument(
            'topic_name',
            default_value='/inputs/driver_inputs',
            description='Topic name for driver inputs output'
        ),
        DeclareLaunchArgument(
            'vehicle_namespace',
            default_value='',
            description='Vehicle namespace (for multi-vehicle setups)'
        ),
        
        # Joy node - publishes joystick data
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': LaunchConfiguration('joy_dev'),
                'deadzone': LaunchConfiguration('deadzone'),
                'autorepeat_rate': LaunchConfiguration('autorepeat_rate'),
            }],
            output='screen'
        ),
        
        # Teleop driver node - converts joy messages to driver inputs
        Node(
            package='simtires_teleop',
            executable='teleop_driver',
            name='teleop_driver',
            namespace=LaunchConfiguration('vehicle_namespace'),
            parameters=[{
                'topic_name': LaunchConfiguration('topic_name'),
            }],
            remappings=[
                # Remap joy topic if using vehicle namespace
                ('joy', [LaunchConfiguration('vehicle_namespace'), 
                        '/joy'] if LaunchConfiguration('vehicle_namespace') else 'joy'),
            ],
            output='screen'
        ),
    ])
