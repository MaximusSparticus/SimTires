#!/usr/bin/env python3
# Basic launch file for SimTires simulation

# TODO(zmd): this needs to be tested / refined

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    # Get package directory
    pkg_dir = get_package_share_directory('simtires')
    
    # Get launch arguments
    world_config = LaunchConfiguration('world_config').perform(context)
    vehicle_config = LaunchConfiguration('vehicle_config').perform(context)
    enable_viz = LaunchConfiguration('enable_visualization').perform(context)
    num_vehicles = int(LaunchConfiguration('num_vehicles').perform(context))
    
    nodes = []
    
    # Main simulation node
    simulation_node = Node(
        package='simtires',
        executable='simulation_node',
        name='simtires_simulation',
        parameters=[{
            'world_config': world_config,
            'vehicle_config': vehicle_config,
            'step_size': 0.003,
            'real_time_factor': 1.0,
            'enable_visualization': enable_viz.lower() == 'true'
        }],
        output='screen'
    )
    nodes.append(simulation_node)
    
    # Launch autonomous driver nodes for each vehicle
    for i in range(num_vehicles):
        if i == 0:
            namespace = ""  # Default vehicle has no namespace
        else:
            namespace = f"vehicle_{i}"
            
        autonomy_node = Node(
            package='simtires_autonomy',
            executable='autonomous_driver_node',
            name='autonomous_driver',
            namespace=namespace,
            parameters=[{
                'vehicle_id': i,
                'max_speed': 5.0,
                'planning_frequency': 10.0
            }],
            output='screen'
        )
        nodes.append(autonomy_node)
    
    return nodes

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'world_config',
            default_value='flat_world',
            description='World configuration to load (flat_world, mountain_world, proving_ground)'
        ),
        DeclareLaunchArgument(
            'vehicle_config', 
            default_value='hmmwv_full_sensors',
            description='Vehicle configuration to load'
        ),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description='Enable Irrlicht visualization'
        ),
        DeclareLaunchArgument(
            'num_vehicles',
            default_value='1',
            description='Number of vehicles to spawn with autonomous drivers'
        ),
        
        # Use OpaqueFunction to access LaunchConfiguration values
        OpaqueFunction(function=launch_setup)
    ])
