"""Whole-body posture optimization: the planning server and its SRDF publisher, on Athena by default."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    arg = {name: LaunchConfiguration(name).perform(context)
           for name in ('params_file', 'overlays', 'srdf_file', 'shape_config', 'esdf_topic')}
    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(package='whole_body_posture_optimization', executable='srdf_with_world_publisher.py', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'srdf_file': arg['srdf_file']}]),
        Node(package='whole_body_planning_server', executable='whole_body_planning_server_node', output='screen',
             # Without a node name the dict applies to every node of the process (ESDF, MoveIt).
             parameters=[arg['params_file']] + arg['overlays'].split() + [
                 {'collision_links_config_file': arg['shape_config'], 'use_sim_time': use_sim_time},
             ],
             remappings=[('esdf_map_in', arg['esdf_topic'])]),
    ]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('whole_body_planning_server'), 'config')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'athena.yaml'),
                              description='server and optimizer parameters'),
        DeclareLaunchArgument('overlays', default_value='',
                              description='parameter files applied after params_file, separated by spaces'),
        DeclareLaunchArgument('srdf_file', default_value=os.path.join(config, 'athena.srdf')),
        DeclareLaunchArgument('shape_config', default_value=os.path.join(config, 'athena_shape_config.yaml'),
                              description='collision sampling of the robot'),
        DeclareLaunchArgument('esdf_topic', default_value='esdf_map_in',
                              description='voxblox_msgs/Layer ESDF input'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_nodes),
    ])
