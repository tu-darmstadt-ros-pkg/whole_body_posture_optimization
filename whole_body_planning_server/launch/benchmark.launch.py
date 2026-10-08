"""The whole-body planning server as a method of the Gazebo locomotion benchmark (methods/wbpo.yaml)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('whole_body_planning_server'), 'config', 'benchmark')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'athena.yaml'),
                              description='server and optimizer parameters'),
        DeclareLaunchArgument('overrides', default_value=os.path.join(config, 'empty.yaml'),
                              description='parameter file applied last, for tuning variants'),
        Node(package='whole_body_planning_server', executable='benchmark_bridge.py', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='whole_body_posture_optimization', executable='srdf_with_world_publisher.py', output='screen',
             parameters=[{'use_sim_time': True, 'srdf_file': os.path.join(config, 'athena.srdf')}]),
        Node(package='whole_body_planning_server', executable='whole_body_planning_server_node', output='screen',
             parameters=[params_file, LaunchConfiguration('overrides'), {
                 'collision_links_config_file': os.path.join(config, 'athena_shape_config.yaml'),
             }],
             remappings=[('esdf_map_in', '/maps/voxblox/esdf')]),
    ])
