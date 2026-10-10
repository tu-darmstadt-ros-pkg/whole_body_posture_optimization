"""The whole-body planning server as a method of the Gazebo locomotion benchmark (methods/wbpo.yaml).

The robot (robot:=, default BENCHMARK_ROBOT) selects its SRDF, shape config and overlays in ROBOTS.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Per robot package: its SRDF, its shape config and its overlays on params_file.
ROBOTS = {
    'athena_description': ('athena.srdf', 'athena_shape_config.yaml', []),
    'ctu_marv_description': ('marv.srdf', 'marv_shape_config.yaml', ['marv.yaml']),
}


def _nodes(context, config):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ROBOTS:
        raise RuntimeError(f'wbpo has no configuration for robot {robot!r}, only {sorted(ROBOTS)}')
    srdf, shape, overlays = ROBOTS[robot]
    return [
        Node(package='whole_body_planning_server', executable='benchmark_bridge.py', output='screen',
             parameters=[{'use_sim_time': True}]),
        Node(package='whole_body_posture_optimization', executable='srdf_with_world_publisher.py', output='screen',
             parameters=[{'use_sim_time': True, 'srdf_file': os.path.join(config, srdf)}]),
        Node(package='whole_body_planning_server', executable='whole_body_planning_server_node', output='screen',
             parameters=[LaunchConfiguration('params_file')] + [os.path.join(config, o) for o in overlays] + [
                 LaunchConfiguration('overrides'),
                 {'collision_links_config_file': os.path.join(config, shape)},
             ],
             remappings=[('esdf_map_in', '/maps/voxblox/esdf')]),
    ]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('whole_body_planning_server'), 'config', 'benchmark')
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value=os.environ.get('BENCHMARK_ROBOT', 'athena_description'),
                              description='robot package, one of ROBOTS'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(config, 'athena.yaml'),
                              description='server and optimizer parameters'),
        DeclareLaunchArgument('overrides', default_value=os.path.join(config, 'empty.yaml'),
                              description='parameter file applied last, for tuning variants'),
        OpaqueFunction(function=_nodes, args=[config]),
    ])
