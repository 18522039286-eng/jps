import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    Shutdown,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('jps_planner')
    map_yaml = LaunchConfiguration('map')
    rviz_cfg = os.path.expanduser('~/jps_planner/jps.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/home/baist/Desktop/mymap.yaml'),

        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': map_yaml}]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_map_base', output='screen',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']),

        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[os.path.join(share, 'config', 'costmap.yaml')]),

        TimerAction(period=3.0, actions=[
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager', output='screen',
                 parameters=[{'autostart': True,
                              'node_names': ['map_server', 'planner_server'],
                              'bond_timeout': 0.0}])]),

        TimerAction(period=8.0, actions=[
            Node(package='jps_planner', executable='jps_planner', name='jps_planner',
                 output='screen',
                 parameters=[os.path.join(share, 'config', 'costmap.yaml')])]),

        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_cfg] if os.path.exists(rviz_cfg) else ['rviz2'],
            output='screen', on_exit=Shutdown()),
    ])