import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('maze_nav')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_yaml = os.path.join(pkg, 'maps', 'maze_map.yaml')
    to_nav = [('cmd_vel', 'cmd_vel_nav')]   # Nav2 output -> converter -> /cmd_vel (TwistStamped)

    def lifecycle(name, nodes):
        return Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name=name, output='screen',
                    parameters=[{'use_sim_time': True, 'autostart': True,
                                 'node_names': nodes, 'bond_timeout': 0.0}])

    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[params, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[params]),
        lifecycle('lifecycle_manager_localization', ['map_server', 'amcl']),

        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[params], remappings=to_nav),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[params]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[params], remappings=to_nav),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[params]),
        lifecycle('lifecycle_manager_navigation',
                  ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']),

        Node(package='maze_nav', executable='twist_stamper.py', name='twist_stamper', output='screen'),
    ])
