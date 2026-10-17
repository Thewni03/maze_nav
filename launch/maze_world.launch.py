import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('maze_nav')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz = get_package_share_directory('ros_gz_sim')
    world = os.path.join(pkg, 'worlds', 'maze.world')
    tb3_launch = os.path.join(tb3, 'launch')

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s -v2 ', world], 'on_exit_shutdown': 'true'}.items())
    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items())
    robot_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())
    # start in the centre of the top-left cell of the maze
    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_launch, 'spawn_turtlebot3.launch.py')),
        launch_arguments={'x_pose': '-1.25', 'y_pose': '1.25'}.items())

    return LaunchDescription([gz_server, gz_gui, robot_state, spawn])
