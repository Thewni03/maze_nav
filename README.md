# Autonomous navigation of TurtleBot3 Burger in a maze (ROS 2 Jazzy, Gazebo)
Package `maze_nav` for IE4060 Assignment 02. Maze = Figure 1 (6x6 cells of 0.5 m).

## Build
    cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
    export TURTLEBOT3_MODEL=burger

## Run
    ros2 launch maze_nav maze_world.launch.py          # Gazebo maze + robot
    ros2 launch maze_nav nav2_maze.launch.py           # Nav2 (map_server, AMCL, NavFn planner, RPP controller)
    ros2 run rviz2 rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
    # click "Nav2 Goal" in RViz to send a goal

## Mapping (slam_toolbox)
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true slam_params_file:=$HOME/ros2_ws/src/maze_nav/config/slam_params.yaml
    python3 scripts/route_driver.py --full             # LiDAR-centred tour of all 36 cells

## Other scripts
- `scripts/generate_maze.py` builds `worlds/maze.world`
- `scripts/make_truth_map.py` builds the exact navigation map from the wall list
- `scripts/nav_benchmark.py` Dijkstra vs A* (planning time, path length)
- `scripts/dqn_maze.py` Deep Q-Network on the maze grid (NumPy only)

## Results (planning only, robot at start, 10 repeats)
| Goal | Dijkstra | A* | Grid optimum |
|---|---|---|---|
| G1 bottom-left | 3.36 m, 0.6 ms | 3.36 m, 0.4 ms | 3.5 m |
| G2 bottom-right | 4.89 m, 0.4 ms | 4.91 m, 0.4 ms | 5.0 m |
| G3 top-right | 4.34 m, 0.6 ms | 4.26 m, 0.2 ms | 4.5 m |
