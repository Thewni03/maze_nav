#!/usr/bin/env python3
"""Generate the Gazebo (gz sim) maze world from Figure 1 of the assignment.

Grid: 6x6 cells, 0.5 m each (3 m x 3 m). Grid coords: x -> right, y -> down,
origin at the top-left corner of the maze. World frame: maze centred on (0,0).
"""
import os, sys

CELL, N, T, H = 0.5, 6, 0.05, 0.5     # cell size, cells, wall thickness, wall height

# Interior walls read from Figure 1 -> (x0, y0, x1, y1) in grid-line units
INTERIOR = [
    # vertical
    (2, 0, 2, 1), (4, 0, 4, 1), (1, 1, 1, 4), (3, 1, 3, 2),
    (5, 1, 5, 2), (2, 2, 2, 3), (3, 3, 3, 4), (5, 3, 5, 5),
    # horizontal
    (2, 2, 5, 2), (4, 3, 6, 3), (1, 4, 4, 4), (0, 5, 1, 5), (2, 5, 5, 5),
]
BORDER = [(0, 0, N, 0), (0, N, N, N), (0, 0, 0, N), (N, 0, N, N)]
WALLS = BORDER + INTERIOR

def to_world(gx, gy):
    return (-N * CELL / 2 + gx * CELL, N * CELL / 2 - gy * CELL)

def box(i, x0, y0, x1, y1):
    (wx0, wy0), (wx1, wy1) = to_world(x0, y0), to_world(x1, y1)
    cx, cy = (wx0 + wx1) / 2, (wy0 + wy1) / 2
    sx, sy = abs(wx1 - wx0) + T, abs(wy1 - wy0) + T      # +T closes corners
    return f"""
      <collision name="c{i}"><pose>{cx:.3f} {cy:.3f} {H/2} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {H}</size></box></geometry></collision>
      <visual name="v{i}"><pose>{cx:.3f} {cy:.3f} {H/2} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {H}</size></box></geometry>
        <material><ambient>0.8 0.3 0.2 1</ambient><diffuse>0.8 0.3 0.2 1</diffuse></material></visual>"""

def world_sdf():
    walls = "".join(box(i, *w) for i, w in enumerate(WALLS))
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="maze_world">
    <physics name="2ms" type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <gravity>0 0 -9.8</gravity>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.8 0.8 0.8 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
        <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.85 0.85 0.85 1</diffuse></material></visual>
    </link></model>
    <model name="maze"><static>true</static><link name="walls">{walls}
    </link></model>
  </world>
</sdf>
"""

def preview(path):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(5, 5))
    for k in range(N + 1):
        ax.plot([0, N], [k, k], c="0.8", lw=.8); ax.plot([k, k], [0, N], c="0.8", lw=.8)
    for x0, y0, x1, y1 in WALLS:
        ax.plot([x0, x1], [y0, y1], c="k", lw=3, solid_capstyle="round")
    ax.set_xlim(-.2, N + .2); ax.set_ylim(N + .2, -.2); ax.set_aspect("equal"); ax.axis("off")
    ax.set_title("Generated maze (check against Figure 1)")
    fig.savefig(path, dpi=100, bbox_inches="tight")

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "worlds", "maze.world")
    open(out, "w").write(world_sdf()); print("wrote", out)
    if "--preview" in sys.argv: preview(os.path.splitext(out)[0] + "_preview.png")
