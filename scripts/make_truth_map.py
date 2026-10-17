#!/usr/bin/env python3
"""Rasterise the maze (same wall list as the Gazebo world) into a Nav2 map.
Map frame = robot spawn pose (top-left cell centre = map (0,0), facing +x), exactly like a SLAM map."""
import os, sys
from PIL import Image, ImageDraw
sys.path.insert(0, os.path.expanduser('~/ros2_ws/src/maze_nav/scripts'))
from generate_maze import WALLS, N, CELL, T

RES, MARGIN = 0.05, 0.2
SX, SY = 1.25, -1.25                      # world -> map shift (spawn pose is (-1.25, 1.25) in the world)
xmin, xmax = -N*CELL/2 + SX - MARGIN, N*CELL/2 + SX + MARGIN
ymin, ymax = -N*CELL/2 + SY - MARGIN, N*CELL/2 + SY + MARGIN
W, H = round((xmax - xmin) / RES), round((ymax - ymin) / RES)
img = Image.new('L', (W, H), 205)         # 205 = unknown
d = ImageDraw.Draw(img)
def px(x, y): return ((x - xmin) / RES, (ymax - y) / RES)
# free space inside the maze
(a, b), (c, e) = px(-N*CELL/2 + SX, N*CELL/2 + SY), px(N*CELL/2 + SX, -N*CELL/2 + SY)
d.rectangle([a, b, c, e], fill=254)
for x0, y0, x1, y1 in WALLS:              # grid lines -> map metres (x right, y up) -> 1-pixel (5 cm) wall lines
    X0, Y0 = -N*CELL/2 + SX + x0*CELL, N*CELL/2 + SY - y0*CELL
    X1, Y1 = -N*CELL/2 + SX + x1*CELL, N*CELL/2 + SY - y1*CELL
    (p0, q0), (p1, q1) = px(X0, Y0), px(X1, Y1)
    d.rectangle([round(min(p0, p1)), round(min(q0, q1)), round(max(p0, p1)), round(max(q0, q1))], fill=0)
out = os.path.expanduser('~/ros2_ws/src/maze_nav/maps/maze_map_truth')
img.save(out + '.pgm')
open(out + '.yaml', 'w').write(f"image: maze_map_truth.pgm\nmode: trinary\nresolution: {RES}\norigin: [{xmin:.3f}, {ymin:.3f}, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
img.resize((W*8, H*8), Image.NEAREST).save('/tmp/maze_map_truth_preview.png')
print('wrote', out + '.pgm', f'{W}x{H}', 'origin', (round(xmin, 3), round(ymin, 3)))
