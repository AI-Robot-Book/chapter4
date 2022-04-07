from implementation2 import *
import numpy as np
import matplotlib.pyplot as plt


g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS

start = (8,7)
goal  = (27,2)
came_from = breadth_first_search(g, start, goal)
draw_grid2(g, path=reconstruct_path(came_from, start=start, goal=goal),
              point_to=came_from, start=start, goal=goal)
