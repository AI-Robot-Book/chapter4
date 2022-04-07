from implementation3 import *
import numpy as np
import matplotlib.pyplot as plt


g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS

start = (0,10)
goal  = (27,2)
came_from = breadth_first_search(g, start, goal)
title = "Breadth-First Search"   
draw_grid2(g, path=reconstruct_path(came_from, start=start, goal=goal), point_to=came_from, start=start, goal=goal, title=title)
