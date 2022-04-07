from implementation import *
import numpy as np
import matplotlib.pyplot as plt


g = GridWithWeights(30,15)
g.walls = DIAGRAM1_WALLS

#g = SquareGrid(30,15)
#g.walls = DIAGRAM1_WALLS

start = (0,10)
goal  = (27,2)
search  = breadth_first_search(g, start, goal)
search2, cost_so_far = dijkstra_search(g, start, goal)
title = "Dijkstra's algorithm"   
#draw_grid2(g, path=reconstruct_path(search, start=start, goal=goal), point_to=search, start=start, goal=goal, title=title)
draw_grid(g, point_to=search2, start=start, goal=goal, title=title)
