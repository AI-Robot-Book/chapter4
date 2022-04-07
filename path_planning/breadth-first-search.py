from implementation2 import *
import numpy as np
import matplotlib.pyplot as plt


g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS

start = (8,7)
goal  = (27,2)
parents = breadth_first_search(g, start, goal)
title = "Breadth-First Search"
draw_grid2(g, title=title, point_to=parents, start=start, goal=goal)
