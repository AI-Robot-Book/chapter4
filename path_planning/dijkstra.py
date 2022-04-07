from implementation2 import *
import numpy as np
import matplotlib.pyplot as plt


g = SquareGrid(30,15)
g.walls = DIAGRAM1_WALLS

start = (8,7)
goal  = (17,2)
parents = dijkstra_search(g, start, goal)
draw_grid2(g, point_to=parents, start=start, goal=goal)
