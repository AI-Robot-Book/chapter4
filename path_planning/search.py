from implementation3 import *
import numpy as np
import matplotlib.pyplot as plt


g = GridWithWeights(30,15)
g.walls = DIAGRAM1_WALLS

#g = SquareGrid(30,15)
#g.walls = DIAGRAM1_WALLS

start = (0,10)
goal  = (27,2)

method = 2  # BFS:1, Dijkstra:2, A*:3

if method == 1:
    # BFS
    search  = breadth_first_search(g, start, goal)
    title = 'Breadth-First Search'
elif method == 2:
    # Dijkstra
    search, cost_so_far  = dijkstra_search(g, start, goal)
    title = 'Dijkstra\'s algorithm'
elif method == 3:
    # A*
    search, cost_so_far  = a_star_search(g, start, goal)
    title = 'A* search'


draw_grid2(g, path=reconstruct_path(search, start=start, goal=goal), \
           point_to=search, start=start, goal=goal, title=title)


#draw_grid2(g, point_to=search2, start=start, goal=goal, title=title)
