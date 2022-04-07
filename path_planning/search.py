import sys
import numpy as np
import matplotlib.pyplot as plt
from implementation3 import *

g = GridWithWeights(30,15)
g.walls = DIAGRAM1_WALLS

#g = SquareGrid(30,15)
#g.walls = DIAGRAM1_WALLS

start = (0,10)
goal  = (27,2)

if len(sys.argv) != 2:
    print('使い方: python3 search.py 1 または 2 または 3')
    sys.exit(1)

method = sys.argv[1]  # BFS:1, Dijkstra:2, A*:3

if method == 1:
    # BFS
    search  = breadth_first_search(g, start, goal)
    title = '幅優先探索'
elif method == 2:
    # Dijkstra
    search, cost_so_far  = dijkstra_search(g, start, goal)
    title = 'ダイクストラ法'
elif method == 3:
    # A*
    search, cost_so_far  = a_star_search(g, start, goal)
    title = 'A*アルゴリズム'
 else:
    print('使い方: python3 search.py 1 または 2 または 3')
    sys.exit(1)


draw_grid2(g, path=reconstruct_path(search, start=start, goal=goal), \
           point_to=search, start=start, goal=goal, title=title)


#draw_grid2(g, point_to=search2, start=start, goal=goal, title=title)
