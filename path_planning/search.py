import sys
import numpy as np
import matplotlib.pyplot as plt
from implementation2 import *

def main():
  

    start = (8,7)
    goal  = (17,2)

    if len(sys.argv) == 1:
        print('使い方: python3 search bfs または dijkstra または astar')
        sys.exit(1)

    if sys.argv[1] == 'bfs':
        g = SquareGrid(30,15)
        g.walls = DIAGRAM1_WALLS
        parents = breadth_first_search(g, start, goal)
    elif sys.argv[1] == 'dijkstra':
        g = GridWithWeights(30,15)
        g.walls = DIAGRAM1_WALLS
        parents = dijkstra_search(g, start, goal)       
    elif sys.argv[1] == 'astar':
        g = GridWithWeights(30,15)
        g.walls = DIAGRAM1_WALLS
        parents = a_star_search(g, start, goal)
    else:
        print('使い方: python3 search bfs または dijkstra または astar')

    draw_grid2(g, point_to=parents, start=start, goal=goal)


if __name__ == '__main__':
    main()
