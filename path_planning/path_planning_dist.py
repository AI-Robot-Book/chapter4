# A sample code of path planning by Kosei Demura
# Copyright 2022 Kosei Demura
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
# 
# This sample code is developed based on the sample code as following.
# --------------------------
# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
# --------------------------
# 
# The main differences are 
# 1. Grafical display of the grid map using the Matplotib instead of the text-based display
# 2. Queue module is used from Python standard library
# 3. Simplification of the sample code for beginner
# 4. Change the variavles name: forntier -> open_cells,   visited -> closed_cells, ...


from __future__ import annotations
# some of these types are deprecated: https://www.python.org/dev/peps/pep-0585/
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional

import numpy as np
import matplotlib.pyplot as plt
import math, queue, sys, time
from mpl_toolkits.axes_grid1 import make_axes_locatable


GridLocation = Tuple[int, int]
Location = TypeVar('Location')


class Graph(Protocol):
    def neighbors(self, id: Location) :
        pass


class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls: List[GridLocation] = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        # Visitting order of neighbors that change the pass
        neighbors = [(x+1, y), (x, y+1), (x-1, y), (x, y-1)] # E N W S
        # neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S        
        # neighbors = [ (x, y-1), (x, y+1), (x-1, y), (x+1, y),] # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results       
    

class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}
        self.width = width
        self.height = height
    
    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)


#class SquareGridNeighborOrder(SquareGrid):
#    def neighbors(self, id):
#        (x, y) = id
#        neighbors = [(x + dx, y + dy) for (dx, dy) in self.NEIGHBOR_ORDER]
#        results = filter(self.in_bounds, neighbors)
#        results = filter(self.passable, results)
#        return list(results)


#class GridWithAdjustedWeights(GridWithWeights):
#    def cost(self, from_node, to_node):
#        prev_cost = super().cost(from_node, to_node)
#        nudge = 0
#        (x1, y1) = from_node
#        (x2, y2) = to_node
#        if (x1 + y1) % 2 == 0 and x2 != x1: nudge = 1
#        if (x1 + y1) % 2 == 1 and y2 != y1: nudge = 1
#        return prev_cost + 0.001 * nudge


class PathPlanning:
    def __init__(self, world_no, method):       
        self.world_no = world_no  # world number
        self.make_grid_world()    # make a grid world

        if method == 1:  # BFS
            self.title = '幅優先探索'
            breadth_first_search(self.graph, self.start, self.goal, self.title)
        elif method == 2:  # Dijkstra
            self.title = 'ダイクストラ法'
            dijkstra_search(self.graph, self.start, self.goal, self.title) 
        elif method == 3:  # A*
            self.title = 'A*アルゴリズム'
            a_star_search(self.graph, self.start, self.goal, self.title)

    def make_grid_world(self):
        if self.world_no == 1:
            self.width = 30        # width of the grid world
            self.height = 15        # height of the grid world   
            self.start = (0, 10)
            self.goal = (27, 2)            
            # Walls1 (30x15 grid world)
            self.DIAGRAM_WALLS = [(21, 0), (22, 0), (21, 1), (22, 1), (21, 2), (22, 2), (3, 3), (4, 3), (21, 3),\
                (22, 3), (3, 4), (4, 4), (13, 4), (14, 4), (21, 4), (22, 4), (3, 5), (4, 5), (13, 5), (14, 5), \
                (21, 5), (22, 5), (23, 5), (24, 5), (25, 5), (3, 6), (4, 6), (13, 6), (14, 6), (21, 6), (22, 6),\
                (23, 6), (24, 6), (25, 6), (3, 7), (4, 7), (13, 7), (14, 7), (3, 8), (4, 8), (13, 8), (14, 8), \
                (3, 9), (4, 9), (13, 9), (14, 9), (3, 10), (4, 10), (13, 10), (14, 10), (3, 11), (4, 11), (13, 11), \
                (14, 11), (13, 12), (14, 12), (13, 13), (14, 13), (13, 14), (14, 14)]
            print('DIAGRAM_WALL1=', self.DIAGRAM_WALLS)
        elif self.world_no == 2:
            self.width = 70        # width of the grid world
            self.height = 70        # height of the grid world    
            self.start = (20, 20)    # start point
            self.goal = (60, 60)    # goal point        
            # Walls2 (70x70 grid world)
            self.DIAGRAM_WALLS = []         
            for i in range(self.width):
                for j in range(self.height):
                    if ((i == 0 or j == 0) or (i == self.width-1 or j == self.height-1) or 
                        (i == 29 and j < 50) or (i == 49 and j >= 30)):
                        self.DIAGRAM_WALLS.append((i, j))
            print('DIAGRAM_WALL2=', self.DIAGRAM_WALLS)        
        elif self.world_no == 3:
            self.width = 10        # width of the grid world
            self.height = 10       # height of the grid world    
            self.start = (0, 0)    # start point
            self.goal = (9, 9)     # goal point        
            # Walls2 (70x70 grid world)
            self.DIAGRAM_WALLS = []         
            for i in range(self.width):
                for j in range(self.height):
                    if ((i == 3 and j < 6) or (i == 6 and j >= 3)):
                        self.DIAGRAM_WALLS.append((i, j))
            print('DIAGRAM_WALL3=', self.DIAGRAM_WALLS)               
        elif self.world_no == 4:
            self.width = 8       # width of the grid world
            self.height = 8      # height of the grid world    
            self.start = (0, 0)  # start point
            self.goal = (7, 7)   # goal point        
            # Walls2 (70x70 grid world)
            self.DIAGRAM_WALLS = [(0, 4), (1, 2), (1, 6), (1, 7), (2, 4), (3, 1), (3, 3), (3 ,4),\
                (3 ,5), (3, 6), (4, 3), (5, 1), (5, 3), (5, 5), (6, 5), (6, 7), (7, 2), (7, 3), (7, 4)]
            print('DIAGRAM_WALL4=', self.DIAGRAM_WALLS)  
        else:
            print("Error: wrong world number")
        
        self.graph = GridWithWeights(self.width, self.height)  # demu
        #self.graph = SquareGrid(self.width, self.height)
        self.graph.walls = self.DIAGRAM_WALLS      


class DrawMap():
    def __init__(self, graph, start, goal, title):
        self.graph = graph
        self.start = start
        self.goal = goal
        self.title = title
        self.path_length = 0
        # self.steps = 0
 
        self.map = np.random.randint(0, 1, size=(self.graph.height, self.graph.width))
        #self.map = np.zeros((self.graph.height, self.graph.width))
        self.map[0, 0] = 0 
        self.map[self.graph.height-1, self.graph.width-1] = 0  # map[y, x]

        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.subplots_adjust(bottom=0.15)
        self.ax.text(start[0],start[1], 'S', va='center', ha='center')
  
    def set_value(self, id, current):
        tile = self.draw_tile(id)
        flag = True
        bad_value = 1000000
        if tile == 'S':  # スタート
            print('start')
            self.map[id[1], id[0]] = bad_value
        elif tile == 'G':  # ゴール
            print('goal')
            self.map[id[1], id[0]] = bad_value # 200
        elif tile == '#':  # obstacles
            self.map[id[1], id[0]] = -100 # 1000 # 200
        else:
            # flag = False  # comment out demu
            # self.steps += 1
            if self.map[id[1],id[0]] == 0: # not visited, not in closed cells
                self.map[id[1],id[0]] = self.map[current[1],current[0]] + 1
                tile = str(self.map[id[1],id[0]])  
                  
        if flag:
            self.ax.text(id[0],id[1], tile, va='center', ha='center')
              
    def draw_tile(self, id): 
        #r = r"$\cdot$"
     
        # if 'point_to' in style and style['point_to'].get(id, None) is not None  \
        #    or  'path' in style and id in style['path'] and style['point_to'].get(id, None) is not None :                
        if id == self.start:
            r = 'S'
        elif id == self.goal:
            r = 'G'
        elif id in self.graph.walls:
            r = '#'
        else:            
            r = str(self.map[id[1],id[0]])      
        return r 

    def draw_obstacles_path(self, path):
        #for id1 in self.DIAGRAM_WALLS:
        for id1 in self.graph.walls: 
            r1 = '#'
            self.ax.text(id1[0],id1[1], r1, va='center', ha='center')
        
        self.path_length = len(path)
        self.title += ' (' + str(self.path_length-1) + ' steps)'
        self.ax.set_title(self.title, pad = 30, fontname='TakaoGothic')       
        
        for i in range(len(path)):
            if path[i] != self.start and path[i] != self.goal: 
                (x1, y1) = path[i]
                (x2, y2) = path[i+1]
                if x2 == x1 + 1: r3 = r"$\rightarrow$"
                if x2 == x1 - 1: r3 = r"$\leftarrow$"
                if y2 == y1 + 1: r3 = r"$\uparrow$"
                if y2 == y1 - 1: r3 = r"$\downarrow$"
                # self.ax.text(x1, y1, r3, va='center', ha='center') # comment out demu

    def paint_tile(self,path):
        bad_value = 1000000
        
        for id in path: # paint yellow
            if id != self.start and id != self.goal:
                self.map[id[1], id[0]] = bad_value
        
        self.map[self.start] = 1001
        self.map[self.goal] = 1001

        

        #for id2 in self.DIAGRAM_WALLS: # paint red
        for id2 in self.graph.walls: # paint black
            self.map[id2[1], id2[0]] = -100                   
     
        self.masked_map = np.ma.masked_where(self.map==bad_value, self.map)
        self.cmap = plt.cm.YlGn # Wistia # Blues
        self.cmap.set_under('black')   # 0より値が小さい格子は黄色
        self.cmap.set_over('pink')   # vmaxより値が大きい格子は赤色
        self.cmap.set_bad('cyan')  # bad_valueの格子はcyan
        
    def show(self, path):
        self.draw_obstacles_path(path)
        self.paint_tile(path)
        im = self.ax.imshow(self.masked_map, cmap=self.cmap, vmin=-1, vmax=1000)
        self.ax.invert_yaxis()         
        
        #divider = make_axes_locatable(self.ax)
        #cax = divider.append_axes("right", size="5%", pad=0.1)
        #self.fig.colorbar(im, label='steps',cax=cax)
        #plt.pause(0.01)
        #plt.grid(True)
        x = -0.5
        y = 0
        for i in range(self.graph.width-1):
            x += 1
            self.ax.axvline(x, linewidth=1, color='black')
        for y in range(self.graph.height-1): 
            y += 1   
            self.ax.axhline(y-0.5, linewidth=1, color='black')
        self.ax.tick_params(length=0)
        self.ax.get_xaxis().set_tick_params(pad=12)
        self.ax.get_yaxis().set_tick_params(pad=12)
        plt.show()


# thanks to @m1sp <Jaiden Mispy> for this simpler version of
# reconstruct_path that doesn't have duplicate entries
def reconstruct_path(came_from: Dict[Location, Location],
                     start: Location, goal: Location) -> List[Location]:
    current: Location = goal
    path: List[Location] = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path


def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)


def breadth_first_search(graph, start, goal, title):
    draw_map = DrawMap(graph, start, goal, title)

    open_cells = queue.Queue() 
    open_cells.put(start)
    closed_cells = dict()
    closed_cells[start] = None

    while not open_cells.empty():
        current = open_cells.get()        
        if current == goal:
            break        
        for next in graph.neighbors(current):
            if next not in closed_cells: 
                closed_cells[next] = current
                open_cells.put(next) 
                draw_map.set_value(next, current)  # demu 
                               
    path = reconstruct_path(came_from=closed_cells, start=start, goal=goal) 
    draw_map.show(path)
    
    return closed_cells


def dijkstra_search(graph, start, goal, title):
    draw_map = DrawMap(graph, start, goal, title)
 
    open_cells = queue.PriorityQueue()
    open_cells.put(start, 0)
    closed_cells = dict()
    closed_cells[start] = None
    cost_so_far = dict()
    cost_so_far[start] = 0
    
    while not open_cells.empty():
        current = open_cells.get()        
        if current == goal:
            break        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                open_cells.put(next, priority)
                closed_cells[next] = current
                draw_map.set_value(next, current)
    
    path=reconstruct_path(came_from=closed_cells, start=start, goal=goal)    
    draw_map.show(path)
    return closed_cells, cost_so_far


def a_star_search(graph, start, goal, title):
    draw_map = DrawMap(graph, start, goal, title)
  
    open_cells = queue.PriorityQueue()
    open_cells.put(start, 0)
    closed_cells = dict()
    closed_cells[start] = None
    cost_so_far = dict()
    cost_so_far[start] = 0
    
    while not open_cells.empty():
        current: Location = open_cells.get()        
        if current == goal:
            break        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                open_cells.put(next, priority)
                closed_cells[next] = current    
                draw_map.set_value(next, current)
    
    path=reconstruct_path(came_from=closed_cells, start=start, goal=goal)    
    draw_map.show(path)    
    return closed_cells, cost_so_far


def main():  
    print("*** path_planning.py ***")

    if len(sys.argv) != 2:
        print('使い方: python3 search.py 1 または 2 または 3')
        print('        1: Breadth-First, 2: Dijkstra,  3. A* search')
        sys.exit(1)
    else:
        method = int(sys.argv[1])  

    world_no = 4
    path_planning = PathPlanning(world_no, method) 


if __name__ == "__main__":
    main()
