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
# 5. 2022-5-1: 4 diretional moves -> 8 directional moves


from __future__ import annotations
# some of these types are deprecated: https://www.python.org/dev/peps/pep-0585/
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional
import random
import numpy as np
import seaborn as sns
import matplotlib.pyplot as plt
import math, sys, time
from queue import PriorityQueue, Queue
from enum import Enum, auto

from mpl_toolkits.axes_grid1 import make_axes_locatable
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.colors import LinearSegmentedColormap

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
        self.desert: List[GridLocation] = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        # Visitting order of neighbors that change the pass
        neighbors = [(x+1, y), (x, y+1), (x-1, y), (x, y-1),  # demu 8 directions
                    (x-1, y-1), (x-1, y+1), (x+1, y-1), (x+1, y+1)]  # 

        #neighbors = [(x+1, y), (x+1, y+1), (x, y+1), (x-1, y+1), 
        #             (x-1, y), (x-1, y-1), (x, y-1), (x+1, y-1)]  # 
        # demu counter clockwise
        #neighbors = [(x+1, y), (x+1, y+1) ,(x, y+1), (x-1, y+1),  \
        #             (x-1, y), (x-1, y-1), (x, y-1), (x+1, y-1)] 
        
        #neighbors = [(x+1, y), (x, y+1), (x-1, y), (x, y-1)] # ENWS
        # neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # EWNS        
        # neighbors = [ (x, y-1), (x, y+1), (x-1, y), (x+1, y),] # SNWE
        
        results = filter(self.in_bounds, neighbors)  # select neighbors in condition
        results = filter(self.passable, results)     # select neighbors in condition
        return results       
    

class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}
        self.width = width
        self.height = height
    
    def cost(self, from_node, to_node):
        (x1, y1) = from_node
        (x2, y2) = to_node
        diff_x = abs(x2 - x1)
        diff_y = abs(y2 - y1)
        if diff_x + diff_y == 2:
            return self.weights.get(to_node, math.sqrt(2))
        else:
            return self.weights.get(to_node, 1.0)
 


#class SquareGridNeighborOrder(SquareGrid):
#    def neighbors(self, id):
#        (x, y) = id
#        neighbors = [(x + dx, y + dy) for (dx, dy) in self.NEIGHBOR_ORDER]
#        results = filter(self.in_bounds, neighbors)
#        results = filter(self.passable, results)
#        return list(results)


class PathPlanning:
    def __init__(self, world_no, method):       
        self.world_no = world_no  # world number
        self.make_grid_world()    # make a grid world
        self.WEIGHTS: Dict[GridLocation, float] = {}

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
            self.DIAGRAM_DESERT = []
            self.WEIGHTS = {loc: 1 for loc in self.DIAGRAM_DESERT}
            # print('DIAGRAM_WALL1=', self.DIAGRAM_WALLS)
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
            self.DIAGRAM_DESERT = []
            self.WEIGHTS = {loc: 1 for loc in self.DIAGRAM_DESERT}
            # print('DIAGRAM_WALL2=', self.DIAGRAM_WALLS)        
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
            self.DIAGRAM_DESERT = []
            self.WEIGHTS = {loc: 1 for loc in self.DIAGRAM_DESERT}
            # print('DIAGRAM_WALL3=', self.DIAGRAM_WALLS)               
        elif self.world_no == 4:
            self.width = 8       # width of the grid world
            self.height = 8      # height of the grid world    
            self.start = (0, 0)  # start point
            self.goal = (7, 7)   # goal point        
            # Walls2 (70x70 grid world)
            self.DIAGRAM_WALLS = [(0, 4), (1, 2), (1, 6), (1, 7), (2, 4), (3, 1), (3, 3), (3 ,4),\
                (3 ,5), (3, 6), (4, 3), (5, 1), (5, 3), (5, 5), (6, 5), (6, 7), (7, 2), (7, 3), (7, 4)]
            self.DIAGRAM_DESERT = []
            self.WEIGHTS = {loc: 1 for loc in self.DIAGRAM_DESERT}
            # print('DIAGRAM_WALL4=', self.DIAGRAM_WALLS) 
        elif self.world_no == 5:  # same world change later
            self.width  = 10       # width of the grid world
            self.height = 10      # height of the grid world    
            self.start = (0, 4)  # start point
            self.goal = (9, 4)   # goal point        
            self.DIAGRAM_WALLS = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
            self.DIAGRAM_DESERT =  [(3, 4), (3, 5), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
                                    (4, 7), (4, 8), (5, 1), (5, 2), (5, 3), (5, 4), (5, 5), (5, 6),
                                    (5, 7), (5, 8), (6, 2), (6, 3), (6, 4), (6, 5), (6, 6), (6, 7),
                                    (7, 3), (7, 4), (7, 5)]
            self.WEIGHTS = {loc: 5 for loc in self.DIAGRAM_DESERT}
            print('WEIGHTS=', self.WEIGHTS) 
        elif self.world_no == 6:
            self.width = 42        # width of the grid world
            self.height = 42        # height of the grid world    
            self.start = (5, 18)    # start point
            self.goal = (35, 25)    # goal point        
            # Walls2 (70x70 grid world)
            self.DIAGRAM_WALLS = []         
            for i in range(self.width):
                for j in range(self.height):
                    if ((i == 0 or j == 0) or (i == self.width-1 or j == self.height-1)):
                        self.DIAGRAM_WALLS.append((i, j))
            self.DIAGRAM_DESERT = []
            self.WEIGHTS = {loc: 1 for loc in self.DIAGRAM_DESERT}
        elif self.world_no == 7:
            self.width = 42        # width of the grid world
            self.height = 42        # height of the grid world    
            self.start = (5, 18)    # start point
            self.goal = (26, 24)    # goal point        
            self.DIAGRAM_WALLS = []         
            for i in range(self.width):
                for j in range(self.height):
                    if ((i == 0 or j == 0) or (i == self.width-1 or j == self.height-1) or
                        ((j == 15) and (15 <= i and i <= 35)) or
                        ((j == 25) and (15 <= i and i <= 25)) or
                        ((j == 28) and (25 <= i and i <= 30)) or
                        ((j == 36) and (18 <= i and i <= 25)) or
                        ((i == 25) and (15 <= j and j <= 36))):
                            self.DIAGRAM_WALLS.append((i, j))
            self.DIAGRAM_DESERT = []
            self.WEIGHTS = {loc: 1 for loc in self.DIAGRAM_DESERT}
        elif self.world_no == 8:
            self.width = 42        # width of the grid world
            self.height = 42        # height of the grid world    
            self.start = (5, 18)    # start point
            self.goal = (26, 24)    # goal point        
            self.DIAGRAM_WALLS = [] 
            thresh = 0.7            
            seed = 8
            random.seed(seed)      
            for i in range(self.width):
                for j in range(self.height):
                    if (i == 0 or j == 0 or
                        i == self.width-1 or j == self.height-1):
                            self.DIAGRAM_WALLS.append((i, j))
                    elif (random.random() > thresh):
                        self.DIAGRAM_WALLS.append((i, j))
            self.DIAGRAM_DESERT = []
            self.WEIGHTS = {loc: 1 for loc in self.DIAGRAM_DESERT}
        else:
            print("Error: wrong world number")
        
        self.graph = GridWithWeights(self.width, self.height)  # demu
        #self.graph = SquareGrid(self.width, self.height)
        self.graph.walls = self.DIAGRAM_WALLS  
        self.graph.desert = self.DIAGRAM_DESERT   
        self.graph.weights = self.WEIGHTS 

class Color(Enum):
    GRAY = auto()         # 1
    LIGHTYELLOW = auto()  # 2
    BLACK = auto()        # 3
    CHOCOLATE = auto()    # 4
    CYAN = auto()         # 5
    GREEN = auto()        # 6

class DrawMap():
    def __init__(self, graph, start, goal, title):
        self.graph = graph
        self.start = start
        self.goal = goal
        self.title = title
        self.path_length = 0
        # 0:white, 1:yellow, 2:brown, 3: blue, 4:green, 5:black
        # CSS colors https://www.w3schools.com/cssref/css_colors.asp
        #self.colors = ['white', 'Gold', 'brown', 'red', 'LimeGreen','FireBrick']
        #self.colors = ['LightSteelBlue', 'DeepSkyBlue', 'brown', 'red', 'LemonChiffon','FireBrick']
        #self.colors = ['LightGrey', 'DeepSkyBlue', 'brown', 'Red', 'LightYellow','Crimson']
        #self.colors = ["black", "deepskyblue", "brown", "red", "lightyellow", "crimson"]
        #self.colors = ["white", "blue", "green", "red", "yellow", "brown"]
        #self.colors = ["white", "gray", "black", "brown", "yeColor(1).name,llow"]
        sns.set()
        self.cmap = ListedColormap([Color(1).name, Color(2).name, Color(3).name,
                                    Color(4).name, Color(5).name, Color(6).name])
        self.bounds = [0, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5]
        self.norm   = BoundaryNorm(self.bounds, self.cmap.N)
        
        
        #self.cmap = LinearSegmentedColormap.from_list('mycmap', self.colors)
         
        # self.steps = 0 
        #self.map = np.random.randint(0, 1, size=(self.graph.height, self.graph.width))
        #self.map = np.zeros((self.graph.height, self.graph.width),dtype=int)
        # self.map = np.zeros((self.graph.height, self.graph.width))
        #self.map = np.ones((self.graph.height, self.graph.width))
        self.map = np.full((self.graph.height, self.graph.width), -1)
        #self.map[0, 0] = 0 
        #self.map[self.graph.height-1, self.graph.width-1] = 0  # map[y, x]

        # maptolot
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.subplots_adjust(bottom=0.15) # 0.15
        #self.ax.text(start[0],start[1], 'S', va='center', ha='center')
        #self.ax.text(goal[0],goal[1], 'G', va='center', ha='center')

        # seaborn
        """        
        self.ax = sns.heatmap(self.map, cmap=self.cmap, norm=self.norm, linewidths=.5, 
            linecolor='black', square=True, cbar=False)
        """
        #plt.annotate('S', start)
        #plt.annotate('G', goal)
        
        #plt.annotate('S', (start[0]+0.4,start[1]+0.4))
        #plt.annotate('G', (goal[0]+0.4,goal[1]+0.4))



  
    # set value of tile to show the environment
    def set_value(self, id, current):
        if self.map[id[1],id[0]] == 0: # not visited, not in closed cells
                self.map[id[1],id[0]] = self.map[current[1],current[0]] + 1
                #self.map[id[1],id[0]] = self.map[current[1],current[0]] \
                #    + self.graph.weights.get((id[0],id[1]),1)
        
        """
        tile = self.get_tile(id)
        flag = True 
        if tile == 'S':  # スタート
            #print('start')
            self.map[id[1], id[0]] = 2 # yellow
        elif tile == 'G':  # ゴール
            #print('goal')
            self.map[id[1], id[0]] = 2 # yellow 
        elif tile == '#':  # obstacles
            self.map[id[1], id[0]] = 1 # black 
        elif tile == '@':  # desert
            self.map[id[1], id[0]] = 3 # brown
        else:
            # flag = False  # comment out demu
            # self.steps += 1
            if self.map[id[1],id[0]] == 0: # not visited, not in closed cells
                # self.map[id[1],id[0]] = self.map[current[1],current[0]] + 1
                self.map[id[1],id[0]] = self.map[current[1],current[0]] \
                    + self.graph.weights.get((id[0],id[1]),1)
                #tile = str(self.map[id[1],id[0]])  
            #self.map[id[1], id[0]] = 4 # green
            #tile = '$\cdot$' 
        if flag:
            # self.ax.text(id[0],id[1], tile, va='center', ha='center')
            pass
        """
    
    # Get the propaty of the title     
    def get_tile(self, id): 
        #r = r"$\cdot$"
     
        # if 'point_to' in style and style['point_to'].get(id, None) is not None  \
        #    or  'path' in style and id in style['path'] and style['point_to'].get(id, None) is not None :                
        if id == self.start:
            r = 'S'
        elif id == self.goal:
            r = 'G'
        elif id in self.graph.walls:
            r = '#'
        elif id in self.graph.desert:
            r = '@'
        else:            
            #r = str(self.map[id[1],id[0]]) 
            r = '$\cdot$'
            pass
        return r 

    def draw_obstacles_path(self, path):
        #for id1 in self.DIAGRAM_WALLS:
        #for id in self.graph.walls: 
        #    r1 = '#'
        #    plt.annotate(r1, (id[0]+0.4,id[1]+0.4))
        #    # self.ax.text(id1[0],id1[1], r1, va='center', ha='center')        
        
        for i in range(len(path)):
            if path[i] != self.start and path[i] != self.goal: 
                (x1, y1) = path[i]
                (x2, y2) = path[i+1]
                if x2 == x1 + 1: r3 = r"$\rightarrow$"
                if x2 == x1 - 1: r3 = r"$\leftarrow$"
                if y2 == y1 + 1: r3 = r"$\uparrow$"
                if y2 == y1 - 1: r3 = r"$\downarrow$"
                if x2 == x1 + 1 and y2 == y1 + 1: r3 = r"$\nearrow$"
                if x2 == x1 + 1 and y2 == y1 - 1: r3 = r"$\searrow$"
                if x2 == x1 - 1 and y2 == y1 + 1: r3 = r"$\nwarrow$"
                if x2 == x1 - 1 and y2 == y1 - 1: r3 = r"$\swarrow$"
                #elf.ax.text(x1, y1, r3, va='center', ha='center') # comment out demu
                # plt.annotate(r3, (x1+0.2,y1+0.4))
                #plt.annotate(r3, (x1,y1),va='center', ha='center')
                #plt.text(x1,y1,r3, va='center', ha='center')
                plt.text(x1+0.5,y1+0.5,r3, va='center', ha='center')

    # Paint the tile
    def paint_tile(self,path,closed_cells):     
        # 0:white, 1:yellow, 2:brown, 3: blue, 4:green, 5:black
        #self.colors = ['white', 'yellow', 'brown', 'blue', 'green','black']
        # BLACK = 0
        # DEEPSKYBLUE = 1
        # BROWN = 2
        # RED = 3
        # LIGHTYELLOW = 4
        # CRIMSON = 5
        #print(f'path*={path}')
        #print(f'map={self.map}')
        #self.map[0, 0] = Color.BLACK.value  
        #print(f'closed_cells={closed_cells}')
        
        for id in closed_cells: 
            self.map[id[1], id[0]] = Color.GRAY.value # Color.YELLOW.value 
            # print(f'map[{id[1]}, {id[0]}]={self.map[id[1], id[0]] }') 
        for id in self.graph.walls: 
            self.map[id[1], id[0]] = Color.LIGHTYELLOW.value # Color.GREEN.value
        for id in self.graph.desert: 
            self.map[id[1], id[0]] = Color.BLACK.value # Color.RED.value 
        for id in path: 
            self.map[id[1], id[0]] = Color.CHOCOLATE.value # Color.CYAN.value
      

        """       
        for id in closed_cells: 
            self.map[id[1], id[0]] = -1 
            # print(f'map[{id[1]}, {id[0]}]={self.map[id[1], id[0]] }') 
        for id in self.graph.walls: 
            self.map[id[1], id[0]] = 100
        for id in self.graph.desert: 
            self.map[id[1], id[0]] = Color.RED.value 
        for id in path: 
            self.map[id[1], id[0]] = 999
        """

        #self.cmap.set_under('red')   # 0より値が小さい格子はred
        #self.cmap.set_over('gray')   # vmaxより値が大きい格子はbrwon
        #self.cmap.set_bad('yellow')  # bad_valueの格子は黄色              
        #bad_value = 999
        #self.masked_map = np.ma.masked_where(self.map==bad_value, self.map)
        #colors = ['white','red', 'yellow', 'brown']
        # 0:white, 1:black, 2:yellow, 3: Brown, 4: Green
        #colors = ['white','black', 'yellow', 'brown', 'green']
        #self.cmap = plt.cm.Blues
        #self.cmap = ListedColormap(colors, name='custon')
        
      
    def show(self, start, goal, path, closed_cells):        
                
        self.draw_obstacles_path(path)
        self.paint_tile(path, closed_cells)
        #im = self.ax.imshow(self.masked_map, cmap=self.cmap, vmin=-1)
        #im = self.ax.imshow(self.map, cmap=self.cmap)        
             
        ##plt.pause(0.01)
        self.ax = sns.heatmap(self.map, cmap=self.cmap, norm=self.norm, linewidths=.5, 
            linecolor='black', square=True, cbar=False)
    
        self.ax.axvline(x = self.graph.width, linewidth=2, color='black')
        self.ax.axhline(y =  0, linewidth=2, color='black')
        #sns.set_style('whitegrid')
        self.ax.invert_yaxis()    
        # self.path_length = len(path)
        self.path_length = len(closed_cells)
        self.title += ' (Explored ' + str(self.path_length-1) + ' cells)'
        self.ax.set_title(self.title, pad = 30, fontname='TakaoGothic')     
        self.ax.text(start[0]+0.5,start[1]+0.5, 'S', va='center', ha='center')
        self.ax.text(goal[0]+0.5,goal[1]+0.5, 'G', va='center', ha='center')
        for id in closed_cells: 
            print(f'map={self.map[id[1],id[0]]}')
            self.ax.text(id[0]+0.5,id[1]+0.5, str(self.map[id[1],id[0]]), va='center', ha='center')
            

        #self.ax.text(start[0],start[1], 'S')
        #self.ax.text(goal[0],goal[1], 'G')
        #plt.annotate('S', start)
        #plt.annotate('G', goal)
        print(np.arange(0, self.graph.width, step=5))
        #self.ax.set_xticks(np.arange(0, self.graph.width, step=5), rotation=0)
        #self.ax.set_yticks(np.arange(0, self.graph.height, step=5), rotation=0)
        #plt.tick_params(width=4,length=4)
        plt.grid(True)
        plt.show()

        #print(f'map[start]={self.map[self.start[0],self.start[1]]}')
        #plt.show()
        

    def draw_grid(self,path):
        min_val = 0
        max_val = 10
        # print("graph width=",graph.width,"height=",graph.height)
        # print("title", style['title'])
	
        map = np.random.randint(0, 1, size=(self.graph.height, self.graph.width))
        map[0,0]=0 
        map[self.graph.height-1,self.graph.width-1]=0

        fig, (ax, ax2) = plt.subplots(figsize=(8,8))
        fig.subplots_adjust(bottom=0.15)
        #ax.set_title(style['title'], pad = 30)
        self.ax.set_title(self.title, pad = 30)
        bad_value = 1000
        for j in range(self.graph.height):
            for i in range(self.graph.width):
                c = map[j,i]
                # ax.text(i, j, str(c), va='center', ha='center')
                #tile = str(draw_tile2(self,graph, (i,j), style))
                #tile = str(draw_tile2(self,graph, (i,j), style))
                tile = self.get_tile((i,j))
                if tile ==  "#":   # ��i
                    map[j,i] =  100
                    continue
                #elif tile == "@":  # L�
                #    map[j,i] = bad_value
                #    continue
                #if 'path' in style and id in style['path']:  
                if (i,j) in path:  
                    map[j,i] = bad_value
                    #continue
                elif tile == "S":  # ����
                    map[j,i] = -1
                elif tile == "G":  # ���
                    map[j,i] = 200
                
                ax.text(i,j, tile, va='center', ha='center')

        masked_array = np.ma.masked_where(map==bad_value, map)
        cmap = plt.cm.Blues
        cmap.set_under('yellow')  # 0��$LUD<Po�r
        cmap.set_over('red')      # vmax��$L'MD<Podr
        cmap.set_bad('pink')      # bad_valuen<Po�� 
        self.ax.matshow(masked_array, cmap=plt.cm.Blues, vmin=0, vmax=127)
                    
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

# Modified for 8 dirction move demu
def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    # return abs(x1 - x2) + abs(y1 - y2)
    # return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))
    return math.hypot(x1 - x2, y1 - y2)


def breadth_first_search(graph, start, goal, title):
    draw_map = DrawMap(graph, start, goal, title)
    open_cells = Queue() 
    open_cells.put(start)
    closed_cells = dict()
    closed_cells[start] = None

    while not open_cells.empty():
        current = open_cells.get()        
        if current == goal:
            break        
        for next in graph.neighbors(current):
            if next not in closed_cells:    
                open_cells.put(next) 
                closed_cells[next] = current 
                #draw_map.set_value(next, current)    
                                    
    path = reconstruct_path(came_from=closed_cells, start=start, goal=goal) 
    #print(f'cost={cost_so_far[goal]}')
    #print(f'***path={path}')
    draw_map.show(start, goal, path,closed_cells)  
    return closed_cells


def dijkstra_search(graph, start, goal, title):
    draw_map = DrawMap(graph, start, goal, title) 
    open_cells = PriorityQueue()
    open_cells.put((0, start))  # (priority, item)
    closed_cells = dict()   # dictionary
    closed_cells[start] = None
    cost_so_far = dict()    # dictionary
    cost_so_far[start] = 0
   
    while not open_cells.empty():        
        current = (open_cells.get())[1]  # get() -> (priority, item)
        if current == goal:
            break        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                open_cells.put((priority, next))
                closed_cells[next] = current
                #draw_map.set_value(next, current)        
    path=reconstruct_path(came_from=closed_cells, start=start, goal=goal)  
    #print(f'cost={cost_so_far[goal]}')  
    draw_map.show(start, goal, path,closed_cells)
    return closed_cells, cost_so_far


def a_star_search(graph, start, goal, title):
    draw_map = DrawMap(graph, start, goal, title)
  
    open_cells = PriorityQueue()
    open_cells.put((0, start))
    closed_cells = dict()
    closed_cells[start] = None
    cost_so_far = dict()
    cost_so_far[start] = 0
    
    while not open_cells.empty():
        current = (open_cells.get())[1]         
        if current == goal:
            break        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in closed_cells or new_cost < cost_so_far[next]:  
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                open_cells.put((priority, next))
                closed_cells[next] = current  
                #draw_map.set_value(next, current)
    path=reconstruct_path(came_from=closed_cells, start=start, goal=goal)  
    #print(f'cost={cost_so_far[goal]}') 
    draw_map.show(start, goal, path,closed_cells)    
    return closed_cells, cost_so_far


def main():  
    print("*** path_planning.py ***")

    if len(sys.argv) != 2:
        print('使い方: python3 search.py 1 または 2 または 3')
        print('        1: Breadth-First, 2: Dijkstra,  3. A* search')
        sys.exit(1)
    else:
        method = int(sys.argv[1])  
        if 1 <= method and method <= 3:
            pass
        else:
            print('使い方: python3 search.py 1 または 2 または 3')
            print('        1: Breadth-First, 2: Dijkstra,  3. A* search')
            sys.exit(1)   

    world_no = 5 # 7 5
 
    try:
        path_planning = PathPlanning(world_no, method) 
    except KeyboardInterrupt:
        pass
 

if __name__ == "__main__":
    main()
