# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
# Modified by Kosei Demura
# キューをPython標準ライブラリのQueueモジュールで書き換え


from __future__ import annotations
# some of these types are deprecated: https://www.python.org/dev/peps/pep-0585/
from typing import Protocol, Dict, List, Iterator, Tuple, TypeVar, Optional

# 以下のURLから地図を描く部分だけを借用
# https://levelup.gitconnected.com/dijkstras-shortest-path-algorithm-in-a-grid-eb505eb3a290
# from implementation import *
import numpy as np
import matplotlib.pyplot as plt
import queue

#def draw_grid2(graph, point_to, start, goal, min_val=0, max_val=10):
# **style: 複数のキーワード引数を辞書として受け取る
def draw_grid2(graph, **style):
    min_val = 0
    max_val = 10
    # print("graph width=",graph.width,"height=",graph.height)
    # print("title", style['title'])
	
    map = np.random.randint(0, 1, size=(graph.height, graph.width))
    map[0,0]=0 
    map[graph.height-1,graph.width-1]=0

    fig, ax = plt.subplots(figsize=(8,8))
    fig.subplots_adjust(bottom=0.15)
    ax.set_title(style['title'], pad = 30)
    bad_value = 1000
    for j in range(graph.height):
        for i in range(graph.width):
            c = map[j,i]
            # ax.text(i, j, str(c), va='center', ha='center')
            tile = str(draw_tile2(graph, (i,j), style))
            if tile ==  "#":   # 障害物
                map[j,i] =  100
                continue
            #elif tile == "@":  # 経路
            #    map[j,i] = bad_value
            #    continue
            #if 'path' in style and id in style['path']:  
            if 'path' in style and (i,j) in style['path']:  
                map[j,i] = bad_value
                #continue
            elif tile == "S":  # スタート
                map[j,i] = -1
            elif tile == "G":  # ゴール
                map[j,i] = 200
            
            ax.text(i,j, tile, va='center', ha='center')

    masked_array = np.ma.masked_where(map==bad_value, map)
    cmap = plt.cm.Blues
    cmap.set_under('yellow')  # 0より値が小さい格子は黄色
    cmap.set_over('red')      # vmaxより値が大きい格子は赤色
    cmap.set_bad('pink')      # bad_valueの格子はピンク 
    ax.matshow(masked_array, cmap=plt.cm.Blues, vmin=0, vmax=127)

    			
    plt.show()
# 借用終わり

# draw_titleを変更　by demu
def draw_tile2(graph, id, style):
    r = r"$\cdot$"
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None  \
        or  'path' in style and id in style['path'] and style['point_to'].get(id, None) is not None :
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = r"$\rightarrow$"
        if x2 == x1 - 1: r = r"$\leftarrow$"
        if y2 == y1 + 1: r = r"$\downarrow$"
        if y2 == y1 - 1: r = r"$\uparrow$"
        #if x2 == x1 + 1: r = r"$\leftarrow$"  # １個ずれる
        #if x2 == x1 - 1: r = r"$\rightarrow$"
        #if y2 == y1 + 1: r = r"$\uparrow$"
        #if y2 == y1 - 1: r = r"$\downarrow$"
    #if 'path' in style and id in style['path']:   r = "@"
    if 'start' in style and id == style['start']: r = "S"
    if 'goal' in style and id == style['goal']:   r = "G"
    if id in graph.walls: r = "#"
    return r











T = TypeVar('T')

Location = TypeVar('Location')
class Graph(Protocol):
    # def neighbors(self, id: Location) -> List[Location]: pass
    def neighbors(self, id: Location) : pass

class SimpleGraph:
    def __init__(self):
        self.edges: Dict[Location, List[Location]] = {}
    
    def neighbors(self, id: Location) -> List[Location]:
        return self.edges[id]

example_graph = SimpleGraph()
example_graph.edges = {
    'A': ['B'],
    'B': ['C'],
    'C': ['B', 'D', 'F'],
    'D': ['C', 'E'],
    'E': ['F'],
    'F': [],
}

import collections

'''
class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, x: T):
        self.elements.append(x)
    
    def get(self) -> T:
        return self.elements.popleft()
'''

# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)

def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = "@"
    if 'start' in style and id == style['start']: r = "S"
    if 'goal' in style and id == style['goal']:   r = "G"
    if id in graph.walls: r = "#"
    return r

def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)

# data from main article
DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]

GridLocation = Tuple[int, int]

class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: List[GridLocation] = []
    
    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls
    
    def neighbors(self, id):
    #def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

class WelsightedGraph(Graph):
    def cost(self, from_id: Location, to_id: Location) -> float: pass

class GridWithWeights(SquareGrid):
    def __init__(self, width: int, height: int):
        super().__init__(width, height)
        self.weights: Dict[GridLocation, float] = {}
    
    def cost(self, from_node: GridLocation, to_node: GridLocation) -> float:
        return self.weights.get(to_node, 1)

diagram4 = GridWithWeights(10, 10)
diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}

import heapq

'''
class PriorityQueue:
    def __init__(self):
        self.elements: List[Tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]
'''

def dijkstra_search(graph, start, goal):
    frontier = queue.PriorityQueue()
    frontier.put(start, 0)
    visited = dict()
    visited[start] = None
    cost_so_far = dict()
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost
                frontier.put(next, priority)
                visited[next] = current
    
    return visited, cost_so_far

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

def a_star_search(graph:WeightedGraph, start, goal):
    frontier = queue.PriorityQueue()
    frontier.put(start, 0)
    visited = dict()
    visited[start] = None
    cost_so_far = dict()
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                visited[next] = current
    
    return visited, cost_so_far

def breadth_first_search(graph, start, goal):
    frontier = queue.Queue() 
    frontier.put(start)
    visited = dict()
    visited[start] = None
    #print("frontier:",frontier.queue)
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            if next not in visited:
                frontier.put(next)
                visited[next] = current

    #print("visited",visited)
    
    return visited

class SquareGridNeighborOrder(SquareGrid):
    def neighbors(self, id):
        (x, y) = id
        neighbors = [(x + dx, y + dy) for (dx, dy) in self.NEIGHBOR_ORDER]
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return list(results)

def test_with_custom_order(neighbor_order):
    if neighbor_order:
        g = SquareGridNeighborOrder(30, 15)
        g.NEIGHBOR_ORDER = neighbor_order
    else:
        g = SquareGrid(30, 15)
    g.walls = DIAGRAM1_WALLS
    start, goal = (8, 7), (27, 2)
    came_from = breadth_first_search(g, start, goal)
    draw_grid(g, path=reconstruct_path(came_from, start=start, goal=goal),
              point_to=came_from, start=start, goal=goal)

class GridWithAdjustedWeights(GridWithWeights):
    def cost(self, from_node, to_node):
        prev_cost = super().cost(from_node, to_node)
        nudge = 0
        (x1, y1) = from_node
        (x2, y2) = to_node
        if (x1 + y1) % 2 == 0 and x2 != x1: nudge = 1
        if (x1 + y1) % 2 == 1 and y2 != y1: nudge = 1
        return prev_cost + 0.001 * nudge

def main():
    printf("Implementatin.py")

if __name__ == "__main__":
    main()
