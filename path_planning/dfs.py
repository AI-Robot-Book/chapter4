# https://www.educative.io/edpresso/how-to-implement-depth-first-search-in-python
graph = {
    'A': ['B', 'C', "D"],
    'B': ['E', "F"],
    'C': ['G', "I"],
    'D': ["I"],
    'E': [],
    "F": [],
    'G': [],
    "I": []
}

def dfs(visited, graph, start, goal):
    stack = []
    visited.append(start)
    stack.append(start)

    while stack:
        current = stack[-1]
        stack.pop()
        print(current)

        if current == goal:
            break

        for neighbour in graph[current]:
            if neighbour not in visited:
                visited.append(neighbour)
                stack.append(neighbour)


dfs([], graph, 'A', 'G')
