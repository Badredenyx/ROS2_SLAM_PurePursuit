# nav_controller/planner.py
import heapq
import numpy as np
from .utils import heuristic

LOOKAHEAD = 0.15
EXPANSION = 2


def costmap(data, width, height, resolution):
    grid = np.array(data).reshape(height, width)
    walls = np.argwhere(grid == 100)
    for dx in range(-EXPANSION, EXPANSION + 1):
        for dy in range(-EXPANSION, EXPANSION + 1):
            if dx == 0 and dy == 0:
                continue
            shifted = walls + [dx, dy]
            shifted[:,0] = np.clip(shifted[:,0], 0, height - 1)
            shifted[:,1] = np.clip(shifted[:,1], 0, width - 1)
            grid[shifted[:,0], shifted[:,1]] = 100
    grid = grid * resolution
    return grid


def astar(grid, start, goal):
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    closed = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    open_heap = [(fscore[start], start)]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        closed.add(current)
        for dx, dy in neighbors:
            nb = (current[0] + dx, current[1] + dy)
            tentative = gscore[current] + heuristic(current, nb)
            if not (0 <= nb[0] < grid.shape[0] and 0 <= nb[1] < grid.shape[1]):
                continue
            if grid[nb[0], nb[1]] == 1:
                continue
            if nb in closed and tentative >= gscore.get(nb, float('inf')):
                continue
            if tentative < gscore.get(nb, float('inf')):
                came_from[nb] = current
                gscore[nb] = tentative
                fscore[nb] = tentative + heuristic(nb, goal)
                heapq.heappush(open_heap, (fscore[nb], nb))
    # no full path: return closest reach
    if goal not in came_from:
        best = min(closed, key=lambda n: heuristic(n, goal), default=None)
        if best:
            path = []
            while best in came_from:
                path.append(best)
                best = came_from[best]
            path.append(start)
            return path[::-1]
    return []
