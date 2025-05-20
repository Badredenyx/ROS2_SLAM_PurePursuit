import heapq  # binary heap for priority queue
import numpy as np

def heuristic(a, b):
    """
    Manhattan distance between grid cells a,b.
    Suitable heuristic for 4‑connected grids.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def reconstruct_path(came_from, start, goal):
    """
    Walk backwards from goal to start using came_from map.
    """
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    path.reverse()
    return path


def astar(grid: np.ndarray, start: tuple, goal: tuple):
    """
    grid: 2D numpy array; 0=free, 1=occupied
    start, goal: (row, col) tuples
    Returns: list of (row,col) path from start to goal, or [] if none
    """
    h, w = grid.shape
    # Open set: heap of (f_score, g_score, node, parent)
    open_set = []
    # Initialize with start node: f = heuristic(start,goal)
    heapq.heappush(open_set, (
        heuristic(start, goal),  # f_score = g+h, here g=0
        0,                        # g_score (cost from start)
        start,
        None                      # parent pointer
    ))
    came_from = {}     # maps node -> parent
    cost_so_far = {start: 0}  # g_score map

    while open_set:
        f, g, current, parent = heapq.heappop(open_set)
        if current == goal:
            # Found goal: record parent, reconstruct path
            came_from[current] = parent
            return reconstruct_path(came_from, start, goal)

        # Explore neighbors in 4 directions
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            # Check bounds and occupancy
            if (0 <= neighbor[0] < h and 0 <= neighbor[1] < w 
                and grid[neighbor] == 0):
                new_cost = g + 1
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    # Push with priority f = g+h
                    heapq.heappush(open_set, (priority, new_cost, neighbor, current))
                    came_from[neighbor] = current
    # No path found
    return []