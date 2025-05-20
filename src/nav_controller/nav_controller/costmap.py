import numpy as np

def inflate(grid: np.ndarray, radius: int) -> np.ndarray:
    """
    Mark all cells within 'radius' of any obstacle (value=1) as occupied (1).
    grid: binary array, 1=obstacle
    radius: number of cells to expand
    """
    h, w = grid.shape
    inflated = grid.copy()
    # For each obstacle cell, mark neighbors
    for i in range(h):
        for j in range(w):
            if grid[i, j] == 1:
                for di in range(-radius, radius+1):
                    for dj in range(-radius, radius+1):
                        ni, nj = i + di, j + dj
                        # Check in‑bounds
                        if 0 <= ni < h and 0 <= nj < w:
                            inflated[ni, nj] = 1
    return inflated