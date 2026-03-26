import heapq
import time

def astar(grid, start, goal, heuristic):
    """
    grid: 2D list representing the grid (0 for walkable, 1 for wall)
    start: tuple (x, y) starting position
    goal: tuple (x, y) goal position
    heuristic: function that takes two positions and returns an estimated cost
    """
    rows, cols = len(grid), len(grid[0])
    open_set = []
    heapq.heappush(open_set, (0, 0, start))  # (f_score, g_score, position)
    came_from = {}
    g_score = {start: 0}
    nodes_expanded = 0
    start_time = time.time()
    while open_set:
        f, g, current = heapq.heappop(open_set)
        nodes_expanded += 1
        if current == goal:
            end_time = time.time()
            return {
                "path": reconstruct_path(came_from, current),
                "cost": g_score[current],
                "nodes_expanded": nodes_expanded,
                "runtime": end_time - start_time
            }
        for neighbor in get_neighbors(current, grid, rows, cols):
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))
    return None  # No path found

def get_neighbors(position, grid, rows, cols):
    x, y = position
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    neighbors = []
    for dx, dy in directions:
        nx, ny = x + dx, y + dy
        if 0 <= nx < rows and 0 <= ny < cols:
            if grid[nx][ny] == 0:  # Walkable
                neighbors.append((nx, ny))
    return neighbors

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path

def zero_heuristic(a, b):
    return 0

def manhattan_heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Non-admissible (Weighted A*)
def weighted_manhattan(a, b, weight=1.2):
    return weight * manhattan_heuristic(a, b)

def weighted_heuristic(a, b):
    return weighted_manhattan(a, b, 1.2)

import random

def generate_grid(rows, cols, obstacle_prob=0.2, seed=41):
    random.seed(seed)
    grid = []
    for _ in range(rows):
        row = []
        for _ in range(cols):
            if random.random() < obstacle_prob:
                row.append(1)  # Wall
            else:
                row.append(0)  # Walkable
        grid.append(row)
    return grid

def run_experiment():
    start = (0, 0)
    goal = (19, 19)
    seed = 51
    while True:
        grid = generate_grid(20,20, obstacle_prob=0.2, seed=seed)
        grid[start[0]][start[1]] = 0  # Ensure start is walkable
        grid[goal[0]][goal[1]] = 0    # Ensure goal is walkable
        if astar(grid, start, goal, zero_heuristic) is not None:
            break
        seed += 1
    heuristics = {
        "Zero (Dijkstra)": zero_heuristic,
        "Manhattan": manhattan_heuristic,
        "Weighted Manhattan": weighted_heuristic
    }
    results = {}
    for name, h in heuristics.items():
        result = astar(grid, start, goal, h)
        results[name] = result
    return results

if __name__ == "__main__":
    results = run_experiment()
    for h_name, result in results.items():
        if result:
            print(f"\n{h_name}")
            print(f"Cost: {result['cost']}")
            print(f"Nodes Expanded: {result['nodes_expanded']}")
            print(f"Runtime: {result['runtime']:.6f} seconds")
        else:
            print(f"\n{h_name}: No path found")