import heapq
import math
import time

# Data input
cities = {
    "A": (0, 0),
    "B": (2, 1),
    "C": (4, 2),
    "D": (5, 5),
    "E": (1, 4)
}

roads = {
    "A": ["B", "E"],
    "B": ["A", "C"],
    "C": ["B", "D"],
    "D": ["C"],
    "E": ["A", "D"]
}

def heuristic(a, b):
    # Euclidean Distance
    (x1, y1) = cities[a]
    (x2, y2) = cities[b]
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)

def a_star_search(start, goal):
    frontier = [(0, start)]
    came_from = {start: None}
    cost_so_far = {start: 0}
    explored_nodes = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        explored_nodes += 1

        if current == goal:
            break

        for neighbor in roads[current]:
            new_cost = cost_so_far[current] + heuristic(current, neighbor)
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)
                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    # Reconstruct path
    path = []
    node = goal
    while node:
        path.append(node)
        node = came_from[node]
    path.reverse()

    return path, cost_so_far[goal], explored_nodes

# Contoh eksekusi
start_time = time.time()
path, cost, nodes = a_star_search("A", "D")
end_time = time.time()

print("Path:", " -> ".join(path))
print("Total cost:", cost)
print("Nodes explored:", nodes)
print("Time taken (ms):", (end_time - start_time) * 1000)
