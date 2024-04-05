import heapq
from math import sqrt


def heuristic(a, b):
    """Calculate the Euclidean distance between two points a and b"""
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def is_valid_neighbor(neighbor, array):
    """Check if a neighbor is within the bounds of the array and not a wall"""
    x, y = neighbor
    if 0 <= x < array.shape[0] and 0 <= y < array.shape[1]:
        return array[x][y] != 1
    return False


def aStar(array, start, goal):
    """Find the shortest path from start to goal using the A* algorithm"""
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    closed_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    open_heap = []
    open_set = set([start])

    heapq.heappush(open_heap, (fscore[start], start))

    while open_heap:
        current = heapq.heappop(open_heap)[1]
        open_set.remove(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            return path

        closed_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if not is_valid_neighbor(neighbor, array):
                continue

            tentative_g_score = (
                gscore[current]
                + heuristic(current, neighbor)
                + array[neighbor[0]][neighbor[1]]
            )

            if neighbor in closed_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if neighbor not in open_set or tentative_g_score < gscore.get(neighbor, 0):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    open_set.add(neighbor)
                    heapq.heappush(open_heap, (fscore[neighbor], neighbor))

    return False
