# Finding path along the map
# Utilizes a*

import heapq
import math
import waypoints as wp

def getDistance(a: int, b: int) -> float:
    xa, ya = wp.waypoints[a]
    xb, yb = wp.waypoints[b]
    return math.hypot(xb - xa, yb - ya)

def isInWaypoint(a: int, x: float, y: float) -> bool:
    xa, ya = wp.waypoints[a]
    return math.hypot(xa - x, ya - y) <= 0.1

def getNearest(x: float, y: float) -> int:
    minval = math.inf
    taken = -1

    for i, (xa, ya) in enumerate(wp.waypoints):
        distance = math.hypot(xa - x, ya - y)

        if distance < minval:
            minval = distance
            taken = i

    return taken

def isOnAxis(x: float, y: float, tolerance: float = 0.1) -> bool:
    return abs(x) <= tolerance or abs(y) <= tolerance


def heuristic(a: int, b: int) -> float:
    return getDistance(a, b)


def astar(start: int, goal: int):
    if start not in wp.graph or goal not in wp.graph:
        return None

    openset = [(heuristic(start, goal), start)]

    gscore = {
        w: math.inf
        for w in wp.graph
    }

    prev = {}
    gscore[start] = 0.0

    while openset:
        current_f, current = heapq.heappop(openset)

        # Ignore outdated entries in the priority queue
        if current_f > gscore[current] + heuristic(current, goal):
            continue

        if current == goal:
            path = []

            while current in prev:
                path.append(current)
                current = prev[current]

            path.append(start)
            return path[::-1]

        for neighbor in wp.graph[current]:
            tentG = (
                gscore[current]
                + getDistance(current, neighbor)
            )

            if tentG < gscore[neighbor]:
                prev[neighbor] = current
                gscore[neighbor] = tentG

                fscore = (
                    tentG
                    + heuristic(neighbor, goal)
                )

                heapq.heappush(
                    openset,
                    (fscore, neighbor)
                )

    return None


def astarFromPos(x: float, y:float, goal: int):
    start = getNearest(x, y)
    if (start == -1):
        return None
    return astar(start, goal)
