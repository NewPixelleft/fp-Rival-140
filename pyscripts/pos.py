# There are 12 main waypoints and 4 aruco waypoints
# waypoints 0-11 are regular waypoints
# waypoints 12-15 are aruco waypoints

REGULAR_WAYPOINT_COUNT : int = 12

waypoints = [
    [-12, 0],       # 0
    [-12, -3],      # 1
    [-3, -12],      # 2
    [0, -12],       # 3
    [3, -12],       # 4
    [12, -3],       # 5
    [12, 0],        # 6
    [12, 3],        # 7
    [3, 12],        # 8
    [0, 12],        # 9
    [-3, 12],       # 10
    [-12, 3],       # 11
    [-12, 12],      # 12
    [12, -12],      # 13
    [-12, -12],     # 14
    [12, 12]        # 15
]

# Relations
# waypoint i should connect to waypoint i + 1 and i - 1 
# for 0 <= i <= 11

# accepts index i, returns i-1
def previousWaypoint(i: int) -> int:
    return (i - 1) % REGULAR_WAYPOINT_COUNT

def nextWaypoint(i: int) -> int:
    return (i + 1) % REGULAR_WAYPOINT_COUNT

def isRegularWaypoint(i: int) -> bool:
    return 0 <= i < REGULAR_WAYPOINT_COUNT

def canGoToTop(i: int) -> bool:
    return 0 <= i < REGULAR_WAYPOINT_COUNT and i % 3 == 0

graph = {
    0: [11, 1],
    1: [0, 2, 14],
    2: [1, 3, 14],
    3: [2, 4],
    4: [3, 5, 13],
    5: [4, 6, 13],
    6: [5, 7],
    7: [6, 8, 15],
    8: [7, 9, 15],
    9: [8, 10],
    10: [9, 11, 12],
    11: [10, 0, 12],

    # ArUco waypoints: connections TBD
    12: [10, 11],
    13: [4, 5],
    14: [1, 2],
    15: [7, 8]
}