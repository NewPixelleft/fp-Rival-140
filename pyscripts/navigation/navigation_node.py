import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Int32MultiArray, Bool
from geometry_msgs.msg import PoseStamped

import waypoints as wp
from pathfinder import astarFromPos


ARRIVAL_TOLERANCE = 0.1


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Publishes the next waypoint to DrivingNode
        self.target_publisher = self.create_publisher(
            Float64MultiArray,
            '/navigation/target',
            10
        )

        # Receives destination coordinates from WaypointController
        self.route_subscription = self.create_subscription(
            Int32MultiArray,
            '/navigation/waypoint_route',
            self.route_callback,
            10
        )

        # Receives arrival notifications from DrivingNode
        self.arrival_subscription = self.create_subscription(
            Bool,
            '/navigation/arrived',
            self.arrival_callback,
            10
        )

        # Receives current rover position
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10
        )
        self.mission_status_publisher = self.create_publisher(
            Bool,
            '/navigation/mission_active',
            10
        )

        self.x = None
        self.y = None

        self.route = []
        self.current_index = 0

        self.requested_route = []
        self.requested_index = 0

        self.final_target = None
        self.final_target_pending = False

        self.waiting = False
        self.current_target_is_final = False

        self.get_logger().info('Navigation Node Initialized')

    def set_mission_active(self, active: bool): 
        msg = Bool()
        msg.data = active

        self.mission_status_publisher.publish(msg)

    def route_callback(self, msg: Int32MultiArray):
        if not msg.data:
            self.get_logger().error('Empty waypoint route')
            return
        requested_route = list(msg.data)

        for waypoint_id in requested_route:
            if waypoint_id not in wp.graph:
                self.get_logger().error(
                    f'Invalid waypoint ID: {waypoint_id}'
                )
                return

        if self.x is None or self.y is None:
            self.get_logger().error(
                'Localization is not available'
            )
            return

        self.requested_route = requested_route
        self.requested_index = 0

        self.get_logger().info(
            f'Received requested route: {requested_route}'
        )

        self.start_requested_destination()

    def pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def destination_callback(self, msg: Float64MultiArray):

        if len(msg.data) != 2:
            self.get_logger().error('Invalid destination coordinates')
            return

        x = float(msg.data[0])
        y = float(msg.data[1])

        if not math.isfinite(x) or not math.isfinite(y):
            self.get_logger().error('Destination must contain finite coordinates')
            return

        if self.x is None or self.y is None:
            self.get_logger().error('Localization is not available yet')
            return

        self.plan_route(x, y)


    def find_destination_waypoint(self, x: float, y: float) -> int:
        for i, (wx, wy) in enumerate(wp.waypoints):

            distance = math.hypot(wx - x, wy - y)

            if distance <= ARRIVAL_TOLERANCE:
                return i


        candidates = list(range(wp.REGULAR_WAYPOINT_COUNT))
        candidates.append(16)

        nearest = min(
            candidates,
            key=lambda i: math.hypot(
                wp.waypoints[i][0] - x,
                wp.waypoints[i][1] - y
            )
        )

        return nearest

    def send_target(self, x: float, y: float):
        msg = Float64MultiArray()
        msg.data = [float(x), float(y)]

        self.target_publisher.publish(msg)

    def start_requested_destination(self):
        self.set_mission_active(True)
        if self.requested_index >= len(self.requested_route):
            self.get_logger().info(
                'All requested destinations completed!'
            )
            return

        goal = self.requested_route[self.requested_index]

        self.get_logger().info(
            f'Calculating A* route to WP {goal}'
        )

        route = astarFromPos(
            self.x,
            self.y,
            goal
        )

        if route is None:
            self.get_logger().error(
                f'A* failed to find a route to WP {goal}'
            )
            self.set_mission_active(False)
            return

        self.route = route
        self.current_index = 0

        self.waiting = False

        self.get_logger().info(
            f'A* calculated route: {self.route}'
        )

        self.send_next_target()

    def plan_route(self, x: float, y: float):
        goal = self.find_destination_waypoint(x, y)

        self.get_logger().info(
            f'Planning route to ({x:.2f}, {y:.2f})'
        )

        route = astarFromPos(
            self.x,
            self.y,
            goal
        )

        if route is None:
            self.get_logger().error('A* could not find a valid route')
            return

        # Replace the current route.
        self.route = route
        self.current_index = 0

        self.final_target = (x, y)

        goal_x, goal_y = wp.waypoints[goal]

        self.final_target_pending = (
            math.hypot(goal_x - x, goal_y - y)
            > ARRIVAL_TOLERANCE
        )

        self.waiting = False
        self.current_target_is_final = False

        self.get_logger().info(
            f'A* Route: {self.route}'
        )

        self.send_next_target()

    def send_next_target(self):
        if self.current_index >= len(self.route):

            self.get_logger().info(
                f'Completed destination '
                f'{self.requested_route[self.requested_index]}'
            )

            self.requested_index += 1

            if self.requested_index < len(self.requested_route):
                self.start_requested_destination()
            else:
                self.waiting = False
                self.set_mission_active(False)

                self.get_logger().info(
                    'ALL REQUESTED DESTINATIONS COMPLETED!'
                )

            return

        waypoint_id = self.route[self.current_index]
        x, y = wp.waypoints[waypoint_id]

        self.get_logger().info(
            f'Sending target WP {waypoint_id}: ({x}, {y})'
        )

        self.send_target(x, y)
        self.waiting = True

    
    

    def arrival_callback(self, msg: Bool):
        if not msg.data:
            return
        if not self.waiting:
            self.get_logger().warn(
                'Received arrival signal while not waiting'
            )
            return
        waypoint_id = self.route[self.current_index]

        self.get_logger().info(
            f'ARRIVED at WP {waypoint_id}'
        )
        self.waiting = False
        self.current_index += 1

        self.send_next_target()

def main(args=None):

    rclpy.init(args=args)

    node = NavigationNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()