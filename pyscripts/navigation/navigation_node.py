from std_msgs.msg import Float64MultiArray, Bool
import math
import waypoints as wp

from pathfinder import isOnAxis, astarFromPos

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, Twist

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        self.target_publisher = self.create_publisher(
            Float64MultiArray,
            '/navigation/target',
            10
        )
        self.arrival_subscription = self.create_subscription(
            Bool,
            '/navigation/arrived',
            self.arrival_callback,
            10
        )
        self.route = [0, 6, 8]
        self.current_index = 0
        self.waiting = False

        self.x = None
        self.y = None

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10
        )

        self.start_timer = self.create_timer(
            2.0,
            self.start_navigation
        )

        self.get_logger().info('Navigation Node Initialized')

    def set_route(self, route: list[int]):
        self.route = route
        self.current_index = 0
        self.waiting = False

        self.send_next_target()

    def start_navigation(self):
        self.start_timer.cancel()
        self.set_route(self.route)

    def pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def set_direct_target(self, x: float, y: float):
        self.route = []
        self.current_index = 0
        self.waiting = True

        msg = Float64MultiArray()
        msg.data = [float(x), float(y)]

        self.target_publisher.publish(msg)

        self.get_logger().info(
            f'Direct navigation to ({x}, {y})'
        )

    def send_next_target(self):
        if self.current_index >= len(self.route):
            self.get_logger().info('Route completed!')
            self.waiting = False
            return

        waypoint_id = self.route[self.current_index]

        x, y = wp.waypoints[waypoint_id]

        msg = Float64MultiArray()
        msg.data = [float(x), float(y)]

        self.target_publisher.publish(msg)

        self.waiting = True

        self.get_logger().info(
            f'Navigating to waypoint {waypoint_id}: ({x}, {y})'
        )

    def arrival_callback(self, msg: Bool):
        if not msg.data or not self.waiting:
            return

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