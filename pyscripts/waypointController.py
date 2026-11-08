import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Bool
from geometry_msgs.msg import PoseStamped

import navigation.waypoints as wp
import argparse
import shlex

import threading


class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        self.route_publisher = self.create_publisher(
            Int32MultiArray,
            '/navigation/waypoint_route',
            10
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10
        )
        self.logging_publisher = self.create_publisher(
            Bool,
            '/mission3/logging_enabled',
            10
        )

        self.command_parser = self.create_command_parser()

        self.x = None
        self.y = None

        self.running = True
        self.scanning_publisher = self.create_publisher(
            Bool,
            '/mission3/scanning_enabled',
            10
        )

        self.get_logger().info(
            'FP RIVAL Initialized'
        )
        

    def create_command_parser(self):
        parser = argparse.ArgumentParser(
            prog='FP RIVAL fprivalN',
            description='Rival Rover Control Interface'
        )

        subparsers = parser.add_subparsers(
            dest='command'
        )

        moveto = subparsers.add_parser(
            'moveto',
            help='Navigate through specified waypoints'
        )

        moveto.add_argument(
            'waypoints',
            nargs='+',
            type=int
        )

        aruco_mode = moveto.add_mutually_exclusive_group()

        aruco_mode.add_argument(
            '--read-aruco-logged',
            action='store_true'
        )

        aruco_mode.add_argument(
            '--read-aruco-non-logged',
            action='store_true'
        )

        show = subparsers.add_parser(
            'show',
            help='Display rover information'
        )

        show.add_argument(
            'target',
            choices=['waypoints', 'graph', 'complete', 'position']
        )

        return parser

    def execute_command(self, args):

        if args.command == 'moveto':
            self.command_moveto(args)

        elif args.command == 'show':
            self.command_show(args.target)


    def command_moveto(self, args):
        route = args.waypoints

        for waypoint_id in route:
            if not 0 <= waypoint_id < len(wp.waypoints):
                print(f"Invalid waypoint index: {waypoint_id}")
                return

        scanning = (
            args.read_aruco_logged
            or args.read_aruco_non_logged
        )

        logging = args.read_aruco_logged

        scan_msg = Bool()
        scan_msg.data = scanning
        self.scanning_publisher.publish(scan_msg)

        log_msg = Bool()
        log_msg.data = logging
        self.logging_publisher.publish(log_msg)

        print("\nRequested route:", " -> ".join(map(str, route)))

        if args.read_aruco_logged:
            print("ArUco: Scanning + continuous logging")

        elif args.read_aruco_non_logged:
            print("ArUco: Scanning without continuous logging")

        else:
            print("ArUco: Disabled")

        self.publish_route(route)

    def command_show(self, target):
        if target == 'waypoints':
            self.handle_list()

        elif target == 'graph':
            self.handle_graph()

        elif target == 'complete':
            self.handle_list()
            self.handle_graph()

        elif target == 'position':
            self.handle_position()

    def pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y

    def run(self):

        print("\n======================")
        print("   FP RIVAL PRJ-140")
        print("======================")
        print("Type 'help' to see available commands.")

        while self.running and rclpy.ok():

            try:
                raw = input("\nfprivaln> ").strip()

                if not raw:
                    continue

                if raw.lower() in ('quit', 'exit'):
                    self.running = False
                    continue

                if raw.lower() in ('help', '?'):
                    self.command_parser.print_help()
                    continue

                try:
                    args = self.command_parser.parse_args(
                        shlex.split(raw)
                    )
                except SystemExit:
                    continue

                self.execute_command(args)

            except (KeyboardInterrupt, EOFError):
                self.running = False

    print("\nStopping FP RIVAL fprivalN.")


    def publish_route(self, route: list[int]):
        msg = Int32MultiArray()
        msg.data = route

        self.route_publisher.publish(msg)

        self.get_logger().info(
            f'Waypoint route published: {route}'
        )

    def handle_waypoint_navigation(self):

        print("\nEnter waypoint indexes.")
        print("Separate multiple indexes with spaces.")
        print("Example: 0 3 6 9")

        try:
            raw = input("Waypoint(s): ").strip()
            if not raw:
                print("Empty waypoint sequence.")
                return
            route = [int(value) for value in raw.split()]
            for waypoint_id in route:
                if not 0 <= waypoint_id < len(wp.waypoints):
                    print(f"Invalid waypoint index: {waypoint_id}")
                    return

            print("\nRequested waypoint sequence:")

            print(" -> ".join(map(str, route)))

            self.publish_route(route)
        except ValueError:
            print("Invalid input. Enter integer waypoint indexes. Please input 0 <= index <= 16")

    def handle_position(self):
        if self.x is None or self.y is None:
            print("\nLocalization is not available.")
            return
        print(
            f"\nCurrent Position: X = {self.x:.3f}, Y = {self.y:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node,),
        daemon=True
    )

    spin_thread.start()

    try:
        node.run()

    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()

