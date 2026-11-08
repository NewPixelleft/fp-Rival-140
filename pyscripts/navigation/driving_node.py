import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import PoseStamped, Twist

MAX_TURN_SPEED = 0.2
TURN_TOLERANCE = 0.01

class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')
        self.x = None
        self.y = None
        self.yaw = None

        self.goalX = None
        self.goalY = None

        self.arrived = True

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/localization/pose',
            self.pose_callback,
            10
        )
        self.target_subscription = self.create_subscription(
            Float64MultiArray,
            '/navigation/target',
            self.target_callback,
            10
        )

        self.status_publisher = self.create_publisher(
            Bool,
            '/navigation/arrived',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )
        self.get_logger().info('Driving Node Initialized')

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        q = msg.pose.orientation

        sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
        cos_yaw = 1.0 - 2.0 * (q.y**2 + q.z**2)

        self.yaw = math.atan2(sin_yaw, cos_yaw)

    def set_target(self, x: float, y: float):
        self.goalX = x
        self.goalY = y
        self.arrived = False
        self.get_logger().info(
            f'New target: ({x:.2f}, {y:.2f})'
        )
    def target_callback(self, msg: Float64MultiArray):
        if len(msg.data) != 2:
            self.get_logger().error('Invalid target coordinates')
            return

        self.set_target(msg.data[0], msg.data[1])

    def stop(self):
        self.cmd_publisher.publish(Twist())


    def control_loop(self):
        # No target
        if self.arrived or self.goalX is None:
            self.stop()
            return
        # Wait for localization
        if self.x is None or self.y is None or self.yaw is None:
            return

        # Calculate distance to target
        dx = self.goalX - self.x
        dy = self.goalY - self.y

        distance = math.hypot(dx, dy)

        # does it ever reach the control loop?
        self.get_logger().info('Loop activated')

        # Appkh nyampe
        if distance < 0.10:
            self.stop()
            self.arrived = True

            status = Bool()
            status.data = True
            self.status_publisher.publish(status)

            self.get_logger().info('Destination reached!')
            return

        # handling error
        target_yaw = math.atan2(dy, dx)

        yaw_error = target_yaw - self.yaw

        yaw_error = math.atan2(
            math.sin(yaw_error),
            math.cos(yaw_error)
        )
        # Movement controller
        cmd = Twist()

        cmd = Twist()

        if abs(yaw_error) > TURN_TOLERANCE:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -MAX_TURN_SPEED,
                min(MAX_TURN_SPEED, yaw_error)
            )
        else:
            cmd.linear.x = 0.25
            cmd.angular.z = 0.0

        self.get_logger().info(
            f'Current yaw: {math.degrees(self.yaw):.3f} deg | '
            f'Target yaw: {math.degrees(target_yaw):.3f} deg | '
            f'Error: {math.degrees(yaw_error):.3f} deg | '
            f'Angular cmd: {cmd.angular.z:.4f} | '
            f'Linear cmd: {cmd.linear.x:.4f}'
        )

        self.cmd_publisher.publish(cmd)
    

def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()