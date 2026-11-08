import rclpy
from rclpy.node import Node

import navigation.initialPosition as ipp

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped

import math


INITIAL_X = ipp.__initial_x
INITIAL_Y = ipp.__initial_y
INITIAL_Z = ipp.__initial_z


def normalize_quaternion(q):
    norm = math.sqrt(
        q.w**2 + q.x**2 + q.y**2 + q.z**2
    )

    if norm < 1e-9:
        return None

    return (
        q.w / norm,
        q.x / norm,
        q.y / norm,
        q.z / norm
    )


def rotate_vector(q, vector):
    """
    Rotate a 3D vector using quaternion q.

    q = (w, x, y, z)
    """

    w, x, y, z = q
    vx, vy, vz = vector

    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)

    return rx, ry, rz


def quaternion_to_euler(q):
    w, x, y, z = q

    # Roll
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x**2 + y**2)
    roll = math.atan2(sin_roll, cos_roll)

    # Pitch
    sin_pitch = 2.0 * (w * y - z * x)
    sin_pitch = max(-1.0, min(1.0, sin_pitch))
    pitch = math.asin(sin_pitch)

    # Yaw
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y**2 + z**2)
    yaw = math.atan2(sin_yaw, cos_yaw)

    return roll, pitch, yaw


class PositionLogger(Node):

    def __init__(self):
        super().__init__('position_logger')

        self.x = INITIAL_X
        self.y = INITIAL_Y
        self.z = INITIAL_Z

        # IMU orientation
        self.orientation = None

        # Time tracking
        self.previous_time = None

        # Subscribers
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            '3D IMU-assisted Position Logger Started'
        )

        self.pose_publisher = self.create_publisher(
            PoseStamped, 
            '/localization/pose',
            10
        )

    def imu_callback(self, msg):

        self.orientation = normalize_quaternion(
            msg.orientation
        )

    def publish_pose(self, stamp):
        if self.orientation is None:
            return
        pose_msg = PoseStamped()

        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.z

        w, x, y, z = self.orientation

        pose_msg.pose.orientation.w = w
        pose_msg.pose.orientation.x = x
        pose_msg.pose.orientation.y = y
        pose_msg.pose.orientation.z = z

        self.pose_publisher.publish(pose_msg)

    def odom_callback(self, msg):

        if self.orientation is None:
            return

        stamp = msg.header.stamp

        current_time = (
            stamp.sec +
            stamp.nanosec * 1e-9
        )

        if self.previous_time is None:
            self.previous_time = current_time
            return

        dt = current_time - self.previous_time

        self.previous_time = current_time

        if dt <= 0.0 or dt > 0.2:
            return

        # Get all three body-frame velocity components
        twist = msg.twist.twist

        body_velocity = (
            twist.linear.x,
            twist.linear.y,
            twist.linear.z
        )

        # Transform velocity into world frame
        world_vx, world_vy, world_vz = rotate_vector(
            self.orientation,
            body_velocity
        )

        # Integrate global position
        self.x += world_vx * dt
        self.y += world_vy * dt
        self.z += world_vz * dt
        self.publish_pose(msg.header.stamp)

        # Extract orientation for diagnostics
        roll, pitch, yaw = quaternion_to_euler(
            self.orientation
        )

        self.get_logger().info(
            f'Position: '
            f'X={self.x:.4f} m | '
            f'Y={self.y:.4f} m | '
            f'Z={self.z:.4f} m || '

            f'Velocity: '
            f'X={world_vx:.3f} | '
            f'Y={world_vy:.3f} | '
            f'Z={world_vz:.3f} || '

            f'Orientation: '
            f'Roll={math.degrees(roll):.2f}° | '
            f'Pitch={math.degrees(pitch):.2f}° | '
            f'Yaw={math.degrees(yaw):.2f}°'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PositionLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()