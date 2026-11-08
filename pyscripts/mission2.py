import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math
import time

XORG = -11
YORG = 0

def localToGlobal(xloc, yloc, xorg, yorg, yaw):
    xglob = (
        xorg
        + xloc * math.cos(yaw)
        - yloc * math.sin(yaw)
    )

    yglob = (
        yorg
        + xloc * math.sin(yaw)
        + yloc * math.cos(yaw)
    )

    return xglob, yglob

initialX = -11
initialY = 0
initialYaw = 0

class Mission2Controller(Node):
    def __init__(self):
        super().__init__('mission2_controller')
        self.goalX = 0.0
        self.goalY = 0.0

        self.locX = 0.0
        self.locY = 0.0

        self.x = initialX
        self.y = initialY

        self.previous_time = None
        self.yaw = None
        self.latest_odom = None
        self.timerCountWhenArrived = 0.0
        self.previous_time = None

        self.YawCalibration = 180.0 / 125.5
        self.previous_yaw = None
        self.corrected_yaw = 0.0
        self.pitch = None

        self.stable_since = None
        self.finished = False

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

    def imu_callback(self, msg):
        q = msg.orientation
        sinPitch = 2.0 * (q.w * q.y - q.z * q.x)
        sinPitch = max(-1.0, min(1.0, sinPitch))
        self.pitch = math.asin(sinPitch)

    def odom_callback(self, msg):
        if self.pitch is None:
            return

        stamp = msg.header.stamp

        current_time = (
            stamp.sec + stamp.nanosec * 1e-9
        )

        q = msg.pose.pose.orientation

        sinYaw = 2.0 * (q.w * q.z + q.x * q.y)
        cosYaw = 1.0 - 2.0 * (q.y**2 + q.z**2)

        yaw = math.atan2(sinYaw, cosYaw)

        # Initialize first sample
        if self.previous_time is None:
            self.previous_time = current_time
            self.previous_yaw = yaw
            return

        dt = current_time - self.previous_time

        if dt <= 0.0 or dt > 0.2:
            self.previous_time = current_time
            self.previous_yaw = yaw
            return

        # CALIBRATED YAW

        dyaw = yaw - self.previous_yaw

        dyaw = math.atan2(
            math.sin(dyaw),
            math.cos(dyaw)
        )

        self.corrected_yaw += (
            dyaw * self.YawCalibration
        )

        self.previous_yaw = yaw
        self.previous_time = current_time

        # IMU-ASSISTED POSITION

        velocity = msg.twist.twist.linear.x

        horizontal_velocity = (
            velocity * math.cos(self.pitch)
        )

        # Integrate movement in local coordinates
        self.locX += (
            horizontal_velocity
            * math.cos(self.corrected_yaw)
            * dt
        )

        self.locY += (
            horizontal_velocity
            * math.sin(self.corrected_yaw)
            * dt
        )

        self.x, self.y = localToGlobal(
            self.locX,
            self.locY,
            initialX,
            initialY,
            initialYaw
        )
        self.get_logger().info(
            f'Global: ({self.x:.3f}, {self.y:.3f}) | '
            f'Pitch: {math.degrees(self.pitch):.2f} | '
            f'Yaw: {math.degrees(self.corrected_yaw):.2f}'
        )


    def control_loop(self):
        if self.finished:
            self.cmd_publisher.publish(Twist())
            return
        
        if self.x is None or self.y is None:
            return

        dx = self.goalX - self.x
        dy = self.goalY - self.y

        cmd = Twist()
        distance = math.hypot(dx, dy)

        if distance < 0.1:
            self.cmd_publisher.publish(Twist())
            self.timerCountWhenArrived += 0.1
            if self.timerCountWhenArrived >= 3.0:
                self.finished = True
                self.get_logger().info("YEEE NYAMPE")
            return
        self.timerCountWhenArrived = 0.0

        targetYaw = math.atan2(dy, dx)
        currentYaw = initialYaw + self.corrected_yaw
        yawErr = targetYaw - currentYaw
        yawErr = math.atan2(
            math.sin(yawErr),
            math.cos(yawErr)
        )

        if abs(yawErr) > 0.12:
            cmd.linear.x = 0.0
            cmd.angular.z = max(
                -0.5,
                min(0.5, 1.5 * yawErr)
            )
        else:
            cmd.linear.x = 0.25
            cmd.angular.z = max(
                -0.3,
                min(0.3, yawErr)
            )    
        self.cmd_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)

    node = Mission2Controller()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Stop rover before shutting down
        stop_cmd = Twist()
        node.cmd_publisher.publish(stop_cmd)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()