import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
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

        self.x = None
        self.y = None
        self.yaw = None
        self.latest_odom = None

        self.YawCalibration = 180.0 / 125.5
        self.previous_yaw = None
        self.corrected_yaw = 0.0

        self.stable_since = None
        self.finished = False

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
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
        
    def odom_callback(self, msg):
            self.latest_odom = msg
            
            xloc = msg.pose.pose.position.x
            yloc = msg.pose.pose.position.y
   
            self.x, self.y = localToGlobal(
                xloc,
                yloc,
                initialX,
                initialY,
                initialYaw
            )
    
            q = msg.pose.pose.orientation
    
            sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
            cos_yaw = 1.0 - 2.0  * (q.y**2  + q.z**2)
    
            yaw = math.atan2(sin_yaw, cos_yaw)
    
            if self.previous_yaw is None:
                self.previous_yaw = yaw
                return
    
            dyaw = yaw - self.previous_yaw
            dyaw = math.atan2(
                math.sin(dyaw), math.cos(dyaw)
            )
    
            self.corrected_yaw += dyaw * self.YawCalibration
            self.previous_yaw = yaw

            # self.get_logger().info(
            #     f'position: ({x:.2f}, {y:.2f}) |'
            #     f'Yaw: ({math.degrees(self.corrected_yaw):.2f}) degrees'
            # )

    def control_loop(self):
        if self.x is None or self.y is None:
            return

        dx = self.goalX - self.x
        dy = self.goalY - self.y

        cmd = Twist()
        distance = math.hypot(dx, dy)

        if distance < 0.20:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_publisher.publish(cmd)
            self.get_logger().info('YEEE NYAMPE')
            return

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