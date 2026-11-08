import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs import Twist
import math

initialX : float = 0
initialY : float = 0
initialYaw : float = 0

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

# YawCalibration : float = 180.0 / 125.0
class OdomSubscriber(Node):
    # python like constructor in C++ terms
    def __init__(self):
        super().__init__('odom_subscriber') # Constructor of 'Node'

        self.previous_yaw = None
        self.corrected_yaw = 0.0
        self.YawCalibration = 180.0 / 125.0

        self.x = None
        self.y = None
        self.latest_odom = None

        self.goalX = 0.0
        self.goalY = 0.0

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            10
        )


        self.timer = self.create_timer(
            0.1, #Every 0.1s
            self.control_loop
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom', # The /odom topic
            self.odom_callback,
            10
        )

    ### ENDFUNCTION

    def odom_callback(self, msg):
        self.latest_odom = msg
        
        xloc = msg.pose.pose.position.x + initialX
        yloc = msg.pose.pose.position.y + initialY

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

        self.get_logger().info(
            f'position: ({x:.2f}, {y:.2f}) |'
            f'Yaw: ({math.degrees(self.corrected_yaw):.2f}) degrees'
        )



def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


