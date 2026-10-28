
import sys

from utilities import Logger, euler_from_quaternion
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom
from rclpy import init, spin, shutdown

rawSensor = 0

class localization(Node):

    def __init__(self, localizationType=rawSensor):

        super().__init__("localizer")

        # Part 3: Best-effort reception accepts reliable or best-effort odometry
        # publishers. Check the active robot with ros2 topic info /odom --verbose.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create CSV logger
        self.loc_logger = Logger("robot_pose.csv", ["x", "y", "theta", "stamp"])
        self.pose = None

        # Subscribe to odometry
        if localizationType == rawSensor:
            self.odom_sub = self.create_subscription(
                odom,
                "/odom",
                self.odom_callback,
                odom_qos
            )
        else:
            print("This type doesn't exist", file=sys.stderr)

    def odom_callback(self, pose_msg):

        # Read X and Y position
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = pose_msg.pose.pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Read timestamp
        stamp = pose_msg.header.stamp

        # Store robot position
        self.pose = [x, y, theta, stamp]

        # Save position data to CSV
        self.loc_logger.log_values([
            x, y, theta,
            Time.from_msg(stamp).nanoseconds
        ])

    def getPose(self):
        return self.pose


# Run only when executing localization.py directly
if __name__ == "__main__":

    init()
    node = localization()

    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        shutdown()
