"""ROS 2 odometry + IMU localizer for MTE 544 Lab 3.

Use the existing LabThree repository's utilities.py for Logger and quaternion
conversion. EKF results are supplied to decisions.py and written to CSV.
"""

import sys

import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry as odom
from sensor_msgs.msg import Imu

from utilities import Logger, euler_from_quaternion
from kalman_filter import kalman_filter

rawSensors = 0
kalmanFilter = 1

# BEST_EFFORT subscription receives both BEST_EFFORT IMU publishers and
# RELIABLE odometry publishers. VOLATILE matches common sensor-topic settings.
# If your hardware publishes /imu with a different frame, account for the
# transform before fusing its accelerations.
_sensor_qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=30,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
odom_qos = _sensor_qos

DEFAULT_HEADERS = [
    "odom_x", "odom_y", "odom_v", "odom_w",
    "imu_ax", "imu_ay",
    "kf_x", "kf_y", "kf_theta", "kf_w", "kf_v", "kf_ax", "kf_ay",
    "stamp",
]


class localization(Node):
    def __init__(self, type=kalmanFilter, dt=0.1,
                 loggerName="robot_pose.csv", loggerHeaders=None,
                 odom_topic="/odom", imu_topic="/imu"):
        super().__init__("localizer")
        self.loc_logger = Logger(
            loggerName, DEFAULT_HEADERS if loggerHeaders is None else loggerHeaders
        )  # Fixes "elf.loc_logger" typo in the starter file.
        self.pose = None
        self.nominal_dt = float(dt)
        self.last_stamp = None
        self.have_initial_state = False

        if type == rawSensors:
            self.initRawSensors(odom_topic)
        elif type == kalmanFilter:
            self.initKalmanfilter(dt, odom_topic, imu_topic)
        else:
            raise ValueError("Unknown localization type: " + str(type))

    def initRawSensors(self, odom_topic="/odom"):
        self.raw_sub = self.create_subscription(
            odom, odom_topic, self.odom_callback, qos_profile=odom_qos
        )

    def initKalmanfilter(self, dt, odom_topic="/odom", imu_topic="/imu"):
        # x = [x, y, theta, omega, v, vdot] -- six state variables.
        x = np.zeros(6)
        # Lab 3 suggests starting with Q=0.5*I_6 and R=0.5*I_4.
        # Change these for the four experimental Q/R combinations.
        Q = 0.5 * np.eye(6)   # process/model uncertainty
        R = 0.8 * np.eye(4)#np.diag([0.4,0.4,0.5,0.5]) #$mddemendfsft uncertainty
        P = 0.5 * np.eye(6)   # initial state-estimation uncertainty
        self.kf = kalman_filter(P, Q, R, x, dt)

        # Subscribe to both topics; only run the filter when we have a pair
        # with similar message timestamps.
        self.odom_sub = message_filters.Subscriber(
            self, odom, odom_topic, qos_profile=_sensor_qos
        )
        self.imu_sub = message_filters.Subscriber(
            self, Imu, imu_topic, qos_profile=_sensor_qos
        )
        # Hold a reference to the synchronizer for the node's lifetime.
        self.time_syncher = message_filters.ApproximateTimeSynchronizer(
            [self.odom_sub, self.imu_sub], queue_size=30, slop=0.15
        )
        self.time_syncher.registerCallback(self.fusion_callback)

    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        # Odom linear.x and angular.z are robot-frame forward speed and yaw rate.
        # IMU x/y accelerations are assumed robot-frame (level robot).
        measured_v = float(odom_msg.twist.twist.linear.x)
        measured_w = float(odom_msg.twist.twist.angular.z)
        measured_ax = float(imu_msg.linear_acceleration.x)
        measured_ay = float(imu_msg.linear_acceleration.y)
        z = np.array([measured_v, measured_w, measured_ax, measured_ay])

        odom_x = float(odom_msg.pose.pose.position.x)
        odom_y = float(odom_msg.pose.pose.position.y)
        odom_theta = float(euler_from_quaternion(odom_msg.pose.pose.orientation))
        stamp = (float(odom_msg.header.stamp.sec) +
                 float(odom_msg.header.stamp.nanosec) * 1e-9)

        # Do not integrate duplicate/out-of-order samples as new motion.
        # Restart this node when intentionally resetting the simulation clock.
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return

        if not self.have_initial_state:
            # Keep the estimated trajectory in the same initial odom frame.
            # Thereafter, x/y are obtained ONLY by EKF motion integration.
            self.kf.x = np.array([
                odom_x, odom_y, odom_theta,
                measured_w, measured_v, measured_ax,
            ], dtype=float)
            self.kf.update(z)
            self.have_initial_state = True
        else:
            # Sensor timestamp difference supplies the actual integration dt.
            # Bound the integration interval after an unusually long pause.
            delta_t = stamp - self.last_stamp
            self.kf.dt = (delta_t if 0.0 < delta_t <= 1.0
                          else self.nominal_dt)
            self.kf.predict()
            self.kf.update(z)

        self.last_stamp = stamp
        xhat = self.kf.get_states()
        # Controller expects [x, y, theta, ROS timestamp] (not float seconds).
        self.pose = [float(xhat[0]), float(xhat[1]), float(xhat[2]),
                     odom_msg.header.stamp]

        # Include both raw measurements and EKF estimates for report plots.
        self.loc_logger.log_values([
            odom_x, odom_y, measured_v, measured_w,
            measured_ax, measured_ay,
            float(xhat[0]), float(xhat[1]), float(xhat[2]),
            float(xhat[3]), float(xhat[4]), float(xhat[5]),
            float(xhat[3] * xhat[4]), stamp,
        ])

    def odom_callback(self, pose_msg):
        # Retains the Lab 2 raw-odometry mode if needed.
        self.pose = [
            pose_msg.pose.pose.position.x,
            pose_msg.pose.pose.position.y,
            euler_from_quaternion(pose_msg.pose.pose.orientation),
            pose_msg.header.stamp,
        ]

    def getPose(self):
        return self.pose


if __name__ == "__main__":
    rclpy.init()
    node = localization(kalmanFilter, dt=0.1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
