"""MTE 544 Lab 3 decision maker using EKF position estimates.

Run: python3 decisions.py --motion spiral
     python3 decisions.py --motion point
     python3 decisions.py --motion trajectory

Use with the LabThree repository's planner.py, controller.py, pid.py and
utilities.py. The localizer is spun alongside this node by one executor.
"""

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import Twist

from utilities import calculate_linear_error
from localization import localization, kalmanFilter
from planner import TRAJECTORY_PLANNER, POINT_PLANNER, SPIRAL_4TUNE, planner
from controller import controller, trajectoryController


class decision_maker(Node):
    def __init__(self, publisher_msg, publishing_topic, qos_publisher,
                 rate=10, motion_type=POINT_PLANNER):
        super().__init__("decision_maker")
        self.publisher = self.create_publisher(
            publisher_msg, publishing_topic, qos_profile=qos_publisher
        )
        publishing_period = 1.0 / rate
        self.reachThreshold = 0.10  # 10 cm; more practical with noisy estimates
        self.completed = False
        self.motion_type = motion_type
        self.warned_waiting = False

        # LAB 3: the controller now receives EKF pose, not raw odometry.
        # Keep this node alive in the executor (see main below).
        self.localizer = localization(kalmanFilter, dt=publishing_period)

        if motion_type == POINT_PLANNER:
            self.controller = controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6)
            self.planner = planner(POINT_PLANNER)
        elif motion_type == TRAJECTORY_PLANNER:
            self.controller = trajectoryController(
                klp=0.2, klv=0.5, kap=0.8, kav=0.6
            )
            self.planner = planner(TRAJECTORY_PLANNER)
        elif motion_type == SPIRAL_4TUNE:
            self.controller = None
            self.planner = None
            self.linearVelocity = 0.0
        else:
            raise ValueError("Invalid planner type: " + str(motion_type))

        self.goal = (self.planner.plan()
                     if self.motion_type != SPIRAL_4TUNE else None)
        self.create_timer(publishing_period, self.timerCallback)

    def timerCallback(self):
        # Both nodes spin in the executor. No nested spin_once() is necessary.
        if self.completed:
            self.publisher.publish(Twist())
            return

        pose = self.localizer.getPose()
        if pose is None:
            if not self.warned_waiting:
                self.get_logger().info("Waiting for synchronized /odom and /imu ...")
                self.warned_waiting = True
            return
        self.warned_waiting = False

        vel_msg = Twist()
        if self.motion_type == SPIRAL_4TUNE:
            # Conservative physical-robot command limits; adjust only in a
            # safe, clear test area. Use Ctrl+C to end the spiral.
            self.linearVelocity = min(self.linearVelocity + 0.01, 0.22)
            vel_msg.linear.x = self.linearVelocity
            vel_msg.angular.z = 0.6
            self.publisher.publish(vel_msg)
            return

        goal_position = (self.goal[-1]
                         if isinstance(self.goal, list) else self.goal)
        if calculate_linear_error(pose, goal_position) < self.reachThreshold:
            self.get_logger().info("Reached goal. Robot stopped.")
            self.publisher.publish(Twist())
            self.controller.PID_angular.logger.save_log()
            self.controller.PID_linear.logger.save_log()
            self.completed = True
            return

        velocity, yaw_rate = self.controller.vel_request(pose, self.goal, True)
        # Clamp commands in both directions (original code only capped +side).
        vel_msg.linear.x = max(-0.22, min(0.22, float(velocity)))
        vel_msg.angular.z = max(-1.0, min(1.0, float(yaw_rate)))
        self.publisher.publish(vel_msg)


def main(args=None):
    if args is None:
        args = parse_args()
    rclpy.init()
    if args.motion == "point":
        motion = POINT_PLANNER
    elif args.motion == "trajectory":
        motion = TRAJECTORY_PLANNER
    elif args.motion == "spiral":
        motion = SPIRAL_4TUNE
    else:
        raise ValueError("Invalid motion: " + str(args.motion))

    dm = decision_maker(Twist, "/cmd_vel", 10, motion_type=motion)
    executor = SingleThreadedExecutor()
    executor.add_node(dm)
    executor.add_node(dm.localizer)
    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit, ExternalShutdownException):
        pass
    finally:
        # A ROS signal handler may already have invalidated the context.
        # Publish a stop while communications are still available, then clean up.
        if rclpy.ok():
            dm.publisher.publish(Twist())
        executor.shutdown()
        dm.localizer.destroy_node()
        dm.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Lab 3 EKF motion demo")
    parser.add_argument("--motion", choices=["point", "trajectory", "spiral"],
                        default="spiral")
    # Leave ROS parameters/remappings for rclpy.init(), including use_sim_time.
    return parser.parse_args(remove_ros_args(args=argv)[1:])


if __name__ == "__main__":
    main()
