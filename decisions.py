
import argparse
import sys

from rclpy import init, ok, shutdown
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist

from utilities import calculate_linear_error
from localization import localization, rawSensor
from planner import TRAJECTORY_PLANNER, POINT_PLANNER, planner
from controller import controller, trajectoryController


GOAL_TOLERANCE_M = 0.10  # Stop when within 10 cm of the goal
POINT_GOAL = [-0.5, -0.5]  # About 1.1 m from the last logged simulation pose


class decision_maker(Node):
    def __init__(self, publisher_msg, publishing_topic, qos_publisher,
                 goalPoint, rate=10, motion_type=POINT_PLANNER, robot="tb3"):
        super().__init__("decision_maker")

        # Part 4: Publish linear and angular velocities on /cmd_vel.
        self.publisher = self.create_publisher(
            publisher_msg, publishing_topic, qos_publisher
        )

        # Select the appropriate controller and planner.
        if motion_type == POINT_PLANNER:
            self.controller = controller(klp=0.2, klv=0.5, kap=0.8, kav=0.6,
                                         robot=robot)
            self.planner = planner(POINT_PLANNER)
        elif motion_type == TRAJECTORY_PLANNER:
            self.controller = trajectoryController(
                klp=0.2, klv=0.5, kap=0.8, kav=0.6, robot=robot
            )
            self.planner = planner(TRAJECTORY_PLANNER)
        else:
            raise ValueError("Unknown planner type")

        # Part 3: Read the robot position from /odom via the localization node.
        self.localizer = localization(rawSensor)
        self.goal = self.planner.plan(goalPoint)

        if self.goal is None or (isinstance(self.goal, list) and not self.goal):
            raise ValueError(
                "No goal returned by planner.py. "
                "Complete trajectory_planner() before using trajectory mode."
            )

        self.goal_reached = False
        self.timer = self.create_timer(1.0 / rate, self.timerCallback)

    def timerCallback(self):
        # Both ROS nodes are handled by the same executor in main().
        pose = self.localizer.getPose()
        if pose is None:
            self.get_logger().info("Waiting for /odom messages...", once=True)
            return

        # Point planner returns a tuple (x, y); trajectory planner returns
        # a list of [x, y] points, with the final point as the destination.
        final_goal = self.goal[-1] if isinstance(self.goal, list) else self.goal

        if calculate_linear_error(pose, final_goal) < GOAL_TOLERANCE_M:
            # Record the final errors without requesting motion.
            self.controller.vel_request(pose, self.goal, False)
            self.goal_reached = True
            self.publisher.publish(Twist())  # Stop the robot.
            self.get_logger().info("Reached goal")
            self.timer.cancel()
            return

        # Part 4: Calculate the commanded velocities from the controller.
        velocity, yaw_rate = self.controller.vel_request(pose, self.goal, True)

        # Publish /cmd_vel; limits are handled in controller.py.
        vel_msg = Twist()
        vel_msg.linear.x = float(velocity)
        vel_msg.angular.z = float(yaw_rate)
        self.publisher.publish(vel_msg)


def main(args=None):
    if args is None:
        parser = argparse.ArgumentParser(description="MTE 544 Lab 2")
        parser.add_argument("--motion", choices=["point", "trajectory"],
                            default="point")
        parser.add_argument("--robot", choices=["tb3", "tb4"], default="tb3",
                            help="Select velocity limits (default: tb3 simulation)")
        parser.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"),
                            default=POINT_GOAL,
                            help="Point goal in odometry coordinates, in metres")
        args = parser.parse_args()

    init()

    cmd_vel_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10
    )

    if args.motion == "point":
        motion_type = POINT_PLANNER
    elif args.motion == "trajectory":
        motion_type = TRAJECTORY_PLANNER
    else:
        print("Invalid motion type")
        shutdown()
        return

    # Create decision-maker and localization
    dm = decision_maker(
        Twist,
        "/cmd_vel",
        cmd_vel_qos,
        goalPoint=args.goal,
        motion_type=motion_type,
        robot=args.robot
    )

    # Run both ROS 2 nodes
    executor = SingleThreadedExecutor()
    executor.add_node(dm)
    executor.add_node(dm.localizer)

    print("Decision-maker started!")
    print("Robot:", args.robot)
    print("Goal:", dm.goal)
    print("Waiting for robot position...")

    try:
        while ok() and not dm.goal_reached:
            executor.spin_once(timeout_sec=0.1)

        if dm.goal_reached:
            print("Robot reached its target!")

    except KeyboardInterrupt:
        print("Controller stopped.")

    finally:
        if ok():
            dm.publisher.publish(Twist())

        dm.controller.PID_linear.logger.save_log()
        dm.controller.PID_angular.logger.save_log()

        executor.shutdown()
        dm.localizer.destroy_node()
        dm.destroy_node()

        if ok():
            shutdown()


if __name__ == "__main__":
    main()
