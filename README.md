# MTE 544 — Lab 3 EKF localization

This repository includes the completed EKF code and the starter modules needed
to run it. No separate starter checkout or file copying is required. Run the ROS
program on the lab's Ubuntu/ROS 2 environment with the TurtleBot simulator or
robot already running. Plain Windows Python does not provide ROS dependencies.

## Files

- `kalman_filter.py`: six-state EKF, motion/measurement models and Jacobians.
- `localization.py`: synchronized odometry/IMU fusion and CSV logging.
- `decisions.py`: spiral, point and optional trajectory motion using EKF pose.
- `planner.py`, `controller.py`, `pid.py`, `utilities.py`: official starter modules.
- `plot_errors.py`: original starter plotting helper.
- `plot_ekf.py`: measured versus estimated path, velocity and acceleration plots.
- `check_setup.py`: dependency and local-module import check; sends no commands.
- `test_ekf.py`: numerical EKF sanity test, which does not require ROS.

The five starter modules are copied unchanged from the official
[LabThree starter](https://github.com/UW-MTE544/MTE544_student/tree/637d4cee57040cc74715760af4ea902bcae9c8d6).
The original MIT license is included in `LICENSE`.

## Set up Ubuntu / ROS 2

Use the ROS distribution configured on the lab desktop. For an existing ROS 2
Humble installation on Ubuntu 22.04, install the Python and message packages if
they are missing:

```bash
sudo apt update
sudo apt install python3-numpy python3-matplotlib \
  ros-humble-rclpy ros-humble-message-filters \
  ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-sensor-msgs
source /opt/ros/humble/setup.bash
```

If the lab provides its own environment setup, source that as instructed instead.
Use the matching distribution name for other ROS installations. These commands
assume ROS itself is already installed; they do not install or launch Gazebo or
the TurtleBot simulation. Prefer the system `python3` used by ROS over a Conda
environment or an isolated virtual environment.

```bash
git clone --branch LabThree --single-branch https://github.com/JordanT-MWang/MTE544Labs.git
cd MTE544Labs
python3 check_setup.py
python3 test_ekf.py
```

The import check must pass before running motion. `requirements.txt` lists the
two non-ROS Python dependencies for math/plotting; `pip install` alone does not
install the ROS runtime or message packages.

## Run with the simulator or robot

Start the simulation or connect to the physical robot following the course
instructions. In each terminal, source the ROS environment and set the lab's
required `ROS_DOMAIN_ID`. Check the available sensor topics:

```bash
ros2 topic list
ros2 topic info -v /odom
ros2 topic info -v /imu
```

The defaults are `/odom`, `/imu`, and `/cmd_vel`. The IMU x/y accelerations must
be aligned with the robot base frame on a level surface, and odometry/IMU
timestamps must use the same clock. The program waits for synchronized sensor
messages before starting motion. Check the IMU topic name, QoS and timestamps if
it keeps waiting.

For simulation with a published `/clock`, use:

```bash
python3 decisions.py --motion spiral --ros-args -p use_sim_time:=true
# Ctrl+C ends the spiral. Save its log before starting the next run.
mv robot_pose.csv robot_pose_spiral.csv
python3 decisions.py --motion point --ros-args -p use_sim_time:=true
# At the goal the node publishes zero velocity; use Ctrl+C to exit.
mv robot_pose.csv robot_pose_point.csv
python3 plot_ekf.py robot_pose_spiral.csv
python3 plot_ekf.py robot_pose_point.csv
```

For a physical robot, omit `--ros-args -p use_sim_time:=true`. Topic remapping is
supported, for example `--ros-args -r /imu:=/your_imu_topic` (combine with the
simulation-time parameter when appropriate). Optional trajectory motion uses
`--motion trajectory`.

The starter point goal is `(-1, -1)` in `planner.py`; the arrival threshold is
0.10 m. Commands are limited to 0.22 m/s and 1.0 rad/s; the spiral uses 0.6 rad/s.
The program attempts to publish a zero command on exit while ROS communications
are active. Follow the course's robot stopping procedure as well.

Each run overwrites `robot_pose.csv`; point/trajectory runs also overwrite
`linear.csv` and `angular.csv`. Save all logs you need before another run. Stop
and restart the controller whenever resetting simulation time or robot odometry.
Duplicate and backward sensor timestamps are ignored.

## EKF equations

State: `x = [x, y, theta, w, v, vdot]`.
Measurements: `z = [v_odom, w_odom, imu_ax, imu_ay]`.

```text
x_next     = x + v cos(theta) dt
y_next     = y + v sin(theta) dt
theta_next = theta + w dt
w_next     = w
v_next     = v + vdot dt
vdot_next  = vdot

h(x) = [v, w, vdot, v*w]
```

Prediction uses `P = A P A^T + Q`; correction uses `S = H P H^T + R`,
`K = P H^T S^-1` and the measurement residual. Covariance correction uses the
Joseph form. Initial `P=0.5 I6`, `Q=0.5 I6`, and `R=0.5 I4`.

For the required spiral experiments, edit Q/R in `localization.py`: increase and
decrease Q while holding R fixed, then increase and decrease R while holding Q
fixed (four combinations). Save each CSV and its plots. Finish with point
controller tuning. Submit the commented prelab code through LEARN at least
24 hours before the group's section, following the official lab instructions.

## Validation and limits

The supplied numerical test checks 100 prediction/update cycles and covariance
validity. Local checks also exercise the Jacobians, synthetic fusion and CSV
plotting, and bundled controller interfaces using ideal planar kinematics.
Those checks use ROS interface stubs and are not a ROS/Gazebo demonstration.

Actual ROS transport, simulation and physical-robot behavior still require
verification on the lab system. The EKF starts from the first odometry pose and
then integrates velocity; the assigned measurement vector has no absolute
position observation, so position can drift over time.
