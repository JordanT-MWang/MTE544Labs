# MTE 544 — Lab 3 pre-lab code

This folder contains **completed replacements** for the starter files in
https://github.com/UW-MTE544/MTE544_student/tree/LabThree . It is a **patch**, not
an entire copy of the course repository. Keep the other starter files
(`planner.py`, `controller.py`, `pid.py`, `utilities.py`, etc.) unchanged.

## Included files

- `kalman_filter.py` — 6-state Extended Kalman Filter; f, h, and both Jacobians.
- `localization.py` — synchronizes `/odom` and `/imu`, runs EKF and logs CSV.
- `decisions.py` — integrates the EKF localizer with point and spiral motion.
- `test_ekf.py` — quick numerical sanity check, without ROS 2.
- `plot_ekf.py` — creates trajectory, velocity and acceleration plots from CSV.

## Copy into the lab starter repository

On an Ubuntu ROS 2 system with the repository cloned:

```bash
git clone --branch LabThree --single-branch https://github.com/UW-MTE544/MTE544_student.git
cd MTE544_student
# Copy kalman_filter.py, localization.py, and decisions.py from this zip here,
# overwriting the starter versions. You can also copy both helper scripts.
```

The three replacements import the other *unchanged* source files from LabThree.

Start the Gazebo/TurtleBot simulation as instructed in the lab. In a sourced
ROS 2 terminal, run:

```bash
python3 decisions.py --motion spiral
# Ctrl+C to stop; save robot_pose.csv before starting another motion.
mv robot_pose.csv robot_pose_spiral.csv
python3 decisions.py --motion point
mv robot_pose.csv robot_pose_point.csv
python3 plot_ekf.py robot_pose_spiral.csv
python3 plot_ekf.py robot_pose_point.csv
```

The code defaults to ROS topics `/odom`, `/imu`, and `/cmd_vel`. Verify topic
names with `ros2 topic list` and their QoS settings with `ros2 topic info -v`.
If your lab robot uses a different IMU name, update `imu_topic` in the
`localization(...)` invocation in `decisions.py` or the default in
`localization.py`. The code expects the IMU's x/y acceleration axes to be
aligned to the robot base frame, with a level robot.

## EKF equations used

State: `x = [x, y, theta, w, v, vdot]`.
Measurements: `z = [v_odom, w_odom, imu_ax, imu_ay]`.

Motion model, time step `dt`:

```text
x_next = x + v cos(theta) dt
 y_next = y + v sin(theta) dt
theta_next = theta + w dt
w_next = w
v_next = v + vdot dt
vdot_next = vdot
```

Measurement model:

```text
h(x) = [v, w, vdot, v*w]
```

The filter predicts `P = A P A^T + Q`, then corrects using `S = H P H^T + R`,
`K = P H^T S^-1` and the measurement residual. The covariance correction uses
the numerically stable Joseph form. Initial `P=0.5 I6`, `Q=0.5 I6`, and
`R=0.5 I4`, matching the lab's recommended starting noise scales.

## For in-lab tuning and the written report

Edit the `Q` and `R` lines in `localization.py` to obtain four spiral runs
(e.g., two variations in Q at fixed R and two variations in R at fixed Q).
Keep the individual CSV files and plot them. Then run the point controller
using your chosen values. Do not overwrite the CSV between runs.

## Validation / limits

- The Python files pass syntax compilation.
- `test_ekf.py` passes 100 predict/update cycles and covariance checks.
- The Jacobians have been checked numerically and the ROS callback has been
  tested using simulated message objects, including CSV output.
- **Not yet tested on your actual ROS 2 / Gazebo / physical robot**. Verify
  subscriptions, topic names, IMU axes, motion safety, and EKF behaviour there.
- EKF `x, y` result starts at the first odometry reading and thereafter
  integrates estimated velocity. There is no direct absolute `x, y` measurement
  in the assigned measurement vector, so position can drift over time.
