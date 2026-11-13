"""Extended Kalman Filter for MTE 544 Lab 3.

State (6): [x, y, theta, omega, v, a]
    x, y       world-frame robot position (m)
    theta      world-frame heading (rad)
    omega      yaw rate (rad/s)
    v          forward speed in the robot's x direction (m/s)
    a          forward acceleration in the robot's x direction (m/s^2)

Measurements (4): [v_odom, omega_odom, ax_imu, ay_imu]
    ax_imu = a           (forward / tangential acceleration)
    ay_imu = omega * v   (lateral / centripetal acceleration)
Assumes planar movement on a level surface, with odometry and IMU axes
aligned to the robot's base frame. Position is estimated by integration;
position is NOT measured directly in the correction step.
"""

import numpy as np


class kalman_filter:
    """Six-state Extended Kalman Filter using the course's original API."""

    def __init__(self, P, Q, R, x, dt):
        # P: uncertainty in the current six-state estimate (6 x 6).
        # Q: uncertainty added by the motion model each prediction (6 x 6).
        # R: uncertainty in the four measurements (4 x 4).
        self.P = np.asarray(P, dtype=float).reshape(6, 6).copy()
        self.Q = np.asarray(Q, dtype=float).reshape(6, 6).copy()
        self.R = np.asarray(R, dtype=float).reshape(4, 4).copy()
        self.x = np.asarray(x, dtype=float).reshape(6).copy()
        self.dt = float(dt)

        # A = df/dx: Jacobian of nonlinear motion model f(x).
        # C = dh/dx: Jacobian of nonlinear measurement model h(x).
        self.A = np.eye(6)
        self.C = np.zeros((4, 6))

    def predict(self):
        """Predict the state and its uncertainty before reading a new sensor."""
        # Evaluate Jacobian at the OLD state, which is where f is linearized.
        self.A = self.jacobian_A()
        self.motion_model()  # x_pred = f(x_previous)
        self.P = self.A @ self.P @ self.A.T + self.Q  # P_pred
        self.P = 0.5 * (self.P + self.P.T)  # avoid numerical asymmetry

    def update(self, z):
        """Correct the prediction using odometry and IMU observations."""
        z = np.asarray(z, dtype=float).reshape(4)

        # Linearize h at the PREDICTED state, then compare z to h(x_pred).
        self.C = self.jacobian_H()
        innovation = z - self.measurement_model()
        S = self.C @ self.P @ self.C.T + self.R  # innovation covariance

        # Kalman gain K = P C^T S^-1; solve avoids explicitly inverting S.
        K = np.linalg.solve(S, self.C @ self.P).T
        self.x = self.x + K @ innovation
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

        # Joseph form is equivalent to (I - K C) P but better behaved
        # numerically (keeps P symmetric / positive semidefinite).
        identity = np.eye(6)
        correction = identity - K @ self.C
        self.P = correction @ self.P @ correction.T + K @ self.R @ K.T
        self.P = 0.5 * (self.P + self.P.T)

    def measurement_model(self):
        """Predict [forward speed, yaw rate, local ax, local ay]."""
        x, y, th, w, v, vdot = self.x
        return np.array([
            v,        # odometry linear.x
            w,        # odometry angular.z
            vdot,     # IMU linear_acceleration.x, tangential
            v * w,    # IMU linear_acceleration.y, centripetal
        ], dtype=float)

    def motion_model(self):
        """Nonlinear robot motion model f(x), assuming constant w and a."""
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        self.x = np.array([
            x + v * np.cos(th) * dt,
            y + v * np.sin(th) * dt,
            th + w * dt,
            w,
            v + vdot * dt,
            vdot,
        ], dtype=float)
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

    def jacobian_A(self):
        """Jacobian df/dx: how each next state depends on current states."""
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        return np.array([
            [1, 0, -v * np.sin(th) * dt, 0, np.cos(th) * dt, 0],
            [0, 1,  v * np.cos(th) * dt, 0, np.sin(th) * dt, 0],
            [0, 0, 1, dt, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, dt],
            [0, 0, 0, 0, 0, 1],
        ], dtype=float)

    def jacobian_H(self):
        """Jacobian dh/dx: which states affect each measured quantity."""
        x, y, th, w, v, vdot = self.x
        return np.array([
            [0, 0, 0, 0, 1, 0],  # h_1 = v
            [0, 0, 0, 1, 0, 0],  # h_2 = w
            [0, 0, 0, 0, 0, 1],  # h_3 = vdot
            [0, 0, 0, v, w, 0],  # h_4 = w * v
        ], dtype=float)

    def get_states(self):
        """Return [x, y, theta, w, v, vdot] as a one-dimensional array."""
        return self.x.copy()
