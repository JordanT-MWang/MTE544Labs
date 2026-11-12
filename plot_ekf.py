"""Plot MTE544 Lab 3 odometry/IMU readings vs. EKF estimates.

Usage: python3 plot_ekf.py robot_pose.csv
Creates: robot_pose_path.png, robot_pose_velocity.png, robot_pose_acceleration.png
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def read_log(filename):
    # Course Logger writes a trailing comma and space on each line.
    with open(filename, newline="") as file:
        rows = list(csv.DictReader(file, skipinitialspace=True))
    if not rows:
        raise ValueError(f"No samples in {filename}")
    fields = [
        "odom_x", "odom_y", "odom_v", "odom_w", "imu_ax", "imu_ay",
        "kf_x", "kf_y", "kf_theta", "kf_w", "kf_v", "kf_ax", "kf_ay",
        "stamp",
    ]
    return {key: np.array([float(row[key]) for row in rows]) for key in fields}


def format_axes(ax, title, xlabel, ylabel):
    ax.set(title=title, xlabel=xlabel, ylabel=ylabel)
    ax.legend()
    ax.grid(True, alpha=0.4)


def main(filename):
    data = read_log(filename)
    prefix = Path(filename).with_suffix("")
    t = data["stamp"] - data["stamp"][0]

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(data["odom_x"], data["odom_y"], "-", label="Odometry path")
    ax.plot(data["kf_x"], data["kf_y"], "--", label="EKF path")
    ax.set_aspect("equal", adjustable="datalim")
    format_axes(ax, "Robot trajectory", "x (m)", "y (m)")
    fig.tight_layout()
    fig.savefig(f"{prefix}_path.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].plot(t, data["odom_v"], "-", label="Measured v")
    axes[0].plot(t, data["kf_v"], "--", label="Estimated v")
    format_axes(axes[0], "Forward speed", "Time (s)", "v (m/s)")
    axes[1].plot(t, data["odom_w"], "-", label="Measured yaw rate")
    axes[1].plot(t, data["kf_w"], "--", label="Estimated yaw rate")
    format_axes(axes[1], "Angular speed", "Time (s)", "ω (rad/s)")
    fig.tight_layout()
    fig.savefig(f"{prefix}_velocity.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    axes[0].plot(t, data["imu_ax"], "-", label="Measured ax")
    axes[0].plot(t, data["kf_ax"], "--", label="Estimated ax")
    format_axes(axes[0], "Forward acceleration", "Time (s)", "ax (m/s²)")
    axes[1].plot(t, data["imu_ay"], "-", label="Measured ay")
    axes[1].plot(t, data["kf_ay"], "--", label="Estimated ay")
    format_axes(axes[1], "Lateral acceleration", "Time (s)", "ay (m/s²)")
    fig.tight_layout()
    fig.savefig(f"{prefix}_acceleration.png", dpi=180)
    plt.close(fig)
    print(f"Saved three plots with prefix: {prefix}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file", nargs="?", default="robot_pose.csv")
    args = parser.parse_args()
    main(args.file)
