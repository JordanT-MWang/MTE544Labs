import numpy as np
import matplotlib.pyplot as plt
from utilities import FileReader
import argparse
def compute_ideal_spiral(time_list, odom_x, odom_y, theta_0, v_step=0.01, v_max=1.0, omega_cmd=1.0, rate=10.0):
    """Reconstruct the ideal spiral directly from control frequency, dv, and constant omega."""
    x = odom_x[0]
    y = odom_y[0]
    th = theta_0
    v = 0.0

    # Control timer period (e.g., 10 Hz -> 0.1 s per step)
    control_dt = 1.0 / rate
    next_control_t = 0.0

    ideal_x, ideal_y = [x], [y]
    for i in range(1, len(time_list)):
        dt = time_list[i] - time_list[i - 1]
        t = time_list[i]

        # Step v at the exact control frequency (matching decisions.py's 10 Hz timer)
        while t >= next_control_t:
            if v < v_max:
                v = min(v_max, v + v_step)
            next_control_t += control_dt

        x += v * np.cos(th) * dt
        y += v * np.sin(th) * dt
        th += omega_cmd * dt

        ideal_x.append(x)
        ideal_y.append(y)

    return ideal_x, ideal_y

def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    idx = {name: i for i, name in enumerate(headers)}

    first_stamp = values[0][idx["stamp"]]
    time_list = [val[idx["stamp"]] - first_stamp for val in values]

    odom_x = [val[idx["odom_x"]] for val in values]
    odom_y = [val[idx["odom_y"]] for val in values]
    odom_v = [val[idx["odom_v"]] for val in values]
    odom_w = [val[idx["odom_w"]] for val in values]

    # Reconstruct ideal spiral aligned to your actual run
    start_idx = next((i for i, v in enumerate(odom_v) if v > 0.01), 0)
    theta_0 = values[start_idx][idx["kf_theta"]]
    ideal_x, ideal_y = compute_ideal_spiral(
    time_list, odom_x, odom_y, theta_0,
    v_step=0.01, v_max=1.0, omega_cmd=1.0, rate=10.0
)
    fig, axes = plt.subplots(4, 1, figsize=(12, 12))

    # 1. 2D Trajectory (X vs Y)
    axes[0].plot(ideal_x, ideal_y, "k:", linewidth=2, zorder=5, label="Ideal Spiral (Commanded)")
    axes[0].plot(odom_x, odom_y, "--", label="Odometry (x, y)")
    axes[0].plot(
        [val[idx["kf_x"]] for val in values],
        [val[idx["kf_y"]] for val in values],
        "-", label="EKF Estimate (kf_x, kf_y)"
    )
    axes[0].set_title("2D Trajectory (X vs Y)")
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].set_aspect("equal", adjustable="datalim")
    axes[0].legend()
    axes[0].grid(True)

    # 2. X Position vs Time
    axes[1].plot(time_list, ideal_x, "k:", linewidth=2, label="ideal_x [m]")
    axes[1].plot(time_list, odom_x, "--", label="odom_x [m]")
    axes[1].plot(time_list, [val[idx["kf_x"]] for val in values], "-", label="kf_x [m]")
    axes[1].set_title("X Position vs Time")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("X [m]")
    axes[1].legend()
    axes[1].grid(True)

    # 3. Y Position vs Time
    axes[2].plot(time_list, ideal_y, "k:", linewidth=2, label="ideal_y [m]")
    axes[2].plot(time_list, odom_y, "--", label="odom_y [m]")
    axes[2].plot(time_list, [val[idx["kf_y"]] for val in values], "-", label="kf_y [m]")
    axes[2].set_title("Y Position vs Time")
    axes[2].set_xlabel("Time [s]")
    axes[2].set_ylabel("Y [m]")
    axes[2].legend()
    axes[2].grid(True)

    # 4. Heading, Linear Velocity, and Yaw Rate
    for col in ["kf_theta", "kf_v", "odom_v", "kf_w", "odom_w"]:
        if col in idx:
            axes[3].plot(time_list, [val[idx[col]] for val in values], label=col)
    axes[3].set_title("Heading, Linear Velocity, and Yaw Rate")
    axes[3].set_xlabel("Time [s]")
    axes[3].legend()
    axes[3].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process some files.")
    parser.add_argument("--files", nargs="+", required=True, help="List of files to process")
    args = parser.parse_args()

    for filename in args.files:
        plot_errors(filename)