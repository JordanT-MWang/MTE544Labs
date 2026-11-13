import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors(filename):
    headers, values = FileReader(filename).read_file()
    idx = {name: i for i, name in enumerate(headers)}

    first_stamp = values[0][idx["stamp"]]
    time_list = [val[idx["stamp"]] - first_stamp for val in values]

    fig, axes = plt.subplots(4, 1, figsize=(12, 12))

    # 1. 2D Trajectory (X vs Y)
    axes[0].plot(
        [val[idx["odom_x"]] for val in values],
        [val[idx["odom_y"]] for val in values],
        "--", label="Odometry (x, y)"
    )
    axes[0].plot(
        [val[idx["kf_x"]] for val in values],
        [val[idx["kf_y"]] for val in values],
        "-", label="EKF Estimate (kf_x, kf_y)"
    )
    axes[0].set_title("2D Trajectory (X vs Y)")
    axes[0].set_xlabel("X [m]")
    axes[0].set_ylabel("Y [m]")
    axes[0].axis("equal")
    axes[0].legend()
    axes[0].grid(True)

    # 2. X Position vs Time (EKF vs Odometry)
    axes[1].plot(
        time_list,
        [val[idx["odom_x"]] for val in values],
        "--", label="odom_x [m]"
    )
    axes[1].plot(
        time_list,
        [val[idx["kf_x"]] for val in values],
        "-", label="kf_x [m]"
    )
    axes[1].set_title("X Position vs Time")
    axes[1].set_xlabel("Time [s]")
    axes[1].set_ylabel("X [m]")
    axes[1].legend()
    axes[1].grid(True)

    # 3. Y Position vs Time (EKF vs Odometry)
    axes[2].plot(
        time_list,
        [val[idx["odom_y"]] for val in values],
        "--", label="odom_y [m]"
    )
    axes[2].plot(
        time_list,
        [val[idx["kf_y"]] for val in values],
        "-", label="kf_y [m]"
    )
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
    
    





import argparse

if __name__=="__main__":

    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('--files', nargs='+', required=True, help='List of files to process')
    
    args = parser.parse_args()
    
    print("plotting the files", args.files)

    filenames=args.files
    for filename in filenames:
        plot_errors(filename)


