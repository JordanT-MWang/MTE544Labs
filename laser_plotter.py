"""
Clean and plot one LaserScan row from a laser_content_*.csv file.

Usage:
    python3 laser_plotter.py --file laser_content_circle.csv
    python3 laser_plotter.py --file laser_content_spiral.csv --row 50
    python3 laser_plotter.py --file laser_content_line.csv --save line_scan.png
"""
import argparse
import re

import numpy as np
import matplotlib.pyplot as plt

# Each data line looks like:  2.78;inf;2.88;...;3.1,0.0174930,2513218000000
# i.e. three comma-separated columns, with the ranges joined by semicolons.


def load_laser_csv(path):
    """Return a list of (ranges, angle_increment, stamp_ns), one per scan."""
    scans = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("ranges"):   # blank or header
                continue
            ranges_cell, inc, stamp = line.split(",")[:3]
            # float("inf") and float("nan") parse fine; skip empty pieces
            ranges = np.array([float(x) for x in ranges_cell.split(";") if x])
            scans.append((ranges, float(inc), int(stamp)))
    return scans


def clean_scan(ranges, angle_increment, angle_min=0.0,
               range_min=0.12, range_max=3.5):
    """
    Build the angle for every beam, then drop inf / NaN / out-of-range beams.
    Angles and ranges are filtered with the same mask so they stay aligned.
    Defaults match the TurtleBot3 LDS-01 (angle_min = 0, 0.12 m to 3.5 m).
    """
    angles = angle_min + np.arange(len(ranges)) * angle_increment
    mask = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    return ranges[mask], angles[mask]


def polar_to_cartesian(ranges, angles):
    return ranges * np.cos(angles), ranges * np.sin(angles)


def main():
    parser = argparse.ArgumentParser(description="Plot one cleaned laser scan")
    parser.add_argument("--file", required=True, help="laser_content_*.csv")
    parser.add_argument("--row", type=int, default=None,
                        help="scan index to plot (default: middle scan)")
    parser.add_argument("--angle_min", type=float, default=0.0,
                        help="angle of the first beam in rad (default 0)")
    parser.add_argument("--save", default=None, help="save figure to this path")
    args = parser.parse_args()

    scans = load_laser_csv(args.file)
    if not scans:
        raise SystemExit(f"No scans found in {args.file}")

    row = args.row if args.row is not None else len(scans) // 2
    if not 0 <= row < len(scans):
        raise SystemExit(f"--row must be between 0 and {len(scans) - 1}")

    ranges, inc, stamp = scans[row]
    r, a = clean_scan(ranges, inc, angle_min=args.angle_min)
    print(f"{len(scans)} scans in file, plotting row {row}")
    print(f"{len(ranges)} beams, {len(r)} valid after cleaning "
          f"({len(ranges) - len(r)} removed)")
    if len(r) == 0:
        raise SystemExit("No valid beams in this row, try a different --row")

    x, y = polar_to_cartesian(r, a)

    fig, ax = plt.subplots(figsize=(6, 6))
    ax.scatter(x, y, s=8, label="laser points")
    ax.scatter([0], [0], c="red", marker="^", s=80, label="robot")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Laser scan (row {row}) - {args.file}")
    ax.grid(True)
    ax.legend()

    if args.save:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved to {args.save}")
    plt.show()


if __name__ == "__main__":
    main()