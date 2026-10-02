#!/usr/bin/env python3
"""Plot a CSV recording produced by the desktop simulator."""

import argparse
import csv

import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv")
    parser.add_argument("--output", help="write an image instead of opening a window")
    args = parser.parse_args()
    with open(args.csv, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise SystemExit("recording contains no samples")
    t = [float(row["time_us"]) / 1e6 for row in rows]
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8))
    axes[0].plot(t, [float(r["spin_rad_s"]) for r in rows], label="actual")
    axes[0].plot(t, [float(r["estimated_spin_rad_s"]) for r in rows], label="estimated")
    axes[0].set_ylabel("spin [rad/s]"); axes[0].legend(); axes[0].grid(alpha=.25)
    axes[1].plot(t, [float(r["wheel_a_command"]) for r in rows], label="wheel A")
    axes[1].plot(t, [float(r["wheel_b_command"]) for r in rows], label="wheel B")
    axes[1].set_ylabel("command"); axes[1].legend(); axes[1].grid(alpha=.25)
    axes[2].plot([float(r["x_m"]) for r in rows], [float(r["y_m"]) for r in rows])
    axes[2].set_xlabel("x [m]"); axes[2].set_ylabel("y [m]"); axes[2].axis("equal"); axes[2].grid(alpha=.25)
    fig.tight_layout()
    if args.output: fig.savefig(args.output, dpi=160)
    else: plt.show()


if __name__ == "__main__":
    main()
