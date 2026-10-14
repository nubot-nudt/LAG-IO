#!/usr/bin/env python3
"""Plot an estimated trajectory and optional ground truth, without error metrics."""

import argparse
import sys
from pathlib import Path

import numpy as np


def load_trajectory(path, columns, time_column=None, merge_duplicate_times=False):
    data = np.loadtxt(path, ndmin=2)
    required = columns + ([] if time_column is None else [time_column])
    if data.shape[0] < 2 or data.shape[1] <= max(required):
        raise ValueError(f"{path}: expected at least two rows and columns {columns}")
    positions = data[:, columns]
    if not np.isfinite(positions).all():
        raise ValueError(f"{path}: positions contain non-finite values")
    times = None if time_column is None else data[:, time_column]
    if times is not None:
        if not np.isfinite(times).all() or (np.diff(times) < 0).any():
            raise ValueError(f"{path}: timestamps must be finite and nondecreasing")
        unique, first, counts = np.unique(times, return_index=True, return_counts=True)
        if len(unique) != len(times):
            if not merge_duplicate_times:
                raise ValueError(f"{path}: estimate timestamps must be strictly increasing")
            # Exported GT may round adjacent timestamps to the same value.
            positions = np.add.reduceat(positions, first, axis=0) / counts[:, None]
            print(f"{path}: merged {len(times) - len(unique)} duplicate GT timestamps", file=sys.stderr)
            times = unique
        if len(times) < 2:
            raise ValueError(f"{path}: at least two distinct timestamps are required")
    return positions, times


def synchronize(estimate, estimate_times, ground_truth, gt_times):
    # Use only estimated poses bracketed by GT; never extrapolate beyond its coverage.
    common = (estimate_times >= gt_times[0]) & (estimate_times <= gt_times[-1])
    if np.count_nonzero(common) < 2:
        raise ValueError("Estimate and GT need at least two estimated poses in their common interval")
    times = estimate_times[common]
    reference = np.column_stack([np.interp(times, gt_times, ground_truth[:, axis])
                                 for axis in range(3)])
    return estimate[common], reference


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("estimate", type=Path)
    parser.add_argument("--gt", type=Path, help="Whitespace-delimited ground truth")
    parser.add_argument("--estimate-columns", type=int, nargs=3, default=[1, 2, 3],
                        metavar=("X", "Y", "Z"), help="Zero-based position columns")
    parser.add_argument("--gt-columns", type=int, nargs=3, default=[1, 2, 3],
                        metavar=("X", "Y", "Z"))
    parser.add_argument("--sync", action="store_true",
                        help="Crop to common time coverage and interpolate GT to estimate timestamps")
    parser.add_argument("--estimate-time-column", type=int, default=0)
    parser.add_argument("--gt-time-column", type=int, default=0)
    parser.add_argument("--swap-xy", action="store_true",
                        help="Display local y horizontally and local x vertically, as in the dataset plots")
    parser.add_argument("--title", help="Optional figure title")
    parser.add_argument("--origin", action="store_true",
                        help="Translate each trajectory to its own first position")
    parser.add_argument("--3d", dest="three_dimensional", action="store_true")
    parser.add_argument("--output", type=Path, help="Save the figure without opening a window")
    args = parser.parse_args()
    if args.sync and not args.gt:
        parser.error("--sync requires --gt")
    if min(args.estimate_columns + args.gt_columns +
           [args.estimate_time_column, args.gt_time_column]) < 0:
        parser.error("Column indices must be non-negative")
    import matplotlib
    if args.output:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    try:
        estimate, estimate_times = load_trajectory(
            args.estimate, args.estimate_columns, args.estimate_time_column if args.sync else None)
        reference = None
        if args.gt:
            reference, gt_times = load_trajectory(
                args.gt, args.gt_columns, args.gt_time_column if args.sync else None,
                merge_duplicate_times=True)
            if args.sync:
                estimate, reference = synchronize(estimate, estimate_times, reference, gt_times)
        trajectories = [] if reference is None else [("Ground truth", reference)]
        trajectories.append(("LAG-IO", estimate))
    except (OSError, ValueError) as error:
        parser.error(str(error))
    fig = plt.figure(figsize=(8, 6), constrained_layout=True)
    ax = fig.add_subplot(111, projection="3d" if args.three_dimensional else None)
    plotted = []
    for label, positions in trajectories:
        if args.origin:
            positions = positions - positions[0]
        if args.swap_xy:
            positions = positions[:, [1, 0, 2]]
        plotted.append(positions)
        style = dict(label=label, color="black" if label == "Ground truth" else "#d62728",
                     linewidth=1.5, linestyle="--" if label == "Ground truth" else "-")
        if args.three_dimensional:
            ax.plot(*positions.T, **style)
        else:
            ax.plot(*positions[:, :2].T, **style)
            if label == "LAG-IO":
                ax.scatter(*positions[0, :2], marker="s", s=30, color="#2ca02c", label="Start", zorder=4)
                ax.scatter(*positions[-1, :2], marker="x", s=35, color="#d62728", label="End", zorder=4)
    ax.set_xlabel("Local y [m]" if args.swap_xy else "x [m]")
    ax.set_ylabel("Local x [m]" if args.swap_xy else "y [m]")
    if args.title:
        ax.set_title(args.title)
    if args.three_dimensional:
        ax.set_zlabel("z [m]")
        ax.set_box_aspect(np.maximum(np.ptp(np.vstack(plotted), axis=0), 1e-6))
    else:
        ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend()
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.output, dpi=180, bbox_inches="tight")
    else:
        plt.show()
    plt.close(fig)


if __name__ == "__main__":
    main()
