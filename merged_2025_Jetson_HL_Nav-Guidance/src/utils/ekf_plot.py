#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Plot — visualise sensor fusion results and ground truth overlay.

Three-panel layout:
  1. 2D map: EKF trajectory, optional ground truth, obstacle rectangle,
             reference points, pool boundary, transponder and north marker
  2. USBL range comparison: measured vs EKF estimate
  3. Depth profile (Z vs time, Y-axis inverted — marine convention)

A time-window filter (T_START / T_END from ekf_config.py) is applied to both
the EKF trajectory and USBL data so the plot covers the same interval used
during fusion.

Usage:
  python3 ekf_plot.py                          # auto-find latest files in sensor_logs/
  python3 ekf_plot.py --ekf sensor_logs/ekf_X.csv
  python3 ekf_plot.py --ekf ... --usbl ... --gt ground_truth_*.csv
"""

import argparse
import glob
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.collections import LineCollection
import matplotlib.transforms as mtransforms

# ============================================================================
# CONFIGURATION — safe fallbacks; overridden by ekf_config.py when available
# ============================================================================

BOA_COORDINATES          = (2.0, 0.0)
NORTH_MARKER_COORDINATES = (0.0, 8.0)
POOL_RADIUS              = 8.0
boa_position             = [2.0, 0.0, 0.0]
SHOW_RECTANGLE           = False
RECT_CENTER              = [0.0, 0.0]
RECT_WIDTH               = 1.0
RECT_HEIGHT              = 1.0
RECT_ANGLE               = 0.0
RECT_COLOR               = 'orange'
RECT_LABEL               = 'Obstacle'
REFERENCE_POINTS         = []
T_START                  = None
T_END                    = None

try:
    from ekf_config import (BOA_COORDINATES, NORTH_MARKER_COORDINATES,
                             POOL_RADIUS, BOA_POSITION)
    boa_position = BOA_POSITION
except ImportError:
    pass

try:
    from ekf_config import (SHOW_RECTANGLE, RECT_CENTER, RECT_WIDTH, RECT_HEIGHT,
                             RECT_ANGLE, RECT_COLOR, RECT_LABEL, REFERENCE_POINTS)
except (ImportError, AttributeError):
    pass

try:
    from ekf_config import T_START, T_END
except (ImportError, AttributeError):
    pass

DATA_DIR = "sensor_logs"


def find_latest(pattern):
    files = glob.glob(pattern)
    return max(files, key=os.path.getmtime) if files else None


def apply_time_window(df, col, t_start, t_end):
    """Return rows of df where df[col] is within [t_start, t_end]."""
    if t_start is not None:
        df = df[df[col] >= t_start]
    if t_end is not None:
        df = df[df[col] <= t_end]
    return df.reset_index(drop=True)


def main():
    parser = argparse.ArgumentParser(
        description='EKF Plot — visualise fusion results with optional ground truth')
    parser.add_argument('--ekf',  default=None,
                        help='EKF trajectory CSV (default: latest in sensor_logs/)')
    parser.add_argument('--usbl', default=None,
                        help='USBL data CSV (default: latest in sensor_logs/)')
    parser.add_argument('--gt',    default=None,
                        help='Ground truth CSV (default: latest ground_truth_*.csv in sensor_logs/)')
    parser.add_argument('--depth', default=None,
                        help='Depth sensor CSV (default: latest depth_*.csv in sensor_logs/)')
    args = parser.parse_args()

    # ── Load EKF trajectory ──────────────────────────────────────────────────
    ekf_path = args.ekf or find_latest(os.path.join(DATA_DIR, 'ekf_*.csv'))
    if ekf_path is None:
        raise FileNotFoundError(f"No ekf_*.csv file found in {DATA_DIR}/")
    ekf_df = pd.read_csv(ekf_path)
    print(f"  EKF loaded:          {ekf_path}  ({len(ekf_df)} points)")

    # Apply time window to the trajectory
    if T_START is not None or T_END is not None:
        ekf_df = apply_time_window(ekf_df, 'timestamp', T_START, T_END)
        print(f"  Time window applied: [{T_START}, {T_END}] s  →  {len(ekf_df)} points remaining")

    time_new = ekf_df['timestamp'].values
    x_new    = ekf_df['x'].values
    y_new    = ekf_df['y'].values
    z_new    = ekf_df['z'].values
    vx       = ekf_df['vx'].values
    vy       = ekf_df['vy'].values
    speed_2d = np.sqrt(vx**2 + vy**2)

    # ── Load USBL data ───────────────────────────────────────────────────────
    usbl_path = args.usbl or find_latest(os.path.join(DATA_DIR, 'usbl_*.csv'))
    if usbl_path is None:
        raise FileNotFoundError(f"No usbl_*.csv file found in {DATA_DIR}/")
    usbl_df = pd.read_csv(usbl_path)
    print(f"  USBL loaded:         {usbl_path}")

    # Apply the same time window to USBL
    if T_START is not None or T_END is not None:
        usbl_df = apply_time_window(usbl_df, 'timestamp_rel', T_START, T_END)

    time_usbl  = usbl_df['timestamp_rel'].values
    usbl_range = usbl_df['range'].values

    # ── Load depth sensor data ───────────────────────────────────────────────
    depth_path = args.depth or find_latest(os.path.join(DATA_DIR, 'depth_*.csv'))
    if depth_path is None:
        raise FileNotFoundError(f"No depth_*.csv file found in {DATA_DIR}/")
    depth_df = pd.read_csv(depth_path)
    print(f"  Depth loaded:        {depth_path}")

    if T_START is not None or T_END is not None:
        depth_df = apply_time_window(depth_df, 'timestamp_rel', T_START, T_END)

    time_depth = depth_df['timestamp_rel'].values
    depth_raw  = depth_df['depth'].values

    # ── Load ground truth (optional) ─────────────────────────────────────────
    gt_xy   = None
    gt_path = args.gt or find_latest(os.path.join(DATA_DIR, 'ground_truth_*.csv'))
    if gt_path:
        try:
            gt_df = pd.read_csv(gt_path)
            gt_xy = gt_df[['x', 'y']].values
            print(f"  Ground truth loaded: {gt_path}  ({len(gt_xy)} points)")
        except Exception as e:
            print(f"  WARNING: could not load ground truth: {e}")
    else:
        print("  No ground truth file found in sensor_logs/")

    # ── EKF range to transponder ─────────────────────────────────────────────
    bp        = boa_position
    range_ekf = np.sqrt((x_new - bp[0])**2 + (y_new - bp[1])**2 + (z_new - bp[2])**2)

    # ── Statistics ───────────────────────────────────────────────────────────
    dist = np.sum(np.sqrt(np.diff(x_new)**2 + np.diff(y_new)**2))
    range_errors = np.array([
        abs(np.sqrt((x_new[np.argmin(np.abs(time_new - t))] - bp[0])**2 +
                    (y_new[np.argmin(np.abs(time_new - t))] - bp[1])**2 +
                    (z_new[np.argmin(np.abs(time_new - t))] - bp[2])**2) - r)
        for t, r in zip(time_usbl, usbl_range)
    ])

    print(f"\n=== STATISTICS ===")
    print(f"  Duration:          {time_new[-1] - time_new[0]:.2f} s")
    print(f"  Distance traveled: {dist:.2f} m")
    print(f"  Mean speed (2D):   {np.mean(speed_2d):.3f} m/s")
    print(f"  Max  speed (2D):   {np.max(speed_2d):.3f} m/s")
    print(f"  USBL range error:  mean={np.mean(range_errors):.3f} m  "
          f"RMS={np.sqrt(np.mean(range_errors**2)):.3f} m  "
          f"max={np.max(range_errors):.3f} m")

    # ── Figure ───────────────────────────────────────────────────────────────
    # Layout: large 2D map on the left, USBL range + depth stacked on the right.
    fig = plt.figure(figsize=(16, 8))
    gs  = fig.add_gridspec(1, 2, width_ratios=[2.8, 1], wspace=0.28)

    # ---- Plot 1: 2D Map ----
    ax1 = fig.add_subplot(gs[0], aspect='equal')

    # Pool boundary
    ax1.add_patch(Circle((0, 0), POOL_RADIUS, fill=False, edgecolor='blue',
                         linewidth=2, linestyle='--',
                         label=f'Pool (R={POOL_RADIUS:.0f} m)'))

    # Transponder
    ax1.plot(bp[0], bp[1], 'r^', markersize=16, zorder=7,
             label='Buoy (transceiver)',
             markeredgecolor='darkred', markeredgewidth=1.5)
    ax1.annotate('Buoy\n(transceiver)', xy=(bp[0], bp[1]),
                 xytext=(bp[0] + 0.4, bp[1] + 0.4),
                 fontsize=9, color='darkred', fontweight='bold')

    # North marker
    ax1.plot(*NORTH_MARKER_COORDINATES, 's', color='gray',
             markersize=12, label='North marker', zorder=5)

    # Rectangle obstacle
    if SHOW_RECTANGLE:
        rect_patch = Rectangle(
            (RECT_CENTER[0] - RECT_WIDTH / 2, RECT_CENTER[1] - RECT_HEIGHT / 2),
            RECT_WIDTH, RECT_HEIGHT,
            linewidth=2, edgecolor=RECT_COLOR, facecolor=RECT_COLOR,
            alpha=0.25, label=RECT_LABEL, zorder=3
        )
        t_rect = mtransforms.Affine2D().rotate_deg_around(
            RECT_CENTER[0], RECT_CENTER[1], RECT_ANGLE) + ax1.transData
        rect_patch.set_transform(t_rect)
        ax1.add_patch(rect_patch)

    # Reference points
    for rp in REFERENCE_POINTS:
        rx, ry, rlabel = float(rp[0]), float(rp[1]), str(rp[2])
        ax1.plot(rx, ry, 'D', color='purple', markersize=9, zorder=6)
        ax1.annotate(rlabel, xy=(rx, ry), xytext=(rx + 0.25, ry + 0.25),
                     fontsize=8, color='purple', fontweight='bold')
    if REFERENCE_POINTS:
        ax1.plot([], [], 'D', color='purple', markersize=9, label='Reference points')

    # Ground truth overlay
    if gt_xy is not None:
        ax1.plot(gt_xy[:, 0], gt_xy[:, 1], 'm--', lw=2,
                 label='Ground truth', zorder=3, alpha=0.85)

    # EKF trajectory coloured by elapsed time
    points   = np.array([x_new, y_new]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm     = plt.Normalize(time_new.min(), time_new.max())
    lc = LineCollection(segments, cmap='plasma', norm=norm,
                        linewidth=2.5, label='EKF trajectory', zorder=4)
    lc.set_array(time_new[:-1])
    ax1.add_collection(lc)

    ax1.plot(x_new[0],  y_new[0],  'go', markersize=12, label='Start', zorder=6)
    ax1.plot(x_new[-1], y_new[-1], 'rs', markersize=12, label='End',   zorder=6)
    plt.colorbar(lc, ax=ax1, label='Time [s]')

    margin = 1.5
    ax1.set_xlim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax1.set_ylim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax1.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax1.axvline(0, color='k', lw=0.5, alpha=0.3)
    ax1.set_xlabel('X [m]', fontsize=12)
    ax1.set_ylabel('Y [m]', fontsize=12)
    ax1.set_title('EKF Trajectory', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=8)

    # ---- Right column: USBL range + Depth stacked ----
    gs_right = gs[1].subgridspec(2, 1, hspace=0.55)

    # ---- Plot 2: USBL Range Comparison ----
    ax2 = fig.add_subplot(gs_right[0])
    ax2.plot(time_usbl, usbl_range, 'o-', color='steelblue',
             markersize=8, linewidth=2, label='USBL measured', zorder=3)
    ax2.plot(time_new, range_ekf, '-', color='red', alpha=0.7,
             linewidth=1.5, label='EKF estimate', zorder=2)
    ax2.axhline(np.mean(usbl_range), color='blue', ls='--', alpha=0.3, lw=1,
                label='Mean range')
    ax2.set_xlabel('Time [s]', fontsize=9)
    ax2.set_ylabel('Range [m]', fontsize=9)
    ax2.set_title('USBL Range', fontsize=10, fontweight='bold')
    ax2.tick_params(labelsize=8)
    ax2.grid(True, alpha=0.3)
    ax2.legend(fontsize=7)

    # ---- Plot 3: Depth Profile (raw depth sensor readings) ----
    # Depth is negated so the surface is 0 and 12 m depth appears as -12
    # at the bottom of the chart (natural y-axis, no invert needed).
    ax3 = fig.add_subplot(gs_right[1])
    ax3.plot(time_depth, -depth_raw, '-', color='teal', linewidth=1.5)
    ax3.fill_between(time_depth, -depth_raw, alpha=0.15, color='teal')
    ax3.axhline(0, color='k', lw=0.5, alpha=0.4, ls='--')
    ax3.set_xlabel('Time [s]', fontsize=9)
    ax3.set_ylabel('Depth [m]', fontsize=9)
    ax3.set_title('Depth Profile', fontsize=10, fontweight='bold')
    ax3.tick_params(labelsize=8)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('ekf_fusion_result.svg', bbox_inches='tight')
    print(f"\n  Plot saved: ekf_fusion_result.svg")
    plt.show()


if __name__ == '__main__':
    main()
