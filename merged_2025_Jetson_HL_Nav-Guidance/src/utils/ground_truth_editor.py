#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ground Truth Editor for AUV navigation.
Draw a ground truth trajectory manually on the pool map.

Controls:
  Left click   → add waypoint
  Right click  → remove nearest waypoint
  Z            → undo last waypoint
  C            → clear all waypoints
  S            → toggle spline / linear segments
  Enter        → save CSV and close
  Esc          → exit without saving
"""

import argparse
import glob
import os
import numpy as np
import matplotlib
# Robust backend selection for WSL2 / headless environments
for _backend in ['TkAgg', 'Qt5Agg', 'Qt6Agg', 'GTK3Agg', 'MacOSX', 'Agg']:
    try:
        matplotlib.use(_backend)
        import matplotlib.pyplot as plt
        if _backend != 'Agg':
            _fig = plt.figure()
            plt.close(_fig)
        break
    except Exception:
        continue
else:
    raise RuntimeError(
        "No graphical backend available. "
        "On WSL2: install VcXsrv or enable X11 forwarding (export DISPLAY=:0).")

from matplotlib.patches import Circle
from datetime import datetime
import pandas as pd
from scipy.interpolate import CubicSpline

# ── Configuration ─────────────────────────────────────────────────────────────
try:
    from ekf_config import (POOL_RADIUS, BOA_COORDINATES,
                            NORTH_MARKER_COORDINATES, BOA_POSITION)
except (ImportError, SyntaxError):
    POOL_RADIUS              = 8.0
    BOA_COORDINATES          = (0.0, 0.0)
    NORTH_MARKER_COORDINATES = (0.0, 8.0)
    BOA_POSITION             = [0.0, 0.0, 0.0]

SPLINE_SAMPLE_STEP   = 0.05   # [m] spline sampling step
WAYPOINT_SNAP_RADIUS = 0.5    # [m] snap radius for right-click removal


# ── Utilities ─────────────────────────────────────────────────────────────────

def load_ekf_csv(path):
    """Load an EKF trajectory CSV. Returns Nx2 array [x, y] or None."""
    try:
        df = pd.read_csv(path)
        return df[['x', 'y']].values
    except Exception as e:
        print(f"  WARNING: could not load EKF CSV '{path}': {e}")
        return None


def build_axes(ax, ekf_xy=None):
    """Draw the static pool map."""
    ax.set_aspect('equal')
    margin = 1.5
    ax.set_xlim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax.set_ylim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.grid(True, alpha=0.3)
    ax.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax.axvline(0, color='k', lw=0.5, alpha=0.3)

    # Pool boundary
    ax.add_patch(Circle((0, 0), POOL_RADIUS, fill=False,
                         edgecolor='blue', lw=2, ls='--', label='Pool'))

    # Transponder
    ax.plot(BOA_COORDINATES[0], BOA_COORDINATES[1], 'r^',
            markersize=16, label='Buoy (transceiver)',
            markeredgecolor='darkred', markeredgewidth=1.5, zorder=5)
    ax.annotate('Buoy\n(transceiver)', xy=BOA_COORDINATES,
                xytext=(BOA_COORDINATES[0] + 0.4, BOA_COORDINATES[1] + 0.4),
                fontsize=9, color='darkred', fontweight='bold')

    # North marker
    ax.plot(*NORTH_MARKER_COORDINATES, 's', color='gray',
            markersize=12, label='North marker', zorder=5)

    # EKF trajectory overlay (background, semi-transparent)
    if ekf_xy is not None:
        ax.plot(ekf_xy[:, 0], ekf_xy[:, 1],
                color='steelblue', lw=1.5, alpha=0.4,
                label='EKF (estimated)', zorder=2)


# ── Editor ────────────────────────────────────────────────────────────────────

class GroundTruthEditor:
    def __init__(self, ekf_xy=None):
        self.waypoints  = []
        self.ekf_xy     = ekf_xy
        self.use_spline = True   # True = cubic spline, False = linear segments

        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        build_axes(self.ax, ekf_xy)

        self.wp_scatter   = self.ax.scatter([], [], s=80, color='red',
                                             zorder=6, label='GT waypoints')
        self.spline_line, = self.ax.plot([], [], 'g-', lw=2.5,
                                          label='Ground truth', zorder=5)
        self.wp_labels    = []

        self.ax.legend(loc='upper right', fontsize=9)
        self._update_title()

    # ── Path computation ──────────────────────────────────────────────────────

    def _compute_spline(self):
        """Cubic spline through waypoints. Returns Nx2 array or None."""
        pts = np.array(self.waypoints)
        if len(pts) < 2:
            return None
        dists = np.concatenate([[0], np.cumsum(
            np.sqrt(np.diff(pts[:, 0])**2 + np.diff(pts[:, 1])**2))])
        total = dists[-1]
        if total < 1e-6:
            return None
        cs_x = CubicSpline(dists, pts[:, 0])
        cs_y = CubicSpline(dists, pts[:, 1])
        t = np.arange(0, total, SPLINE_SAMPLE_STEP)
        if len(t) == 0 or t[-1] < total:
            t = np.append(t, total)
        return np.column_stack([cs_x(t), cs_y(t)])

    def _compute_segments(self):
        """Linear interpolation between waypoints. Returns Nx2 array or None."""
        pts = np.array(self.waypoints)
        if len(pts) < 2:
            return None
        result = []
        for i in range(len(pts) - 1):
            p0, p1  = pts[i], pts[i + 1]
            seg_len = np.linalg.norm(p1 - p0)
            if seg_len < 1e-6:
                continue
            n_steps = max(2, int(np.ceil(seg_len / SPLINE_SAMPLE_STEP)))
            ts      = np.linspace(0, 1, n_steps, endpoint=False)
            result.extend(p0 + s * (p1 - p0) for s in ts)
        result.append(pts[-1])
        return np.array(result)

    def _compute_path(self):
        return self._compute_spline() if self.use_spline else self._compute_segments()

    # ── Rendering ─────────────────────────────────────────────────────────────

    def _redraw(self):
        if self.waypoints:
            self.wp_scatter.set_offsets(np.array(self.waypoints))
        else:
            self.wp_scatter.set_offsets(np.empty((0, 2)))

        for lbl in self.wp_labels:
            lbl.remove()
        self.wp_labels = []
        for i, (x, y) in enumerate(self.waypoints):
            t = self.ax.text(x + 0.15, y + 0.15, str(i + 1),
                             fontsize=8, color='darkred', zorder=7)
            self.wp_labels.append(t)

        path_pts = self._compute_path()
        if path_pts is not None:
            self.spline_line.set_data(path_pts[:, 0], path_pts[:, 1])
        else:
            self.spline_line.set_data([], [])

        self._update_title()
        self.fig.canvas.draw_idle()

    def _update_title(self):
        n    = len(self.waypoints)
        mode = 'SPLINE' if self.use_spline else 'SEGMENTS'
        self.ax.set_title(
            f'Ground Truth Editor  |  Waypoints: {n}  |  Mode: {mode}\n'
            'Left click=add  Right click=remove  Z=undo  C=clear  '
            'S=toggle mode  Enter=save  Esc=quit',
            fontsize=10
        )

    # ── Event handlers ────────────────────────────────────────────────────────

    def _on_click(self, event):
        if event.inaxes != self.ax:
            return
        x, y = event.xdata, event.ydata

        if event.button == 1:           # left → add
            self.waypoints.append((x, y))
            self._redraw()

        elif event.button == 3:         # right → remove nearest
            if not self.waypoints:
                return
            pts   = np.array(self.waypoints)
            dists = np.sqrt((pts[:, 0] - x)**2 + (pts[:, 1] - y)**2)
            idx   = int(np.argmin(dists))
            if dists[idx] < WAYPOINT_SNAP_RADIUS:
                self.waypoints.pop(idx)
                self._redraw()

    def _on_key(self, event):
        key = event.key

        if key == 'z' and self.waypoints:
            self.waypoints.pop()
            self._redraw()

        elif key == 'c':
            self.waypoints.clear()
            self._redraw()

        elif key == 's':
            self.use_spline = not self.use_spline
            print(f"  Mode: {'cubic spline' if self.use_spline else 'linear segments'}")
            self._redraw()

        elif key == 'enter':
            self._save_and_close()

        elif key == 'escape':
            plt.close(self.fig)

    # ── Save ──────────────────────────────────────────────────────────────────

    def _save_and_close(self):
        path_pts = self._compute_path()
        if path_pts is None:
            print("  WARNING: nothing to save — add at least 2 waypoints first.")
            return

        mode      = 'spline' if self.use_spline else 'segments'
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename  = f'ground_truth_{timestamp}.csv'
        df = pd.DataFrame(path_pts, columns=['x', 'y'])
        df.to_csv(filename, index=False)
        print(f"  Ground truth saved: {filename}  ({len(df)} points, mode: {mode})")
        plt.close(self.fig)

    # ── Run ───────────────────────────────────────────────────────────────────

    def run(self):
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)
        self.fig.canvas.mpl_connect('key_press_event',   self._on_key)
        self._redraw()
        plt.tight_layout()
        plt.show()


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='Ground Truth Editor for AUV')
    parser.add_argument('--ekf', type=str, default=None,
                        help='EKF trajectory CSV to overlay as background')
    args = parser.parse_args()

    ekf_xy = None
    if args.ekf:
        ekf_xy = load_ekf_csv(args.ekf)
    else:
        # Auto-detect the latest ekf_*.csv
        candidates = (glob.glob('ekf_*.csv') +
                      glob.glob('sensor_logs/ekf_*.csv') +
                      glob.glob('data_paper/*/ekf_*.csv'))
        if candidates:
            latest = max(candidates, key=os.path.getmtime)
            print(f"  EKF auto-detected: {latest}")
            ekf_xy = load_ekf_csv(latest)
        else:
            print("  No EKF file found — starting without overlay.")

    editor = GroundTruthEditor(ekf_xy=ekf_xy)
    editor.run()


if __name__ == '__main__':
    main()
