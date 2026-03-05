#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Sensor Fusion for AUV navigation.
Fuses IMU, USBL and depth sensor measurements with an Extended Kalman Filter.

Features:
- Automatic covariance estimation from sensor data
- Constant-velocity motion model (more stable for AUVs)
- IMU bias correction via static calibration file
- Range-only USBL update (angles ignored)
- Time-window filter to restrict processing to a subset of the recording
- Trajectory saved to CSV for use with ekf_plot.py and ground_truth_editor.py
- Integrated visualization: rectangle obstacle, reference points, depth profile
"""

import os
import glob
import json
from datetime import datetime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle
from matplotlib.collections import LineCollection
import matplotlib.transforms as mtransforms

# ============================================================================
# CONFIGURATION — fallback defaults (overridden by ekf_config.py at runtime)
# ============================================================================

BOA_COORDINATES          = (2.0, 0.0)
NORTH_MARKER_COORDINATES = (0.0, 8.0)
POOL_RADIUS              = 8.0
DATA_DIR                 = "sensor_logs"
APPLY_BIAS_CORRECTION    = True
VERBOSE                  = True

# ============================================================================
# UTILITY
# ============================================================================

def find_latest_csv(directory, prefix):
    pattern = os.path.join(directory, f"{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"No file found matching pattern: {pattern}")
    return max(files, key=os.path.getmtime)


def apply_time_window(df, col, t_start, t_end):
    """Return rows of df where df[col] is within [t_start, t_end]."""
    if t_start is not None:
        df = df[df[col] >= t_start]
    if t_end is not None:
        df = df[df[col] <= t_end]
    return df.reset_index(drop=True)

# ============================================================================
# SENSOR NOISE ANALYSIS
# ============================================================================

def analyze_sensor_noise(imu_data, usbl_data, depth_data, imu_calibration=None):
    """
    Estimate covariance matrices from sensor data.

    Args:
        imu_data:        DataFrame with IMU measurements
        usbl_data:       DataFrame with USBL measurements
        depth_data:      DataFrame with depth measurements
        imu_calibration: dict with static calibration
                         {'bias_acc': [x,y,z], 'var_acc': [x,y,z],
                          'bias_gyr': [x,y,z], 'var_gyr': [x,y,z]}
    """
    print("\n" + "="*70)
    print("SENSOR NOISE ANALYSIS")
    print("="*70)

    # --- IMU ---
    if imu_calibration is not None:
        print("\n  Using static IMU calibration")
        bias_acc = np.array(imu_calibration['bias_acc'])
        bias_gyr = np.array(imu_calibration['bias_gyr'])
        var_acc  = np.array(imu_calibration['var_acc'])
        var_gyr  = np.array(imu_calibration['var_gyr'])

        print(f"\n  IMU (static calibration):")
        print(f"    Accel bias: [{bias_acc[0]:.6f}, {bias_acc[1]:.6f}, {bias_acc[2]:.6f}] m/s²")
        print(f"    Accel std:  [{np.sqrt(var_acc[0]):.6f}, {np.sqrt(var_acc[1]):.6f}, {np.sqrt(var_acc[2]):.6f}] m/s²")
        print(f"    Gyro  bias: [{bias_gyr[0]:.6f}, {bias_gyr[1]:.6f}, {bias_gyr[2]:.6f}] rad/s")
        print(f"    Gyro  std:  [{np.sqrt(var_gyr[0]):.6f}, {np.sqrt(var_gyr[1]):.6f}, {np.sqrt(var_gyr[2]):.6f}] rad/s")
    else:
        print("\n  WARNING: estimating bias/noise from motion data — use static calibration instead")

        acc_x = imu_data['acc_x'].values
        acc_y = imu_data['acc_y'].values
        acc_z = imu_data['acc_z'].values
        gyr_x = imu_data['gyr_x'].values
        gyr_y = imu_data['gyr_y'].values
        gyr_z = imu_data['gyr_z'].values

        bias_acc = np.array([np.mean(acc_x), np.mean(acc_y), np.mean(acc_z)])
        bias_gyr = np.array([np.mean(gyr_x), np.mean(gyr_y), np.mean(gyr_z)])
        var_acc  = np.array([np.var(acc_x),  np.var(acc_y),  np.var(acc_z)])
        var_gyr  = np.array([np.var(gyr_x),  np.var(gyr_y),  np.var(gyr_z)])

        print(f"\n  IMU (rough estimate):")
        print(f"    Accel bias: [{bias_acc[0]:.4f}, {bias_acc[1]:.4f}, {bias_acc[2]:.4f}] m/s²")
        print(f"    Accel std:  [{np.sqrt(var_acc[0]):.4f}, {np.sqrt(var_acc[1]):.4f}, {np.sqrt(var_acc[2]):.4f}] m/s²")
        print(f"    Gyro  bias: [{bias_gyr[0]:.4f}, {bias_gyr[1]:.4f}, {bias_gyr[2]:.4f}] rad/s")
        print(f"    Gyro  std:  [{np.sqrt(var_gyr[0]):.4f}, {np.sqrt(var_gyr[1]):.4f}, {np.sqrt(var_gyr[2]):.4f}] rad/s")

    # --- USBL ---
    usbl_range = usbl_data['range'].values
    var_usbl   = np.var(usbl_range)
    print(f"\n  USBL:")
    print(f"    Mean range: {np.mean(usbl_range):.3f} m")
    print(f"    Std  range: {np.sqrt(var_usbl):.3f} m")

    # --- Depth ---
    if 'depth' in depth_data.columns:
        depth     = depth_data['depth'].values
        var_depth = np.var(depth)
        print(f"\n  Depth sensor:")
        print(f"    Mean depth: {np.mean(depth):.3f} m")
        print(f"    Std  depth: {np.sqrt(var_depth):.4f} m")
    else:
        var_depth = 0.01
        print(f"\n  Depth sensor: using default variance {var_depth}")

    return {
        'bias_acc': bias_acc,
        'bias_gyr': bias_gyr,
        'var_acc':  var_acc,
        'var_gyr':  var_gyr,
        'var_usbl': var_usbl,
        'var_depth': var_depth
    }

# ============================================================================
# EXTENDED KALMAN FILTER
# ============================================================================

class EKF_AUV:
    """
    Extended Kalman Filter for AUV navigation.

    State (6D): [x, y, z, vx, vy, vz]
    Motion model: constant velocity with optional IMU acceleration correction.
    """

    def __init__(self, sensor_noise, dt=0.02, config=None, initial_state=None, boa_position=None):
        self.dt           = dt
        self.sensor_noise = sensor_noise

        # Transponder (USBL) position used for range computation
        self.boa_position = np.zeros(3) if boa_position is None else np.array(boa_position)
        if boa_position is not None and VERBOSE:
            print(f"\n  Buoy (transceiver) position: [{boa_position[0]:.2f}, {boa_position[1]:.2f}, {boa_position[2]:.2f}] m")

        if config is None:
            config = {
                'Q_POSITION': 0.01, 'Q_VELOCITY': 0.5, 'Q_VELOCITY_Z': 0.1,
                'R_USBL_FACTOR': 1.5, 'R_DEPTH_MIN': 0.01,
                'P0_POSITION_XY': 1.0, 'P0_POSITION_Z': 0.1, 'P0_VELOCITY': 0.01,
                'ACC_INTEGRATION_WEIGHT': 0.1
            }

        self.config     = config
        self.acc_weight = config.get('ACC_INTEGRATION_WEIGHT', 0.1)

        # State: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        if initial_state is not None:
            if 'position' in initial_state:
                pos = initial_state['position']
                self.x[0:3] = pos
                if VERBOSE:
                    print(f"\n  Initial position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] m")
            if 'velocity' in initial_state:
                vel = initial_state['velocity']
                self.x[3:6] = vel
                if VERBOSE:
                    print(f"  Initial velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}] m/s")

        # Initial covariance P0
        self.P = np.diag([
            config.get('P0_POSITION_XY', 1.0),
            config.get('P0_POSITION_XY', 1.0),
            config.get('P0_POSITION_Z',  0.1),
            config.get('P0_VELOCITY',    0.01),
            config.get('P0_VELOCITY',    0.01),
            config.get('P0_VELOCITY',    0.01)
        ])

        # Process noise Q
        q_pos   = config.get('Q_POSITION',   0.01)
        q_vel   = config.get('Q_VELOCITY',   0.5)
        q_vel_z = config.get('Q_VELOCITY_Z', 0.1)
        self.Q  = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel_z])

        # Measurement noise R
        r_factor   = config.get('R_USBL_FACTOR', 1.5)
        r_depth_mn = config.get('R_DEPTH_MIN',   0.01)
        self.R_usbl  = np.array([[sensor_noise['var_usbl'] * r_factor]])
        self.R_depth = np.array([[max(sensor_noise['var_depth'], r_depth_mn)]])

        if VERBOSE:
            print(f"\n" + "="*70)
            print("EKF INITIALISATION")
            print("="*70)
            print(f"\n  Process noise Q:")
            print(f"    Position:   {q_pos} m²")
            print(f"    Velocity:   {q_vel} m²/s²")
            print(f"    Velocity Z: {q_vel_z} m²/s²")
            print(f"\n  Measurement noise R:")
            print(f"    USBL range: {self.R_usbl[0,0]:.4f} m²")
            print(f"    Depth:      {self.R_depth[0,0]:.4f} m²")
            print(f"\n  Acceleration integration weight: {self.acc_weight}")

    def predict(self, acc_body, gyro, roll, pitch, yaw):
        """
        Prediction step — integrates IMU measurements.

        Constant-velocity model with a weighted acceleration correction:
          x_{k+1} = x_k + vx_k * dt
          v_{k+1} = v_k + a_world * dt * weight
        """
        R_bw      = self._rotation_matrix(roll, pitch, yaw)
        acc_world = R_bw @ acc_body

        x_pred    = self.x.copy()
        x_pred[0] = self.x[0] + self.x[3] * self.dt
        x_pred[1] = self.x[1] + self.x[4] * self.dt
        x_pred[2] = self.x[2] + self.x[5] * self.dt
        x_pred[3] = self.x[3] + acc_world[0] * self.dt * self.acc_weight
        x_pred[4] = self.x[4] + acc_world[1] * self.dt * self.acc_weight
        x_pred[5] = self.x[5] + acc_world[2] * self.dt * self.acc_weight

        F       = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt

        self.x = x_pred
        self.P = F @ self.P @ F.T + self.Q

    def update_usbl(self, range_measured):
        """
        Update step — range-only USBL correction.
        h(x) = sqrt((x-x_t)² + (y-y_t)² + (z-z_t)²)  where t = transponder
        """
        dx = self.x[0] - self.boa_position[0]
        dy = self.x[1] - self.boa_position[1]
        dz = self.x[2] - self.boa_position[2]
        r  = max(np.sqrt(dx**2 + dy**2 + dz**2), 1e-6)

        H      = np.zeros((1, 6))
        H[0,0] = dx / r
        H[0,1] = dy / r
        H[0,2] = dz / r

        y   = range_measured - r
        S   = H @ self.P @ H.T + self.R_usbl
        K   = self.P @ H.T / S[0, 0]
        self.x = self.x + K.flatten() * y

        # Joseph form for numerical stability
        I   = np.eye(6)
        IKH = I - np.outer(K, H)
        self.P = IKH @ self.P @ IKH.T + np.outer(K, K) * self.R_usbl[0, 0]

    def update_depth(self, depth_measured):
        """Update step — depth sensor correction.  h(x) = z"""
        H      = np.zeros((1, 6))
        H[0,2] = 1.0
        y      = depth_measured - self.x[2]
        S      = H @ self.P @ H.T + self.R_depth
        K      = self.P @ H.T / S[0, 0]
        self.x = self.x + K.flatten() * y
        self.P = (np.eye(6) - np.outer(K, H)) @ self.P

    def _rotation_matrix(self, roll, pitch, yaw):
        """Body-to-world rotation matrix (ZYX convention)."""
        cr, sr = np.cos(roll),  np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw),   np.sin(yaw)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr            ]
        ])

    def get_state(self):
        return self.x.copy()

    def get_covariance(self):
        return self.P.copy()

# ============================================================================
# SENSOR FUSION LOOP
# ============================================================================

def run_sensor_fusion(imu_data, usbl_data, depth_data, sensor_noise,
                      config=None, initial_state=None, boa_position=None):
    """
    Run the EKF sensor fusion loop.

    Returns:
        trajectory: ndarray of shape (N, 7) → [timestamp, x, y, z, vx, vy, vz]
    """
    print("\n" + "="*70)
    print("SENSOR FUSION")
    print("="*70)

    dt_imu   = np.diff(imu_data['timestamp_rel'].values)
    dt_mean  = np.mean(dt_imu)
    print(f"\n  IMU rate: {1/dt_mean:.1f} Hz  (dt = {dt_mean:.4f} s)")

    ekf = EKF_AUV(sensor_noise, dt=dt_mean, config=config,
                  initial_state=initial_state, boa_position=boa_position)

    time_imu   = imu_data['timestamp_rel'].values
    time_usbl  = usbl_data['timestamp_rel'].values
    time_depth = depth_data['timestamp_rel'].values if len(depth_data) > 0 else np.array([])

    bias_acc = sensor_noise['bias_acc'] if APPLY_BIAS_CORRECTION else np.zeros(3)
    bias_gyr = sensor_noise['bias_gyr'] if APPLY_BIAS_CORRECTION else np.zeros(3)

    trajectory    = []
    idx_usbl      = 0
    idx_depth     = 0

    print(f"\n  IMU samples:   {len(imu_data)}")
    print(f"  USBL fixes:    {len(usbl_data)}")
    print(f"  Depth samples: {len(depth_data)}")
    print(f"\n  Running fusion...")

    for i in range(len(imu_data)):
        t = time_imu[i]

        acc_body = np.array([
            imu_data.iloc[i]['acc_x'] - bias_acc[0],
            imu_data.iloc[i]['acc_y'] - bias_acc[1],
            imu_data.iloc[i]['acc_z'] - bias_acc[2]
        ])
        gyro = np.array([
            imu_data.iloc[i]['gyr_x'] - bias_gyr[0],
            imu_data.iloc[i]['gyr_y'] - bias_gyr[1],
            imu_data.iloc[i]['gyr_z'] - bias_gyr[2]
        ])
        roll  = imu_data.iloc[i]['roll']
        pitch = imu_data.iloc[i]['pitch']
        yaw   = imu_data.iloc[i]['yaw']

        ekf.predict(acc_body, gyro, roll, pitch, yaw)

        # USBL update
        if idx_usbl < len(usbl_data) and abs(t - time_usbl[idx_usbl]) < dt_mean:
            range_meas = usbl_data.iloc[idx_usbl]['range']
            ekf.update_usbl(range_meas)
            idx_usbl += 1
            if VERBOSE and idx_usbl % 5 == 0:
                print(f"    t={t:6.2f} s  USBL fix #{idx_usbl}  range={range_meas:.2f} m")

        # Depth update
        if len(time_depth) > 0 and idx_depth < len(depth_data):
            if abs(t - time_depth[idx_depth]) < dt_mean:
                if 'depth' in depth_data.columns:
                    ekf.update_depth(depth_data.iloc[idx_depth]['depth'])
                idx_depth += 1

        state = ekf.get_state()
        trajectory.append([t, state[0], state[1], state[2],
                           state[3], state[4], state[5]])

    trajectory = np.array(trajectory)
    print(f"\n  Fusion complete: {len(trajectory)} points, {idx_usbl}/{len(usbl_data)} USBL fixes used")
    return trajectory

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    print("="*70)
    print("EKF SENSOR FUSION — AUV NAVIGATION")
    print("="*70)

    # --- Load configuration ---
    config       = None
    initial_state = None
    boa_position = None
    T_START      = None
    T_END        = None

    SHOW_RECTANGLE   = False
    RECT_CENTER      = [0.0, 0.0]
    RECT_WIDTH       = 1.0
    RECT_HEIGHT      = 1.0
    RECT_ANGLE       = 0.0
    RECT_COLOR       = 'orange'
    RECT_LABEL       = 'Obstacle'
    REFERENCE_POINTS = []

    try:
        from ekf_config import (Q_POSITION, Q_VELOCITY, Q_VELOCITY_Z,
                                R_USBL_FACTOR, R_DEPTH_MIN,
                                P0_POSITION_XY, P0_POSITION_Z, P0_VELOCITY,
                                ACC_INTEGRATION_WEIGHT,
                                INITIAL_POSITION, INITIAL_VELOCITY,
                                BOA_POSITION, T_START, T_END)
        config = {
            'Q_POSITION': Q_POSITION, 'Q_VELOCITY': Q_VELOCITY,
            'Q_VELOCITY_Z': Q_VELOCITY_Z, 'R_USBL_FACTOR': R_USBL_FACTOR,
            'R_DEPTH_MIN': R_DEPTH_MIN, 'P0_POSITION_XY': P0_POSITION_XY,
            'P0_POSITION_Z': P0_POSITION_Z, 'P0_VELOCITY': P0_VELOCITY,
            'ACC_INTEGRATION_WEIGHT': ACC_INTEGRATION_WEIGHT
        }
        initial_state = {'position': INITIAL_POSITION, 'velocity': INITIAL_VELOCITY}
        boa_position  = BOA_POSITION
        print("\n  Configuration loaded from ekf_config.py")
    except ImportError:
        print("\n  ekf_config.py not found — using built-in defaults")

    try:
        from ekf_config import (SHOW_RECTANGLE, RECT_CENTER, RECT_WIDTH, RECT_HEIGHT,
                                RECT_ANGLE, RECT_COLOR, RECT_LABEL, REFERENCE_POINTS,
                                BOA_COORDINATES, NORTH_MARKER_COORDINATES, POOL_RADIUS)
    except (ImportError, AttributeError):
        pass

    # --- Load IMU calibration ---
    imu_calibration = None
    try:
        with open('imu_calibration.json', 'r') as f:
            imu_calibration = json.load(f)
        print("  IMU static calibration loaded from imu_calibration.json")
    except FileNotFoundError:
        print("  imu_calibration.json not found — will estimate from data")

    # --- Load sensor CSV files ---
    print("\n  Loading sensor data...")
    imu_data   = pd.read_csv(find_latest_csv(DATA_DIR, "imu"))
    usbl_data  = pd.read_csv(find_latest_csv(DATA_DIR, "usbl"))
    depth_data = pd.read_csv(find_latest_csv(DATA_DIR, "depth"))

    # --- Apply time window filter ---
    if T_START is not None or T_END is not None:
        print(f"\n  Applying time window filter: [{T_START}, {T_END}] s")
        n_imu_before   = len(imu_data)
        n_usbl_before  = len(usbl_data)
        n_depth_before = len(depth_data)

        imu_data   = apply_time_window(imu_data,   'timestamp_rel', T_START, T_END)
        usbl_data  = apply_time_window(usbl_data,  'timestamp_rel', T_START, T_END)
        depth_data = apply_time_window(depth_data, 'timestamp_rel', T_START, T_END)

        print(f"    IMU:   {n_imu_before} → {len(imu_data)} samples")
        print(f"    USBL:  {n_usbl_before} → {len(usbl_data)} fixes")
        print(f"    Depth: {n_depth_before} → {len(depth_data)} samples")

    # --- Analyse sensor noise ---
    sensor_noise = analyze_sensor_noise(imu_data, usbl_data, depth_data, imu_calibration)

    # --- Run EKF ---
    trajectory = run_sensor_fusion(imu_data, usbl_data, depth_data, sensor_noise,
                                   config, initial_state, boa_position)

    # ========================================================================
    # SAVE TRAJECTORY CSV
    # The saved file is loaded by ekf_plot.py and ground_truth_editor.py.
    # ========================================================================

    ts      = datetime.now().strftime('%Y%m%d_%H%M%S')
    ekf_out = os.path.join(DATA_DIR, f'ekf_{ts}.csv')
    ekf_df  = pd.DataFrame(trajectory,
                           columns=['timestamp', 'x', 'y', 'z', 'vx', 'vy', 'vz'])
    ekf_df.to_csv(ekf_out, index=False)

    x_new    = trajectory[:, 1]
    y_new    = trajectory[:, 2]
    time_new = trajectory[:, 0]
    dist     = np.sum(np.sqrt(np.diff(x_new)**2 + np.diff(y_new)**2))

    print(f"\n=== SUMMARY ===")
    print(f"  Duration:          {time_new[-1] - time_new[0]:.2f} s")
    print(f"  Distance traveled: {dist:.2f} m")
    print(f"  Trajectory points: {len(ekf_df)}")
    print(f"\n  Trajectory saved: {ekf_out}")
    print(f"  Run: python3 ekf_plot.py             to plot with ground truth")
    print(f"  Run: python3 ground_truth_editor.py  to draw ground truth")

    # ========================================================================
    # VISUALIZATION
    # Layout: 2D map (left, dominant) | USBL range + Depth profile (right)
    # ========================================================================

    print("\n" + "="*70)
    print("VISUALIZATION")
    print("="*70)

    fig = plt.figure(figsize=(16, 8))
    gs  = fig.add_gridspec(1, 2, width_ratios=[2.8, 1], wspace=0.28)

    # ---- Plot 1: 2D Trajectory Map ----
    ax1 = fig.add_subplot(gs[0], aspect='equal')

    ax1.add_patch(Circle((0, 0), POOL_RADIUS, fill=False, edgecolor='blue',
                         linewidth=2, linestyle='--',
                         label=f'Pool (R={POOL_RADIUS:.0f} m)'))

    # Transponder marker — uses the EKF's own position for consistency
    tp_xy = (boa_position[0], boa_position[1]) if boa_position is not None \
            else BOA_COORDINATES
    ax1.plot(tp_xy[0], tp_xy[1], 'r^', markersize=16, zorder=7,
             label='Buoy (transceiver)',
             markeredgecolor='darkred', markeredgewidth=1.5)
    ax1.annotate('Buoy\n(transceiver)', xy=tp_xy,
                 xytext=(tp_xy[0] + 0.4, tp_xy[1] + 0.4),
                 fontsize=9, color='darkred', fontweight='bold')

    ax1.plot(NORTH_MARKER_COORDINATES[0], NORTH_MARKER_COORDINATES[1],
             's', color='gray', markersize=12, label='North marker', zorder=5)

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

    # EKF trajectory coloured by time
    points   = np.array([x_new, y_new]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm     = plt.Normalize(time_new.min(), time_new.max())
    lc       = LineCollection(segments, cmap='plasma', norm=norm,
                              linewidth=2.5, label='EKF trajectory', zorder=4)
    lc.set_array(time_new[:-1])
    ax1.add_collection(lc)

    ax1.plot(x_new[0],  y_new[0],  'go', markersize=12, label='Start', zorder=8)
    ax1.plot(x_new[-1], y_new[-1], 'rs', markersize=12, label='End',   zorder=8)
    plt.colorbar(lc, ax=ax1, label='Time [s]')

    margin = 1.5
    ax1.set_xlim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax1.set_ylim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax1.axhline(0, color='k', lw=0.5, alpha=0.3)
    ax1.axvline(0, color='k', lw=0.5, alpha=0.3)
    ax1.set_xlabel('X [m]', fontsize=12)
    ax1.set_ylabel('Y [m]', fontsize=12)
    ax1.set_title('EKF Trajectory (Constant Velocity)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=8)

    # ---- Right column: USBL range + Depth ----
    gs_right = gs[1].subgridspec(2, 1, hspace=0.55)

    ax2 = fig.add_subplot(gs_right[0])
    time_usbl  = usbl_data['timestamp_rel'].values
    usbl_range = usbl_data['range'].values

    if boa_position is not None:
        bp = boa_position
        range_ekf = np.sqrt((x_new - bp[0])**2 +
                            (y_new - bp[1])**2 +
                            (trajectory[:, 3] - bp[2])**2)
    else:
        range_ekf = np.sqrt(x_new**2 + y_new**2 + trajectory[:, 3]**2)

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

    # Depth profile — Y-axis inverted (marine convention: surface at top)
    ax3 = fig.add_subplot(gs_right[1])
    depth_z = trajectory[:, 3]
    ax3.plot(time_new, depth_z, '-', color='teal', linewidth=1.5)
    ax3.fill_between(time_new, depth_z, alpha=0.15, color='teal')
    ax3.invert_yaxis()
    ax3.set_xlabel('Time [s]', fontsize=9)
    ax3.set_ylabel('Depth [m]', fontsize=9)
    ax3.set_title('Depth Profile', fontsize=10, fontweight='bold')
    ax3.tick_params(labelsize=8)
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()

    # --- Final statistics ---
    vx       = trajectory[:, 4]
    vy       = trajectory[:, 5]
    speed_2d = np.sqrt(vx**2 + vy**2)
    range_errors = []
    for i, t_u in enumerate(time_usbl):
        idx = np.argmin(np.abs(time_new - t_u))
        if boa_position is not None:
            bp = boa_position
            re = np.sqrt((x_new[idx]-bp[0])**2 + (y_new[idx]-bp[1])**2 +
                         (trajectory[idx,3]-bp[2])**2)
        else:
            re = np.sqrt(x_new[idx]**2 + y_new[idx]**2 + trajectory[idx,3]**2)
        range_errors.append(abs(re - usbl_range[i]))
    range_errors = np.array(range_errors)

    print(f"\n=== FINAL STATISTICS ===")
    print(f"  Duration:          {time_new[-1] - time_new[0]:.2f} s")
    print(f"  Distance traveled: {dist:.2f} m")
    print(f"  Start position:    ({x_new[0]:.3f}, {y_new[0]:.3f}, {trajectory[0,3]:.3f}) m")
    print(f"  End   position:    ({x_new[-1]:.3f}, {y_new[-1]:.3f}, {trajectory[-1,3]:.3f}) m")
    print(f"  Mean speed (2D):   {np.mean(speed_2d):.3f} m/s")
    print(f"  Max  speed (2D):   {np.max(speed_2d):.3f} m/s")
    print(f"  USBL range error:  mean={np.mean(range_errors):.3f} m  "
          f"RMS={np.sqrt(np.mean(range_errors**2)):.3f} m  "
          f"max={np.max(range_errors):.3f} m")

    plt.savefig('ekf_fusion_result.svg', bbox_inches='tight')
    print(f"\n  Plot saved: ekf_fusion_result.svg")
    plt.show()
