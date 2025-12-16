#!/usr/bin/env python3
"""
Kalman Filter Sensor Fusion - VIDEO RECORDING VERSION
Runs simulation first, then creates video from complete dataset
"""
import time
import threading
import numpy as np
import math
import os
import csv
import sys
import platform
from datetime import datetime

from lib.KF.KF_profond import KF
from lib.KF.RMatrix import Rxyz

# ---- Matplotlib and Video ----
MATPLOTLIB_AVAILABLE = False
VIDEO_AVAILABLE = False
try:
    import matplotlib
    if platform.system() == 'Darwin':
        matplotlib.use('Agg')
    
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
    
    try:
        from matplotlib.animation import FFMpegWriter
        VIDEO_AVAILABLE = True
    except ImportError:
        VIDEO_AVAILABLE = False
            
except ImportError:
    MATPLOTLIB_AVAILABLE = False

# ==================== CONFIG ====================
CONTROL_LOOP_MS = 50
SIMULATION_DURATION = 30.0  # Run simulation for 60 seconds
VIDEO_FPS = 20  # 20 fps for smooth video (matches 50ms control loop)
VIDEO_DPI = 100
VIDEO_BITRATE = 2000

# Simulation parameters
RADIUS = 10.0
OMEGA = 0.15
TILT_ANGLE = np.deg2rad(2.0)
USBL_REQUEST_INTERVAL = 2.0

SIGMA_ACC = 0.03
SIGMA_POS = 0.15
SIGMA_DEPTH = 0.05
SIGMA_YAW = np.deg2rad(1.0)

LOG_DIR = "log_transponder"
VIDEO_DIR = "videos"

# ==================== HELPER FUNCTIONS ====================
def compute_tilted_circle_position(theta, radius, tilt_angle, z_center=-5.0):
    x_flat = radius * math.cos(theta)
    y_flat = radius * math.sin(theta)
    z_flat = z_center
    
    x = x_flat * math.cos(tilt_angle) - (z_flat) * math.sin(tilt_angle)
    y = y_flat
    z = x_flat * math.sin(tilt_angle) + (z_flat) * math.cos(tilt_angle)
    
    return np.array([[x], [y], [z]])

def compute_tilted_circle_acceleration(theta, radius, omega, tilt_angle):
    a_x_flat = -radius * (omega ** 2) * math.cos(theta)
    a_y_flat = -radius * (omega ** 2) * math.sin(theta)
    a_z_flat = 0.0
    
    a_x = a_x_flat * math.cos(tilt_angle) - a_z_flat * math.sin(tilt_angle)
    a_y = a_y_flat
    a_z = a_x_flat * math.sin(tilt_angle) + a_z_flat * math.cos(tilt_angle)
    
    return np.array([[a_x], [a_y], [a_z]])

def generate_full_circle(radius, tilt_angle, z_center=-5.0, num_points=200):
    thetas = np.linspace(0, 2*np.pi, num_points)
    positions = []
    for theta in thetas:
        pos = compute_tilted_circle_position(theta, radius, tilt_angle, z_center)
        positions.append(pos.flatten())
    return np.array(positions)

# ==================== SIMULATION ====================
def run_simulation():
    """Run complete simulation and return all data"""
    print("=" * 80)
    print("PHASE 1: RUNNING SIMULATION")
    print("=" * 80)
    print("Duration: {:.1f} seconds".format(SIMULATION_DURATION))
    print("Control Loop: {}ms".format(CONTROL_LOOP_MS))
    print("=" * 80)
    
    # Initialize Kalman Filter
    Q = np.eye(3) * 0.05
    R1 = np.eye(3) * (SIGMA_ACC ** 2)
    R2 = np.eye(3) * (SIGMA_POS ** 2)
    R3 = np.array([[SIGMA_DEPTH ** 2]])
    
    C1 = np.array([[0,0,0, 0,0,0, 1,0,0], [0,0,0, 0,0,0, 0,1,0], [0,0,0, 0,0,0, 0,0,1]], dtype=float)
    C2 = np.array([[1,0,0, 0,0,0, 0,0,0], [0,1,0, 0,0,0, 0,0,0], [0,0,1, 0,0,0, 0,0,0]], dtype=float)
    C3 = np.array([[0,0,1, 0,0,0, 0,0,0]], dtype=float)
    
    kf = KF(Q, R1, R2, R3, C1, C2, C3, T=CONTROL_LOOP_MS/1000.0)
    
    p_init = compute_tilted_circle_position(0, RADIUS, TILT_ANGLE)
    kf.setX(np.array([[p_init[0,0], p_init[1,0], p_init[2,0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]).T)
    
    # Data storage
    data = {
        'time': [],
        'true_pos': [],
        'est_pos': [],
        'meas_pos': [],
        'error': [],
        'true_depth': [],
        'est_depth': [],
        'meas_depth': []
    }
    
    loop_time = CONTROL_LOOP_MS / 1000.0
    num_iterations = int(SIMULATION_DURATION / loop_time)
    last_usbl_time = -USBL_REQUEST_INTERVAL  # Force first USBL immediately
    
    print("\nRunning {} iterations...".format(num_iterations))
    
    for iteration in range(num_iterations):
        t_sim = iteration * loop_time
        theta = OMEGA * t_sim
        
        # True position
        p_true_vec = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
        p_true = p_true_vec.flatten()
        
        # Simulate IMU
        a_world = compute_tilted_circle_acceleration(theta, RADIUS, OMEGA, TILT_ANGLE)
        roll = 0.0
        pitch = 0.0
        yaw_true = theta + math.pi / 2.0
        
        from lib.KF.RMatrix import Rxyz
        Rwb = Rxyz(roll, pitch, yaw_true)
        a_body_true = Rwb.T @ a_world
        a_body_meas = a_body_true + np.random.normal(0.0, SIGMA_ACC, size=(3, 1))
        yaw_meas = yaw_true + np.random.normal(0.0, SIGMA_YAW)
        
        # Simulate USBL (event-driven)
        usbl_fix_used = False
        if (t_sim - last_usbl_time) >= USBL_REQUEST_INTERVAL:
            last_usbl_time = t_sim
            p_meas = p_true_vec + np.random.normal(0.0, SIGMA_POS, size=(3, 1))
            kf.update(p_meas, roll, pitch, yaw_meas, mode='usbl', dt_override=loop_time)
            data['meas_pos'].append((t_sim, p_meas.flatten()))
            usbl_fix_used = True
        else:
            kf.update(a_body_meas, roll, pitch, yaw_meas, mode='imu', dt_override=loop_time)
        
        # Simulate Depth (every 3 iterations)
        depth_true = -p_true[2]
        depth_meas = depth_true + np.random.normal(0.0, SIGMA_DEPTH)
        if iteration % 3 == 0:
            kf.update(np.array([[-depth_meas]]), roll, pitch, yaw_meas, mode='depth', dt_override=0.0)
        
        # Get estimates
        state = kf.getX()
        pos_est = state[0:3, 0]
        error_norm = np.linalg.norm(pos_est - p_true)
        
        # Store data
        data['time'].append(t_sim)
        data['true_pos'].append(p_true.copy())
        data['est_pos'].append(pos_est.copy())
        data['error'].append(error_norm)
        data['true_depth'].append(depth_true)
        data['est_depth'].append(-pos_est[2])
        data['meas_depth'].append(depth_meas)
        
        if (iteration + 1) % 200 == 0:
            print("  Progress: {}/{} ({:.1f}%)".format(
                iteration + 1, num_iterations, 100 * (iteration + 1) / num_iterations))
    
    print("\n✓ Simulation complete!")
    print("  Total data points: {}".format(len(data['time'])))
    print("  Time range: {:.2f} - {:.2f} seconds".format(data['time'][0], data['time'][-1]))
    print("  USBL fixes: {}".format(len(data['meas_pos'])))
    print("  Mean error: {:.4f} m".format(np.mean(data['error'])))
    
    return data

# ==================== VIDEO CREATION ====================
def create_video(data):
    """Create video from complete dataset"""
    print("\n" + "=" * 80)
    print("PHASE 2: CREATING VIDEO")
    print("=" * 80)
    print("Video FPS: {}".format(VIDEO_FPS))
    print("Video DPI: {}".format(VIDEO_DPI))
    print("Bitrate: {}kbps".format(VIDEO_BITRATE))
    print("=" * 80)
    
    if not MATPLOTLIB_AVAILABLE or not VIDEO_AVAILABLE:
        print("ERROR: Video creation not available")
        return
    
    # Calculate global axis limits from complete dataset
    true_arr = np.array(data['true_pos'])
    est_arr = np.array(data['est_pos'])
    all_pos = np.concatenate([true_arr, est_arr])
    
    # Position limits with margin
    x_min, x_max = all_pos[:, 0].min(), all_pos[:, 0].max()
    y_min, y_max = all_pos[:, 1].min(), all_pos[:, 1].max()
    z_min, z_max = all_pos[:, 2].min(), all_pos[:, 2].max()
    margin = 2.0
    
    pos_limits = {
        'x': (x_min - margin, x_max + margin),
        'y': (y_min - margin, y_max + margin),
        'z': (z_min - margin, z_max + margin)
    }
    
    # Error limits
    error_max = max(data['error']) * 1.2
    
    # Depth limits
    all_depths = data['true_depth'] + data['est_depth'] + data['meas_depth']
    depth_min = min(all_depths)
    depth_max = max(all_depths)
    depth_margin = max((depth_max - depth_min) * 0.15, 0.5)
    
    print("\nCalculated axis limits:")
    print("  X: [{:.2f}, {:.2f}]".format(pos_limits['x'][0], pos_limits['x'][1]))
    print("  Y: [{:.2f}, {:.2f}]".format(pos_limits['y'][0], pos_limits['y'][1]))
    print("  Z: [{:.2f}, {:.2f}]".format(pos_limits['z'][0], pos_limits['z'][1]))
    print("  Error: [0, {:.4f}]".format(error_max))
    print("  Depth: [{:.2f}, {:.2f}]".format(depth_min - depth_margin, depth_max + depth_margin))
    
    # Create figure
    fig = plt.figure(figsize=(16, 10))
    
    # 3D trajectory
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.set_xlabel('X [m]', fontsize=10)
    ax1.set_ylabel('Y [m]', fontsize=10)
    ax1.set_zlabel('Z [m]', fontsize=10)
    ax1.set_title('3D Trajectory', fontsize=12, fontweight='bold')
    ax1.set_xlim(pos_limits['x'])
    ax1.set_ylim(pos_limits['y'])
    ax1.set_zlim(pos_limits['z'])
    
    # 2D trajectory
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.set_xlabel('X [m]', fontsize=10)
    ax2.set_ylabel('Y [m]', fontsize=10)
    ax2.set_title('Top View (X-Y)', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim(pos_limits['x'])
    ax2.set_ylim(pos_limits['y'])
    
    # Error plot
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.set_xlabel('Time [s]', fontsize=10)
    ax3.set_ylabel('Position Error [m]', fontsize=10)
    ax3.set_title('Estimation Error', fontsize=12, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.set_xlim(data['time'][0], data['time'][-1])
    ax3.set_ylim(0, error_max)
    
    # Position components
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.set_xlabel('Time [s]', fontsize=10)
    ax4.set_ylabel('Position [m]', fontsize=10)
    ax4.set_title('Position Components', fontsize=12, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.set_xlim(data['time'][0], data['time'][-1])
    ax4.set_ylim(pos_limits['z'][0], pos_limits['x'][1])  # Use full range
    
    # Depth plot
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.set_xlabel('Time [s]', fontsize=10)
    ax5.set_ylabel('Depth [m]', fontsize=10)
    ax5.set_title('Depth Tracking', fontsize=12, fontweight='bold')
    ax5.grid(True, alpha=0.3)
    ax5.set_xlim(data['time'][0], data['time'][-1])
    ax5.set_ylim(depth_max + depth_margin, depth_min - depth_margin)
    
    # Statistics panel
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.axis('off')
    ax6.set_title('Statistics', fontsize=12, fontweight='bold')
    
    # Ground truth reference
    full_circle = generate_full_circle(RADIUS, TILT_ANGLE)
    ax1.plot(full_circle[:, 0], full_circle[:, 1], full_circle[:, 2], 
            'k--', alpha=0.2, linewidth=1.5, label='Ground Truth')
    ax2.plot(full_circle[:, 0], full_circle[:, 1], 
            'k--', alpha=0.2, linewidth=1.5, label='Ground Truth')
    
    # Initialize plot lines
    line_true_3d, = ax1.plot([], [], [], 'b-', label='True', linewidth=2)
    line_est_3d, = ax1.plot([], [], [], 'r-', label='Estimated', linewidth=2)
    scatter_meas_3d = ax1.scatter([], [], [], c='g', marker='o', s=50, label='USBL', alpha=0.7)
    
    line_true_2d, = ax2.plot([], [], 'b-', label='True', linewidth=2)
    line_est_2d, = ax2.plot([], [], 'r-', label='Estimated', linewidth=2)
    scatter_meas_2d = ax2.scatter([], [], c='g', marker='o', s=50, label='USBL', alpha=0.7)
    
    line_error, = ax3.plot([], [], 'r-', linewidth=2)
    
    line_pos_x, = ax4.plot([], [], 'r-', label='X', linewidth=2)
    line_pos_y, = ax4.plot([], [], 'g-', label='Y', linewidth=2)
    line_pos_z, = ax4.plot([], [], 'b-', label='Z', linewidth=2)
    
    line_depth_true, = ax5.plot([], [], 'b-', label='True', linewidth=2)
    line_depth_est, = ax5.plot([], [], 'r-', label='Estimated', linewidth=2)
    line_depth_meas, = ax5.plot([], [], 'go', label='Measured', markersize=4)
    
    stats_text = ax6.text(0.1, 0.5, '', transform=ax6.transAxes, 
                         fontsize=11, verticalalignment='center', family='monospace',
                         bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    
    for ax in [ax1, ax2, ax3, ax4, ax5]:
        ax.legend(loc='upper right', fontsize=9)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    # Setup video writer
    os.makedirs(VIDEO_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    video_filename = os.path.join(VIDEO_DIR, "kf_simulation_{}.mp4".format(timestamp))
    
    metadata = {'title': 'Kalman Filter Sensor Fusion'}
    writer = FFMpegWriter(fps=VIDEO_FPS, metadata=metadata, bitrate=VIDEO_BITRATE)
    
    # Calculate frame indices
    total_frames = len(data['time'])
    frame_step = max(1, total_frames // (int(SIMULATION_DURATION * VIDEO_FPS)))
    frame_indices = list(range(0, total_frames, frame_step))
    
    print("\nCreating video...")
    print("  Total frames to render: {}".format(len(frame_indices)))
    print("  Frame step: {} (showing every {}th data point)".format(frame_step, frame_step))
    print("  Output file: {}".format(video_filename))
    
    with writer.saving(fig, video_filename, dpi=VIDEO_DPI):
        for frame_num, idx in enumerate(frame_indices):
            # Get data up to this point
            ts = data['time'][:idx+1]
            true_pos = true_arr[:idx+1]
            est_pos = est_arr[:idx+1]
            errors = data['error'][:idx+1]
            true_depth = data['true_depth'][:idx+1]
            est_depth = data['est_depth'][:idx+1]
            meas_depth = data['meas_depth'][:idx+1]
            
            current_time = data['time'][idx]
            
            # Update 3D trajectory
            line_true_3d.set_data(true_pos[:, 0], true_pos[:, 1])
            line_true_3d.set_3d_properties(true_pos[:, 2])
            line_est_3d.set_data(est_pos[:, 0], est_pos[:, 1])
            line_est_3d.set_3d_properties(est_pos[:, 2])
            
            # USBL measurements up to current time
            meas_up_to_now = [p for t, p in data['meas_pos'] if t <= current_time]
            if meas_up_to_now:
                meas_arr = np.array(meas_up_to_now)
                scatter_meas_3d._offsets3d = (meas_arr[:, 0], meas_arr[:, 1], meas_arr[:, 2])
                scatter_meas_2d.set_offsets(meas_arr[:, :2])
            
            # Update 2D trajectory
            line_true_2d.set_data(true_pos[:, 0], true_pos[:, 1])
            line_est_2d.set_data(est_pos[:, 0], est_pos[:, 1])
            
            # Update error
            line_error.set_data(ts, errors)
            
            # Update position components
            line_pos_x.set_data(ts, est_pos[:, 0])
            line_pos_y.set_data(ts, est_pos[:, 1])
            line_pos_z.set_data(ts, est_pos[:, 2])
            
            # Update depth
            line_depth_true.set_data(ts, true_depth)
            line_depth_est.set_data(ts, est_depth)
            line_depth_meas.set_data(ts, meas_depth)
            
            # Update statistics
            stats_str = """
Time: {:.2f} s

Position Error:
  Current: {:.4f} m
  Mean: {:.4f} m
  Max: {:.4f} m
  Min: {:.4f} m

Data Points:
  Total: {}
  USBL Fixes: {}
            """.format(
                current_time,
                errors[-1],
                np.mean(errors),
                max(errors),
                min(errors),
                len(ts),
                len(meas_up_to_now)
            )
            stats_text.set_text(stats_str)
            
            # Update title
            fig.suptitle('Kalman Filter Sensor Fusion - Time: {:.2f}s'.format(current_time), 
                        fontsize=14, fontweight='bold')
            
            writer.grab_frame()
            
            if (frame_num + 1) % 50 == 0:
                print("  Progress: {}/{} frames ({:.1f}%)".format(
                    frame_num + 1, len(frame_indices), 100 * (frame_num + 1) / len(frame_indices)))
    
    file_size_mb = os.path.getsize(video_filename) / (1024 * 1024)
    duration = len(frame_indices) / VIDEO_FPS
    
    print("\n✓ Video created successfully!")
    print("  File: {}".format(video_filename))
    print("  Size: {:.2f} MB".format(file_size_mb))
    print("  Duration: {:.1f} seconds".format(duration))
    print("  Frames: {}".format(len(frame_indices)))
    
    plt.close(fig)

# ==================== MAIN ====================
def main():
    print("=" * 80)
    print("KALMAN FILTER SENSOR FUSION - VIDEO CREATOR")
    print("=" * 80)
    print("This script runs in two phases:")
    print("  1. Run complete simulation ({:.1f}s)".format(SIMULATION_DURATION))
    print("  2. Create video from data")
    print("=" * 80)
    print()
    
    # Phase 1: Run simulation
    data = run_simulation()
    
    # Phase 2: Create video
    create_video(data)
    
    print("\n" + "=" * 80)
    print("COMPLETE!")
    print("=" * 80)

if __name__ == "__main__":
    main()