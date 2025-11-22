#!/usr/bin/env python3
import socket
import pickle
import threading
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# Config
UDP_PORT = 5555
PLOT_HISTORY = 150
TILT_ANGLE = np.deg2rad(2.0)

# Data buffer
data_buffer = {
    'time': deque(maxlen=PLOT_HISTORY),
    'true_pos': deque(maxlen=PLOT_HISTORY),
    'est_pos': deque(maxlen=PLOT_HISTORY),
    'meas_pos': [],
    'error': deque(maxlen=PLOT_HISTORY),
    'true_depth': deque(maxlen=PLOT_HISTORY),
    'est_depth': deque(maxlen=PLOT_HISTORY),
    'meas_depth': deque(maxlen=PLOT_HISTORY)
}
data_lock = threading.Lock()
running = True

def udp_receiver():
    """Thread che riceve dati UDP"""
    global running
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', UDP_PORT))
    sock.settimeout(1.0)
    
    print(f"UDP Viewer listening on 0.0.0.0:{UDP_PORT}")
    
    packet_count = 0
    while running:
        try:
            packet, _ = sock.recvfrom(65535)
            data = pickle.loads(packet)
            packet_count += 1
            
            if packet_count % 20 == 0:
                print(f"Received {packet_count} packets")
            
            with data_lock:
                data_buffer['time'] = deque(data['time'], maxlen=PLOT_HISTORY)
                data_buffer['true_pos'] = deque([np.array(p) for p in data['true_pos']], maxlen=PLOT_HISTORY)
                data_buffer['est_pos'] = deque([np.array(p) for p in data['est_pos']], maxlen=PLOT_HISTORY)
                data_buffer['meas_pos'] = [(t, np.array(p)) for t, p in data['meas_pos']]
                data_buffer['error'] = deque(data['error'], maxlen=PLOT_HISTORY)
                data_buffer['true_depth'] = deque(data['true_depth'], maxlen=PLOT_HISTORY)
                data_buffer['est_depth'] = deque(data['est_depth'], maxlen=PLOT_HISTORY)
                data_buffer['meas_depth'] = deque(data['meas_depth'], maxlen=PLOT_HISTORY)
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[UDP] Error receiving: {e}")
    
    sock.close()

def init_plot():
    """Initialize all plot elements"""
    # 3D plot
    if len(data_buffer['true_pos']) > 0:
        true_arr = np.array(list(data_buffer['true_pos']))
        line_true.set_data(true_arr[:,0], true_arr[:,1])
        line_true.set_3d_properties(true_arr[:,2])
    
    if len(data_buffer['est_pos']) > 0:
        est_arr = np.array(list(data_buffer['est_pos']))
        line_est.set_data(est_arr[:,0], est_arr[:,1])
        line_est.set_3d_properties(est_arr[:,2])
    
    return line_true, line_est, scatter_meas, line_true_xy, line_est_xy, scatter_meas_xy, \
           line_err, line_x, line_y, line_z, line_true_depth, line_est_depth, line_meas_depth

def update_plot(frame):
    """Update plot with new data"""
    with data_lock:
        if len(data_buffer['time']) == 0:
            return line_true, line_est, scatter_meas, line_true_xy, line_est_xy, scatter_meas_xy, \
                   line_err, line_x, line_y, line_z, line_true_depth, line_est_depth, line_meas_depth
        
        ts = list(data_buffer['time'])
        true_pos = list(data_buffer['true_pos'])
        est_pos = list(data_buffer['est_pos'])
        meas_pos = data_buffer['meas_pos']
        errors = list(data_buffer['error'])
        true_depth = list(data_buffer['true_depth'])
        est_depth = list(data_buffer['est_depth'])
        meas_depth = list(data_buffer['meas_depth'])
    
    true_arr = np.array(true_pos)
    est_arr = np.array(est_pos)
    
    # 3D trajectory
    line_true.set_data(true_arr[:,0], true_arr[:,1])
    line_true.set_3d_properties(true_arr[:,2])
    line_est.set_data(est_arr[:,0], est_arr[:,1])
    line_est.set_3d_properties(est_arr[:,2])
    
    if len(meas_pos) > 0:
        meas_arr = np.array([p for _, p in meas_pos])
        scatter_meas._offsets3d = (meas_arr[:,0], meas_arr[:,1], meas_arr[:,2])
    else:
        scatter_meas._offsets3d = ([], [], [])
    
    # XY view
    line_true_xy.set_data(true_arr[:,0], true_arr[:,1])
    line_est_xy.set_data(est_arr[:,0], est_arr[:,1])
    
    if len(meas_pos) > 0:
        scatter_meas_xy.set_offsets(meas_arr[:,:2])
    else:
        scatter_meas_xy.set_offsets(np.empty((0, 2)))
    
    all_x = np.concatenate([true_arr[:,0], est_arr[:,0]])
    all_y = np.concatenate([true_arr[:,1], est_arr[:,1]])
    x_margin = (all_x.max() - all_x.min()) * 0.1 + 1
    y_margin = (all_y.max() - all_y.min()) * 0.1 + 1
    ax2.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
    ax2.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
    
    # Error
    line_err.set_data(ts, errors)
    ax3.collections.clear()
    ax3.fill_between(ts, errors, 0, alpha=0.2, color='red')
    if len(errors) > 0:
        error_max = max(errors)
        ax3.set_ylim(0, error_max * 1.15)
    ax3.set_xlim(min(ts), max(ts))
    
    # Position components
    line_x.set_data(ts, est_arr[:,0])
    line_y.set_data(ts, est_arr[:,1])
    line_z.set_data(ts, est_arr[:,2])
    
    all_pos = np.concatenate([est_arr[:,0], est_arr[:,1], est_arr[:,2]])
    pos_margin = (all_pos.max() - all_pos.min()) * 0.1 + 0.5
    ax4.set_ylim(all_pos.min() - pos_margin, all_pos.max() + pos_margin)
    ax4.set_xlim(min(ts), max(ts))
    
    # Depth
    if len(true_depth) > 0:
        line_true_depth.set_data(ts, true_depth)
        line_est_depth.set_data(ts, est_depth)
        line_meas_depth.set_data(ts, meas_depth)
        
        all_depths = true_depth + est_depth + meas_depth
        depth_min = min(all_depths)
        depth_max = max(all_depths)
        depth_margin = (depth_max - depth_min) * 0.15 + 0.2
        
        ax5.set_ylim(depth_max + depth_margin, depth_min - depth_margin)
        ax5.set_xlim(min(ts), max(ts))
    
    return line_true, line_est, scatter_meas, line_true_xy, line_est_xy, scatter_meas_xy, \
           line_err, line_x, line_y, line_z, line_true_depth, line_est_depth, line_meas_depth

def main():
    global running, line_true, line_est, scatter_meas, line_true_xy, line_est_xy, scatter_meas_xy
    global line_err, line_x, line_y, line_z, line_true_depth, line_est_depth, line_meas_depth
    global ax2, ax3, ax4, ax5
    
    # Start receiver thread
    receiver = threading.Thread(target=udp_receiver, daemon=True)
    receiver.start()
    
    # Setup figure with GridSpec (same as original)
    fig = plt.figure(figsize=(18, 10))
    
    gs = GridSpec(4, 2,
                  width_ratios=[2, 1],
                  height_ratios=[1, 1, 1, 1],
                  left=0.05, right=0.98,
                  top=0.96, bottom=0.05,
                  wspace=0.25, hspace=0.30)
    
    # 3D trajectory (left, spans all rows)
    ax1 = fig.add_subplot(gs[:, 0], projection='3d')
    line_true, = ax1.plot([], [], [], 'b-', label='Ground Truth', linewidth=2.5, alpha=0.8)
    line_est, = ax1.plot([], [], [], 'r-', label='KF Estimate', linewidth=2.5, alpha=0.9)
    scatter_meas = ax1.scatter([], [], [], c='lime', s=80, marker='o', 
                               edgecolors='darkgreen', linewidths=2, label='USBL Fix', alpha=0.9)
    
    ax1.set_xlabel('X [m]', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
    ax1.set_zlabel('Z [m]', fontsize=11, fontweight='bold')
    ax1.set_title(f'3D Trajectory - Tilted Circle ({np.rad2deg(TILT_ANGLE):.0f}°)',
                  fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax1.grid(True, alpha=0.3)
    ax1.set_facecolor('#f0f0f0')
    ax1.set_xlim(-12, 12)
    ax1.set_ylim(-12, 12)
    ax1.set_zlim(-8, -2)
    
    # XY top view
    ax2 = fig.add_subplot(gs[0, 1])
    line_true_xy, = ax2.plot([], [], 'b-', label='True', linewidth=2, alpha=0.7)
    line_est_xy, = ax2.plot([], [], 'r-', label='KF Est', linewidth=2, alpha=0.8)
    scatter_meas_xy = ax2.scatter([], [], c='lime', s=50, marker='o', 
                                  edgecolors='darkgreen', linewidths=1.5, label='USBL', alpha=0.9)
    ax2.set_xlabel('X [m]', fontsize=9, fontweight='bold')
    ax2.set_ylabel('Y [m]', fontsize=9, fontweight='bold')
    ax2.set_title('XY Top View', fontsize=10, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=7, framealpha=0.9)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_facecolor('#f8f8f8')
    ax2.axis('equal')
    
    # Position error
    ax3 = fig.add_subplot(gs[1, 1])
    line_err, = ax3.plot([], [], 'r-', linewidth=2.5, alpha=0.8)
    ax3.set_xlabel('Time [s]', fontsize=9, fontweight='bold')
    ax3.set_ylabel('Error [m]', fontsize=9, fontweight='bold')
    ax3.set_title('Position Error Norm', fontsize=10, fontweight='bold')
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.set_facecolor('#f8f8f8')
    ax3.set_ylim(0, 2)
    
    # Position components
    ax4 = fig.add_subplot(gs[2, 1])
    line_x, = ax4.plot([], [], 'r-', label='X', linewidth=2, alpha=0.8)
    line_y, = ax4.plot([], [], 'g-', label='Y', linewidth=2, alpha=0.8)
    line_z, = ax4.plot([], [], 'b-', label='Z', linewidth=2, alpha=0.8)
    ax4.set_xlabel('Time [s]', fontsize=9, fontweight='bold')
    ax4.set_ylabel('Position [m]', fontsize=9, fontweight='bold')
    ax4.set_title('Position Components', fontsize=10, fontweight='bold')
    ax4.legend(loc='upper right', fontsize=7, framealpha=0.9)
    ax4.grid(True, alpha=0.3, linestyle='--')
    ax4.set_facecolor('#f8f8f8')
    
    # Depth sensor data
    ax5 = fig.add_subplot(gs[3, 1])
    line_true_depth, = ax5.plot([], [], 'b-', label='True Depth', linewidth=2.5, alpha=0.8)
    line_est_depth, = ax5.plot([], [], 'r-', label='KF Estimate', linewidth=2.5, alpha=0.9)
    line_meas_depth, = ax5.plot([], [], 'go', label='Depth Sensor', markersize=4, alpha=0.6)
    ax5.set_xlabel('Time [s]', fontsize=9, fontweight='bold')
    ax5.set_ylabel('Depth [m]', fontsize=9, fontweight='bold')
    ax5.set_title('Depth Measurements', fontsize=10, fontweight='bold')
    ax5.legend(loc='upper right', fontsize=7, framealpha=0.9)
    ax5.grid(True, alpha=0.3, linestyle='--')
    ax5.set_facecolor('#f8f8f8')
    ax5.invert_yaxis()
    
    plt.tight_layout()
    
    # Use FuncAnimation for smooth updates
    ani = FuncAnimation(fig, update_plot, init_func=init_plot,
                       interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nShutting down viewer...")
        running = False

if __name__ == "__main__":
    main()