#!/usr/bin/env python3
import socket
import pickle
import threading
import numpy as np
from collections import deque
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Config
UDP_PORT = 5555
PLOT_HISTORY = 150

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

def main():
    global running
    
    # Start receiver thread
    receiver = threading.Thread(target=udp_receiver, daemon=True)
    receiver.start()
    
    # Setup plot
    fig = plt.figure(figsize=(16, 10))
    
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Trajectory')
    
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('XY View')
    ax2.grid(True)
    
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Error [m]')
    ax3.set_title('Position Error')
    ax3.grid(True)
    
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Position [m]')
    ax4.set_title('Position Components')
    ax4.grid(True)
    
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Depth [m]')
    ax5.set_title('Depth Tracking')
    ax5.grid(True)
    ax5.invert_yaxis()
    
    # Init lines
    line_true, = ax1.plot([], [], [], 'g-', alpha=0.6, linewidth=1.5, label='True')
    line_est, = ax1.plot([], [], [], 'b-', linewidth=2, label='Est')
    scatter_meas = ax1.scatter([], [], [], c='red', s=40, marker='x', label='USBL')
    
    line_true_xy, = ax2.plot([], [], 'g-', alpha=0.6, linewidth=1.5, label='True')
    line_est_xy, = ax2.plot([], [], 'b-', linewidth=2, label='Est')
    scatter_meas_xy = ax2.scatter([], [], c='red', s=40, marker='x', label='USBL')
    
    line_err, = ax3.plot([], [], 'r-', linewidth=1.5)
    
    line_x, = ax4.plot([], [], 'r-', label='X', linewidth=1.5)
    line_y, = ax4.plot([], [], 'g-', label='Y', linewidth=1.5)
    line_z, = ax4.plot([], [], 'b-', label='Z', linewidth=1.5)
    
    line_true_depth, = ax5.plot([], [], 'g--', alpha=0.6, label='True', linewidth=1.5)
    line_est_depth, = ax5.plot([], [], 'b-', label='Est', linewidth=2)
    line_meas_depth, = ax5.plot([], [], 'rx', markersize=4, label='Meas')
    
    ax1.legend()
    ax2.legend()
    ax4.legend()
    ax5.legend()
    
    plt.tight_layout()
    
    try:
        while True:
            with data_lock:
                if len(data_buffer['time']) == 0:
                    plt.pause(0.1)
                    continue
                
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
            
            # 3D plot
            line_true.set_data(true_arr[:,0], true_arr[:,1])
            line_true.set_3d_properties(true_arr[:,2])
            line_est.set_data(est_arr[:,0], est_arr[:,1])
            line_est.set_3d_properties(est_arr[:,2])
            
            if len(meas_pos) > 0:
                meas_arr = np.array([p for _, p in meas_pos])
                scatter_meas._offsets3d = (meas_arr[:,0], meas_arr[:,1], meas_arr[:,2])
            
            # XY view
            line_true_xy.set_data(true_arr[:,0], true_arr[:,1])
            line_est_xy.set_data(est_arr[:,0], est_arr[:,1])
            if len(meas_pos) > 0:
                scatter_meas_xy.set_offsets(meas_arr[:,:2])
            
            all_x = np.concatenate([true_arr[:,0], est_arr[:,0]])
            all_y = np.concatenate([true_arr[:,1], est_arr[:,1]])
            x_margin = (all_x.max() - all_x.min()) * 0.1 + 1
            y_margin = (all_y.max() - all_y.min()) * 0.1 + 1
            ax2.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
            ax2.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
            
            # Error
            line_err.set_data(ts, errors)
            ax3.collections[:] = []
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
            
            plt.pause(0.05)
            
    except KeyboardInterrupt:
        print("\nShutting down viewer...")
        running = False
    
    plt.close('all')

if __name__ == "__main__":
    main()