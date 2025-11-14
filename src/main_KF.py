#!/usr/bin/env python3
import time
import threading
import numpy as np
import math
from datetime import datetime
from collections import deque

from lib.KF.KF import KF
from lib.KF.RMatrix import Rxyz

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("[WARNING] matplotlib not available, plotting disabled")

# ==================== CONFIG ====================
CONTROL_LOOP_MS = 50  # 50ms = 20Hz (era 20ms nella simulazione)
ENABLE_PLOTTING = True and MATPLOTLIB_AVAILABLE
USE_REAL_SENSORS = False  # Set True per sensori reali

# Circular trajectory params
RADIUS = 10.0
OMEGA = 0.15

# Sensor noise
SIGMA_ACC = 0.03
SIGMA_POS = 0.15
SIGMA_YAW = np.deg2rad(1.0)

# USBL position fix every N iterations
USBL_FIX_EVERY = 5

# Plot history length
PLOT_HISTORY = 200

# ==================== GLOBALS ====================
sensor_data = {
    'imu': {'euler': [0.0, 0.0, 0.0], 'acc_body': [0.0, 0.0, 0.0], 'valid': False},
    'usbl': {'pos_world': [0.0, 0.0, 0.0], 'valid': False},
    'depth': {'depth': 0.0, 'valid': False}
}
data_lock = threading.Lock()
running = True

# Simulation time (shared)
sim_time = {'t': 0.0}
sim_lock = threading.Lock()

# KF state
kf_state = {
    'pos': [0.0, 0.0, 0.0],
    'vel': [0.0, 0.0, 0.0],
    'acc': [0.0, 0.0, 0.0]
}
kf_lock = threading.Lock()

# Plot data
plot_data = {
    'time': deque(maxlen=PLOT_HISTORY),
    'true_pos': deque(maxlen=PLOT_HISTORY),
    'est_pos': deque(maxlen=PLOT_HISTORY),
    'meas_pos': [],  # Store only valid fixes (not a deque)
    'error': deque(maxlen=PLOT_HISTORY)
}
plot_lock = threading.Lock()

# ==================== SIMULATION THREADS ====================
def imu_simulation_thread():
    """Simulates IMU data following circular trajectory"""
    global running
    
    while running:
        with sim_lock:
            t = sim_time['t']
        
        theta = OMEGA * t
        
        # Ground truth acceleration (world frame)
        a_world = np.array([
            [-RADIUS * (OMEGA ** 2) * math.cos(theta)],
            [-RADIUS * (OMEGA ** 2) * math.sin(theta)],
            [0.0]
        ])
        
        # Orientation
        roll = 0.0
        pitch = 0.0
        yaw_true = theta + math.pi / 2.0
        
        # Transform to body frame
        Rwb = Rxyz(roll, pitch, yaw_true)
        a_body_true = Rwb.T @ a_world
        
        # Add noise
        a_body_meas = a_body_true + np.random.normal(0.0, SIGMA_ACC, size=(3,1))
        yaw_meas = yaw_true + np.random.normal(0.0, SIGMA_YAW)
        
        with data_lock:
            sensor_data['imu']['euler'] = [roll, pitch, yaw_meas]
            sensor_data['imu']['acc_body'] = [a_body_meas[0,0], a_body_meas[1,0], a_body_meas[2,0]]
            sensor_data['imu']['valid'] = True
        
        time.sleep(0.005)

def usbl_simulation_thread():
    """Simulates USBL position - main loop decides when to use it as fix"""
    global running
    
    while running:
        with sim_lock:
            t = sim_time['t']
        
        theta = OMEGA * t
        
        # Ground truth position (world frame)
        p_true = np.array([
            [RADIUS * math.cos(theta)],
            [RADIUS * math.sin(theta)],
            [0.0]
        ])
        
        # Add measurement noise and update
        p_meas = p_true + np.random.normal(0.0, SIGMA_POS, size=(3,1))
        with data_lock:
            sensor_data['usbl']['pos_world'] = [p_meas[0,0], p_meas[1,0], p_meas[2,0]]
            sensor_data['usbl']['valid'] = True
        
        time.sleep(0.01)

def depth_simulation_thread():
    """Simulates depth sensor (not used in KF for now)"""
    global running
    
    while running:
        with sim_lock:
            t = sim_time['t']
        
        theta = OMEGA * t
        depth_sim = 0.0  # Circular trajectory at constant depth
        
        with data_lock:
            sensor_data['depth']['depth'] = depth_sim
            sensor_data['depth']['valid'] = True
        
        time.sleep(0.1)

# ==================== PLOTTING ====================
def plot_thread():
    """Real-time plotting thread with improved layout"""
    if not ENABLE_PLOTTING:
        return
    
    plt.ion()
    fig = plt.figure(figsize=(16, 9))
    
    # Create custom layout: large 3D on left, smaller plots on right
    from matplotlib.gridspec import GridSpec
    gs = GridSpec(3, 2, figure=fig, width_ratios=[2, 1], height_ratios=[1, 1, 1],
                  left=0.05, right=0.98, top=0.95, bottom=0.06, wspace=0.25, hspace=0.35)
    
    # Large 3D trajectory plot (left side, spans all rows)
    ax1 = fig.add_subplot(gs[:, 0], projection='3d')
    line_true, = ax1.plot([], [], [], 'b-', label='Ground Truth', linewidth=2.5, alpha=0.8)
    line_est, = ax1.plot([], [], [], 'r-', label='KF Estimate', linewidth=2.5, alpha=0.9)
    scatter_meas = ax1.scatter([], [], [], c='lime', s=80, marker='o', 
                               edgecolors='darkgreen', linewidths=2, label='USBL Fix', alpha=0.9)
    
    ax1.set_xlabel('X [m]', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
    ax1.set_zlabel('Z [m]', fontsize=11, fontweight='bold')
    ax1.set_title('3D Trajectory - Circular Path', fontsize=13, fontweight='bold', pad=15)
    ax1.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax1.grid(True, alpha=0.3)
    ax1.set_facecolor('#f0f0f0')
    
    # Initial limits (will auto-adjust)
    ax1.set_xlim(-12, 12)
    ax1.set_ylim(-12, 12)
    ax1.set_zlim(-2, 2)
    
    # XY trajectory (top right)
    ax2 = fig.add_subplot(gs[0, 1])
    line_true_xy, = ax2.plot([], [], 'b-', label='True', linewidth=2, alpha=0.7)
    line_est_xy, = ax2.plot([], [], 'r-', label='KF Est', linewidth=2, alpha=0.8)
    scatter_meas_xy = ax2.scatter([], [], c='lime', s=50, marker='o', 
                                  edgecolors='darkgreen', linewidths=1.5, label='USBL', alpha=0.9)
    ax2.set_xlabel('X [m]', fontsize=10, fontweight='bold')
    ax2.set_ylabel('Y [m]', fontsize=10, fontweight='bold')
    ax2.set_title('XY Top View', fontsize=11, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.set_facecolor('#f8f8f8')
    ax2.axis('equal')
    
    # Position error over time (middle right)
    ax3 = fig.add_subplot(gs[1, 1])
    line_err, = ax3.plot([], [], 'r-', linewidth=2.5, alpha=0.8)
    ax3.fill_between([], [], 0, alpha=0.2, color='red')
    ax3.set_xlabel('Time [s]', fontsize=10, fontweight='bold')
    ax3.set_ylabel('Error [m]', fontsize=10, fontweight='bold')
    ax3.set_title('Position Error Norm', fontsize=11, fontweight='bold')
    ax3.grid(True, alpha=0.3, linestyle='--')
    ax3.set_facecolor('#f8f8f8')
    ax3.set_ylim(0, 2)  # Initial limit
    
    # Position components over time (bottom right)
    ax4 = fig.add_subplot(gs[2, 1])
    line_x, = ax4.plot([], [], 'r-', label='X', linewidth=2, alpha=0.8)
    line_y, = ax4.plot([], [], 'g-', label='Y', linewidth=2, alpha=0.8)
    line_z, = ax4.plot([], [], 'b-', label='Z', linewidth=2, alpha=0.8)
    ax4.set_xlabel('Time [s]', fontsize=10, fontweight='bold')
    ax4.set_ylabel('Position [m]', fontsize=10, fontweight='bold')
    ax4.set_title('Position Components', fontsize=11, fontweight='bold')
    ax4.legend(loc='upper right', fontsize=8, framealpha=0.9)
    ax4.grid(True, alpha=0.3, linestyle='--')
    ax4.set_facecolor('#f8f8f8')
    
    # Store for fill_between update
    fill_err = None
    
    while running:
        with plot_lock:
            if len(plot_data['time']) > 0:
                ts = list(plot_data['time'])
                true_pos = list(plot_data['true_pos'])
                est_pos = list(plot_data['est_pos'])
                meas_pos = [p for t, p in plot_data['meas_pos']]
                errors = list(plot_data['error'])
                
                if len(true_pos) > 0:
                    true_arr = np.array(true_pos)
                    est_arr = np.array(est_pos)
                    
                    # Update 3D trajectory
                    line_true.set_data(true_arr[:,0], true_arr[:,1])
                    line_true.set_3d_properties(true_arr[:,2])
                    
                    line_est.set_data(est_arr[:,0], est_arr[:,1])
                    line_est.set_3d_properties(est_arr[:,2])
                    
                    if len(meas_pos) > 0:
                        meas_arr = np.array(meas_pos)
                        scatter_meas._offsets3d = (meas_arr[:,0], meas_arr[:,1], meas_arr[:,2])
                    
                    # Auto-adjust 3D view limits
                    all_x = np.concatenate([true_arr[:,0], est_arr[:,0]])
                    all_y = np.concatenate([true_arr[:,1], est_arr[:,1]])
                    all_z = np.concatenate([true_arr[:,2], est_arr[:,2]])
                    
                    x_margin = (all_x.max() - all_x.min()) * 0.1 + 1
                    y_margin = (all_y.max() - all_y.min()) * 0.1 + 1
                    z_margin = max((all_z.max() - all_z.min()) * 0.2, 1)
                    
                    ax1.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
                    ax1.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
                    ax1.set_zlim(all_z.min() - z_margin, all_z.max() + z_margin)
                    
                    # Update XY trajectory
                    line_true_xy.set_data(true_arr[:,0], true_arr[:,1])
                    line_est_xy.set_data(est_arr[:,0], est_arr[:,1])
                    if len(meas_pos) > 0:
                        scatter_meas_xy.set_offsets(meas_arr[:,:2])
                    
                    # Auto-adjust XY limits
                    ax2.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
                    ax2.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
                    
                    # Update position error with fill
                    line_err.set_data(ts, errors)
                    ax3.collections.clear()  # Clear old fill
                    ax3.fill_between(ts, errors, 0, alpha=0.2, color='red')
                    
                    # Auto-adjust error limits
                    if len(errors) > 0:
                        error_max = max(errors)
                        ax3.set_ylim(0, error_max * 1.15)
                    ax3.set_xlim(min(ts), max(ts))
                    
                    # Update position components
                    line_x.set_data(ts, est_arr[:,0])
                    line_y.set_data(ts, est_arr[:,1])
                    line_z.set_data(ts, est_arr[:,2])
                    
                    # Auto-adjust position components limits
                    all_pos = np.concatenate([est_arr[:,0], est_arr[:,1], est_arr[:,2]])
                    pos_margin = (all_pos.max() - all_pos.min()) * 0.1 + 0.5
                    ax4.set_ylim(all_pos.min() - pos_margin, all_pos.max() + pos_margin)
                    ax4.set_xlim(min(ts), max(ts))
        
        plt.pause(0.05)
    
    plt.close('all')

# ==================== MAIN CONTROL LOOP ====================
def main():
    global running
    
    print("=" * 80)
    print("KALMAN FILTER SENSOR FUSION - CIRCULAR TRAJECTORY SIMULATION")
    print(f"Control Loop: {CONTROL_LOOP_MS}ms")
    print(f"Plotting: {'ENABLED' if ENABLE_PLOTTING else 'DISABLED'}")
    print("=" * 80)
    
    # Initialize KF
    Q = np.eye(3) * 0.05
    R1 = np.eye(3) * 0.02
    R2 = np.eye(6) * 0.5
    R2[3:6, 3:6] *= 0.2
    
    C1 = np.array([
        [0,0,0, 0,0,0, 1,0,0],
        [0,0,0, 0,0,0, 0,1,0],
        [0,0,0, 0,0,0, 0,0,1],
    ], dtype=float)
    
    C2 = np.array([
        [0,0,0, 0,0,0, 1,0,0],
        [0,0,0, 0,0,0, 0,1,0],
        [0,0,0, 0,0,0, 0,0,1],
        [1,0,0, 0,0,0, 0,0,0],
        [0,1,0, 0,0,0, 0,0,0],
        [0,0,1, 0,0,0, 0,0,0],
    ], dtype=float)
    
    kf = KF(Q, R1, R2, C1, C2, T=CONTROL_LOOP_MS/1000.0)
    kf.setX(np.zeros((9,1)))
    
    # Start sensor threads
    threads = [
        threading.Thread(target=imu_simulation_thread, daemon=True),
        threading.Thread(target=usbl_simulation_thread, daemon=True),
        threading.Thread(target=depth_simulation_thread, daemon=True),
    ]
    
    if ENABLE_PLOTTING:
        threads.append(threading.Thread(target=plot_thread, daemon=True))
    
    for t in threads:
        t.start()
    
    time.sleep(0.5)
    
    loop_time = CONTROL_LOOP_MS / 1000.0
    iteration = 0
    
    try:
        while True:
            start = time.time()
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            # Update simulation time
            with sim_lock:
                sim_time['t'] = iteration * loop_time
                t_sim = sim_time['t']
            
            # Get sensor data
            with data_lock:
                imu = sensor_data['imu'].copy()
                usbl = sensor_data['usbl'].copy()
                depth = sensor_data['depth'].copy()
            
            # Determine if USBL fix available this iteration
            usbl_fix_available = (iteration % USBL_FIX_EVERY == 0)
            
            # Ground truth for comparison
            theta = OMEGA * t_sim
            p_true = np.array([
                RADIUS * math.cos(theta),
                RADIUS * math.sin(theta),
                0.0
            ])
            
            # Prepare KF update
            roll, pitch, yaw = imu['euler']
            a_body = np.array(imu['acc_body']).reshape((3,1))
            
            if usbl_fix_available and usbl['valid']:
                # Update with acceleration + position
                p_world = np.array(usbl['pos_world']).reshape((3,1))
                yy = np.vstack([a_body, p_world])
                kf.update(yy, roll=roll, pitch=pitch, yaw=yaw, queue=False, dt_override=loop_time)
            else:
                # Update with acceleration only
                kf.update(a_body, roll=roll, pitch=pitch, yaw=yaw, queue=True, dt_override=loop_time)
            
            # Get KF state
            state = kf.getX()
            pos_est = state[0:3,0]
            vel_est = state[3:6,0]
            acc_est = state[6:9,0]
            
            with kf_lock:
                kf_state['pos'] = pos_est.tolist()
                kf_state['vel'] = vel_est.tolist()
                kf_state['acc'] = acc_est.tolist()
            
            # Calculate error
            error_norm = np.linalg.norm(pos_est - p_true)
            
            # Update plot data
            if ENABLE_PLOTTING:
                with plot_lock:
                    plot_data['time'].append(t_sim)
                    plot_data['true_pos'].append(p_true.copy())
                    plot_data['est_pos'].append(pos_est.copy())
                    if usbl_fix_available and usbl['valid']:
                        plot_data['meas_pos'].append((t_sim, np.array(usbl['pos_world'])))
                    # Remove old USBL fixes outside time window
                    if len(plot_data['time']) > 0:
                        t_min = plot_data['time'][0]
                        plot_data['meas_pos'] = [(t, p) for t, p in plot_data['meas_pos'] if t >= t_min]
                    plot_data['error'].append(error_norm)
            
            # Terminal output
            print(f"\n{'='*80}")
            print(f"[{timestamp}] Iteration: {iteration:4d}  Time: {t_sim:6.2f}s")
            print(f"{'='*80}")
            
            # True position
            print(f"TRUE   Pos: [{p_true[0]:7.3f}, {p_true[1]:7.3f}, {p_true[2]:7.3f}]")
            
            # IMU data
            status = "OK" if imu['valid'] else "FAIL"
            print(f"IMU    [{status:4s}] Euler: [{roll:7.3f}, {pitch:7.3f}, {yaw:7.3f}]")
            print(f"              Acc_b: [{a_body[0,0]:7.3f}, {a_body[1,0]:7.3f}, {a_body[2,0]:7.3f}]")
            
            # USBL data
            status = "FIX" if usbl_fix_available else "----"
            if usbl['valid']:
                print(f"USBL   [{status:4s}] Pos: [{usbl['pos_world'][0]:7.3f}, {usbl['pos_world'][1]:7.3f}, {usbl['pos_world'][2]:7.3f}]")
            else:
                print(f"USBL   [----] No data")
            
            # Depth (not used)
            status = "OK" if depth['valid'] else "FAIL"
            print(f"DEPTH  [{status:4s}] Z: {depth['depth']:7.3f} m (not used in KF)")
            
            # KF estimate
            print(f"\nKF EST Pos: [{pos_est[0]:7.3f}, {pos_est[1]:7.3f}, {pos_est[2]:7.3f}]")
            print(f"       Vel: [{vel_est[0]:7.3f}, {vel_est[1]:7.3f}, {vel_est[2]:7.3f}]")
            print(f"       Acc: [{acc_est[0]:7.3f}, {acc_est[1]:7.3f}, {acc_est[2]:7.3f}]")
            print(f"\nERROR: {error_norm:7.4f} m")
            
            iteration += 1
            
            # Timing
            elapsed = time.time() - start
            sleep_time = max(0, loop_time - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        running = False
        time.sleep(0.5)

if __name__ == "__main__":
    main()