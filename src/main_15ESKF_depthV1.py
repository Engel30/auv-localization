#!/usr/bin/env python3
import time
import threading
import numpy as np
import math
import os
import csv
from datetime import datetime
from collections import deque

from lib.KF.RMatrix import Rxyz

# ESKF15 import
import sys
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from lib.KF.ESKF_depth import ESKF15, yaw_from_q, rotation_to_quaternion


try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("[WARNING] matplotlib not available, plotting disabled")

# ==================== CONFIG ====================
CONTROL_LOOP_MS = 50  # 100 Hz per ESKF
ENABLE_PLOTTING = True and MATPLOTLIB_AVAILABLE
ENABLE_LOGGING = False
USE_REAL_SENSORS = False
SHOW_FULL_GROUND_TRUTH = False

RADIUS = 10.0
OMEGA = 0.12
TILT_ANGLE = np.deg2rad(20.0)

# Noise parameters
SIGMA_GYRO = np.deg2rad(0.05)    # rad/sqrt(s)
SIGMA_ACC = 0.05                  # m/s^2/sqrt(s)
SIGMA_BG = np.deg2rad(0.002)     # gyro bias RW
SIGMA_BA = 0.005                  # accel bias RW
SIGMA_POS = 0.10                  # USBL noise (m)
SIGMA_YAW = np.deg2rad(2.0)      # yaw noise

USBL_FIX_EVERY = 50  # 2 Hz @ 100 Hz
YAW_UPDATE_EVERY = 10  # 10 Hz
PLOT_HISTORY = 150

LOG_DIR = "log"

# True sensor biases
BG_TRUE = np.array([[np.deg2rad(0.2)], [np.deg2rad(-0.15)], [np.deg2rad(0.1)]])
BA_TRUE = np.array([[0.03], [-0.02], [0.01]])

# ==================== GLOBALS ====================
sensor_data = {
    'imu': {'gyro': [0.0, 0.0, 0.0], 'acc_body': [0.0, 0.0, 0.0], 
            'euler': [0.0, 0.0, 0.0], 'valid': False},
    'usbl': {'pos_world': [0.0, 0.0, 0.0], 'valid': False},
    'depth': {'depth': 0.0, 'valid': False}
}
data_lock = threading.Lock()
running = True

sim_time = {'t': 0.0}
sim_lock = threading.Lock()

eskf_state = {
    'pos': [0.0, 0.0, 0.0],
    'vel': [0.0, 0.0, 0.0],
    'q': [1.0, 0.0, 0.0, 0.0],
    'bg': [0.0, 0.0, 0.0],
    'ba': [0.0, 0.0, 0.0]
}
eskf_lock = threading.Lock()

plot_data = {
    'time': deque(maxlen=PLOT_HISTORY),
    'true_pos': deque(maxlen=PLOT_HISTORY),
    'est_pos': deque(maxlen=PLOT_HISTORY),
    'meas_pos': [],
    'error': deque(maxlen=PLOT_HISTORY),
    'true_depth': deque(maxlen=PLOT_HISTORY),
    'est_depth': deque(maxlen=PLOT_HISTORY),
    'meas_depth': deque(maxlen=PLOT_HISTORY)
}
plot_lock = threading.Lock()

log_file = None
log_writer = None
log_lock = threading.Lock()

# ==================== HELPER FUNCTIONS ====================
def compute_tilted_circle_position(theta, radius, tilt_angle, z_center=0.0):
    x_flat = radius * math.cos(theta)
    y_flat = radius * math.sin(theta)
    z_flat = z_center
    
    c = math.cos(tilt_angle); s = math.sin(tilt_angle)
    x = x_flat * c - z_flat * s
    y = y_flat
    z = x_flat * s + z_flat * c
    
    return np.array([[x], [y], [z]])

def compute_tilted_circle_velocity(theta, radius, omega, tilt_angle):
    vx_flat = -radius * omega * math.sin(theta)
    vy_flat = radius * omega * math.cos(theta)
    vz_flat = 0.0
    
    c = math.cos(tilt_angle); s = math.sin(tilt_angle)
    vx = vx_flat * c - vz_flat * s
    vy = vy_flat
    vz = vx_flat * s + vz_flat * c
    
    return np.array([[vx], [vy], [vz]])

def compute_tilted_circle_acceleration(theta, radius, omega, tilt_angle):
    ax_flat = -radius * (omega**2) * math.cos(theta)
    ay_flat = -radius * (omega**2) * math.sin(theta)
    az_flat = 0.0
    
    c = math.cos(tilt_angle); s = math.sin(tilt_angle)
    ax = ax_flat * c - az_flat * s
    ay = ay_flat
    az = ax_flat * s + az_flat * c
    
    return np.array([[ax], [ay], [az]])

def compute_body_frame(theta, tilt_angle):
    """Frenet triad: tangent, normal, binormal"""
    # Tangent = velocity direction
    v = compute_tilted_circle_velocity(theta, RADIUS, OMEGA, tilt_angle)
    t_hat = (v / np.linalg.norm(v)).reshape(3)
    
    # Normal = -position (towards center)
    p = compute_tilted_circle_position(theta, RADIUS, tilt_angle)
    n_hat = (-p / np.linalg.norm(p)).reshape(3)
    
    # Binormal = t x n
    b_hat = np.cross(t_hat, n_hat)
    b_hat /= np.linalg.norm(b_hat)
    
    # Reorthogonalize
    n_hat = np.cross(b_hat, t_hat)
    n_hat /= np.linalg.norm(n_hat)
    
    Rwb = np.column_stack([t_hat, n_hat, b_hat])
    return Rwb

def generate_full_circle(radius, tilt_angle, z_center=0.0, num_points=200):
    thetas = np.linspace(0, 2*np.pi, num_points)
    positions = []
    for theta in thetas:
        pos = compute_tilted_circle_position(theta, radius, tilt_angle, z_center)
        positions.append(pos.flatten())
    return np.array(positions)

def init_logging():
    global log_file, log_writer
    if not ENABLE_LOGGING:
        return
    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(LOG_DIR, f"eskf15_simulation_{timestamp}.csv")
    log_file = open(log_filename, 'w', newline='')
    log_writer = csv.writer(log_file)
    header = ['timestamp', 'sim_time', 'iteration',
              'true_pos_x', 'true_pos_y', 'true_pos_z',
              'imu_valid', 'gyro_x', 'gyro_y', 'gyro_z',
              'acc_x', 'acc_y', 'acc_z', 'yaw',
              'usbl_valid', 'usbl_x', 'usbl_y', 'usbl_z', 'usbl_fix_used',
              'depth_valid', 'depth',
              'est_pos_x', 'est_pos_y', 'est_pos_z',
              'est_vel_x', 'est_vel_y', 'est_vel_z',
              'est_yaw', 'error_norm']
    log_writer.writerow(header)
    print(f"Logging enabled: {log_filename}")

def log_data(timestamp_str, sim_time_val, iteration, p_true, imu, usbl, depth,
             usbl_fix_used, state_est, error_norm):
    if not ENABLE_LOGGING or log_writer is None:
        return
    with log_lock:
        row = [timestamp_str, f"{sim_time_val:.3f}", iteration,
               f"{p_true[0]:.6f}", f"{p_true[1]:.6f}", f"{p_true[2]:.6f}",
               int(imu['valid']), 
               f"{imu['gyro'][0]:.6f}", f"{imu['gyro'][1]:.6f}", f"{imu['gyro'][2]:.6f}",
               f"{imu['acc_body'][0]:.6f}", f"{imu['acc_body'][1]:.6f}", f"{imu['acc_body'][2]:.6f}",
               f"{imu['euler'][2]:.6f}",
               int(usbl['valid']),
               f"{usbl['pos_world'][0]:.6f}", f"{usbl['pos_world'][1]:.6f}", f"{usbl['pos_world'][2]:.6f}",
               int(usbl_fix_used),
               int(depth['valid']), f"{depth['depth']:.6f}",
               f"{state_est['pos'][0]:.6f}", f"{state_est['pos'][1]:.6f}", f"{state_est['pos'][2]:.6f}",
               f"{state_est['vel'][0]:.6f}", f"{state_est['vel'][1]:.6f}", f"{state_est['vel'][2]:.6f}",
               f"{yaw_from_q(state_est['q']):.6f}", f"{error_norm:.6f}"]
        log_writer.writerow(row)

def close_logging():
    global log_file
    if ENABLE_LOGGING and log_file is not None:
        log_file.close()
        print("\nLogging file closed")

# ==================== SIMULATION THREADS ====================
def imu_simulation_thread():
    global running
    dt = CONTROL_LOOP_MS / 1000.0
    
    while running:
        with sim_lock:
            t = sim_time['t']
        
        theta = OMEGA * t
        
        # True body frame
        Rwb = compute_body_frame(theta, TILT_ANGLE)
        
        # Angular velocity (world): rotation about binormal
        b_hat = Rwb[:, 2].reshape(3, 1)
        omega_w = OMEGA * b_hat
        omega_b_true = Rwb.T @ omega_w
        
        # Specific force (body)
        a_w = compute_tilted_circle_acceleration(theta, RADIUS, OMEGA, TILT_ANGLE)
        g = np.array([[0], [0], [-9.81]])
        f_w = a_w - g
        f_b_true = Rwb.T @ f_w
        
        # Add biases and noise
        gyro_meas = omega_b_true + BG_TRUE + np.random.normal(0, SIGMA_GYRO/np.sqrt(dt), size=(3,1))
        acc_meas = f_b_true + BA_TRUE + np.random.normal(0, SIGMA_ACC/np.sqrt(dt), size=(3,1))
        
        # Yaw for reference (not always used)
        q_true = rotation_to_quaternion(Rwb)
        yaw_true = yaw_from_q(q_true)
        yaw_meas = yaw_true + np.random.normal(0, SIGMA_YAW)
        
        with data_lock:
            sensor_data['imu']['gyro'] = [gyro_meas[0,0], gyro_meas[1,0], gyro_meas[2,0]]
            sensor_data['imu']['acc_body'] = [acc_meas[0,0], acc_meas[1,0], acc_meas[2,0]]
            sensor_data['imu']['euler'] = [0.0, 0.0, yaw_meas]
            sensor_data['imu']['valid'] = True
        
        time.sleep(dt * 0.5)

def usbl_simulation_thread():
    global running
    
    while running:
        with sim_lock:
            t = sim_time['t']
        
        theta = OMEGA * t
        p_true = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
        
        p_meas = p_true + np.random.normal(0, SIGMA_POS, size=(3,1))
        
        with data_lock:
            sensor_data['usbl']['pos_world'] = [p_meas[0,0], p_meas[1,0], p_meas[2,0]]
            sensor_data['usbl']['valid'] = True
        
        time.sleep(0.1)

def depth_simulation_thread():
    global running
    
    while running:
        with sim_lock:
            t = sim_time['t']
        
        theta = OMEGA * t
        p_true = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
        depth_true = -p_true[2,0]
        
        depth_meas = depth_true + np.random.normal(0, 0.05)
        
        with data_lock:
            sensor_data['depth']['depth'] = depth_meas
            sensor_data['depth']['valid'] = True
        
        time.sleep(0.05)

def plot_thread():
    if not MATPLOTLIB_AVAILABLE:
        return
    
    full_circle = generate_full_circle(RADIUS, TILT_ANGLE) if SHOW_FULL_GROUND_TRUTH else None
    
    fig = plt.figure(figsize=(15, 10))
    
    ax1 = fig.add_subplot(231, projection='3d')
    ax1.set_title('3D Trajectory')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    if full_circle is not None:
        ax1.plot(full_circle[:,0], full_circle[:,1], full_circle[:,2], 
                'k--', alpha=0.3, label='Ground truth path')
    line_true_3d, = ax1.plot([], [], [], 'g-', label='True', linewidth=2)
    line_est_3d, = ax1.plot([], [], [], 'b-', label='ESKF', linewidth=2)
    scatter_meas_3d = ax1.scatter([], [], [], c='red', s=20, label='USBL', alpha=0.6)
    ax1.legend()
    
    ax2 = fig.add_subplot(232)
    ax2.set_title('XY Trajectory')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.grid(True)
    line_true_xy, = ax2.plot([], [], 'g-', label='True', linewidth=2)
    line_est_xy, = ax2.plot([], [], 'b-', label='ESKF', linewidth=2)
    scatter_meas_xy = ax2.scatter([], [], c='red', s=20, label='USBL', alpha=0.6)
    ax2.legend()
    
    ax3 = fig.add_subplot(233)
    ax3.set_title('Position Error')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Error [m]')
    ax3.grid(True)
    line_err, = ax3.plot([], [], 'r-', linewidth=2)
    
    ax4 = fig.add_subplot(234)
    ax4.set_title('Position Components')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Position [m]')
    ax4.grid(True)
    line_x, = ax4.plot([], [], 'r-', label='X', linewidth=1.5)
    line_y, = ax4.plot([], [], 'g-', label='Y', linewidth=1.5)
    line_z, = ax4.plot([], [], 'b-', label='Z', linewidth=1.5)
    ax4.legend()
    
    ax5 = fig.add_subplot(235)
    ax5.set_title('Depth')
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Depth [m]')
    ax5.grid(True)
    ax5.invert_yaxis()
    line_true_depth, = ax5.plot([], [], 'g-', label='True', linewidth=2)
    line_est_depth, = ax5.plot([], [], 'b-', label='ESKF', linewidth=2)
    line_meas_depth, = ax5.plot([], [], 'r.', label='Sensor', markersize=3)
    ax5.legend()
    
    plt.tight_layout()
    plt.ion()
    plt.show()
    
    while running:
        with plot_lock:
            if len(plot_data['time']) > 0:
                ts = list(plot_data['time'])
                true_pos = list(plot_data['true_pos'])
                est_pos = list(plot_data['est_pos'])
                meas_pos = plot_data['meas_pos'].copy()
                errors = list(plot_data['error'])
                true_depth = list(plot_data['true_depth'])
                est_depth = list(plot_data['est_depth'])
                meas_depth = list(plot_data['meas_depth'])
                
                if len(ts) > 1:
                    true_arr = np.array(true_pos)
                    est_arr = np.array(est_pos)
                    meas_arr = np.array([p for _, p in meas_pos]) if len(meas_pos) > 0 else np.empty((0,3))
                    
                    # 3D
                    line_true_3d.set_data_3d(true_arr[:,0], true_arr[:,1], true_arr[:,2])
                    line_est_3d.set_data_3d(est_arr[:,0], est_arr[:,1], est_arr[:,2])
                    if len(meas_pos) > 0:
                        scatter_meas_3d._offsets3d = (meas_arr[:,0], meas_arr[:,1], meas_arr[:,2])
                    
                    all_pos = np.vstack([true_arr, est_arr])
                    margin = (all_pos.max(axis=0) - all_pos.min(axis=0)) * 0.1 + 1
                    ax1.set_xlim(all_pos[:,0].min() - margin[0], all_pos[:,0].max() + margin[0])
                    ax1.set_ylim(all_pos[:,1].min() - margin[1], all_pos[:,1].max() + margin[1])
                    ax1.set_zlim(all_pos[:,2].min() - margin[2], all_pos[:,2].max() + margin[2])
                    
                    # XY
                    line_true_xy.set_data(true_arr[:,0], true_arr[:,1])
                    line_est_xy.set_data(est_arr[:,0], est_arr[:,1])
                    if len(meas_pos) > 0:
                        scatter_meas_xy.set_offsets(meas_arr[:,:2])
                    
                    x_margin = (true_arr[:,0].max() - true_arr[:,0].min()) * 0.1 + 1
                    y_margin = (true_arr[:,1].max() - true_arr[:,1].min()) * 0.1 + 1
                    ax2.set_xlim(true_arr[:,0].min() - x_margin, true_arr[:,0].max() + x_margin)
                    ax2.set_ylim(true_arr[:,1].min() - y_margin, true_arr[:,1].max() + y_margin)
                    
                    # Error
                    line_err.set_data(ts, errors)
                    ax3.collections.clear()
                    ax3.fill_between(ts, errors, 0, alpha=0.2, color='red')
                    if len(errors) > 0:
                        ax3.set_ylim(0, max(errors) * 1.15)
                    ax3.set_xlim(min(ts), max(ts))
                    
                    # Components
                    line_x.set_data(ts, est_arr[:,0])
                    line_y.set_data(ts, est_arr[:,1])
                    line_z.set_data(ts, est_arr[:,2])
                    all_comp = np.concatenate([est_arr[:,0], est_arr[:,1], est_arr[:,2]])
                    comp_margin = (all_comp.max() - all_comp.min()) * 0.1 + 0.5
                    ax4.set_ylim(all_comp.min() - comp_margin, all_comp.max() + comp_margin)
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
    
    plt.close('all')

# ==================== MAIN CONTROL LOOP ====================
def main():
    global running
    
    print("=" * 80)
    print("15-STATE ERROR-STATE KALMAN FILTER - ESKF15")
    print(f"Control Loop: {CONTROL_LOOP_MS}ms")
    print(f"Trajectory: Tilted Circle ({np.rad2deg(TILT_ANGLE):.0f}°)")
    print(f"Plotting: {'ENABLED' if ENABLE_PLOTTING else 'DISABLED'}")
    print(f"Logging: {'ENABLED' if ENABLE_LOGGING else 'DISABLED'}")
    print("=" * 80)
    
    init_logging()
    
    dt = CONTROL_LOOP_MS / 1000.0
    
    # ESKF noise matrices
    Qc = np.diag([SIGMA_GYRO**2]*3 + [SIGMA_ACC**2]*3 + 
                 [SIGMA_BG**2]*3 + [SIGMA_BA**2]*3)
    R_pos = np.diag([SIGMA_POS**2]*3)
    R_yaw = np.array([[SIGMA_YAW**2]])
    
    eskf = ESKF15(Qc=Qc, R_pos=R_pos, R_yaw=R_yaw, dt=dt)
    
    # Initialize at starting position
    p_init = compute_tilted_circle_position(0, RADIUS, TILT_ANGLE)
    eskf.p = p_init.copy()
    eskf.v[:] = 0.0
    # Will be initialized on first measurement
    
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
    
    iteration = 0
    first_fix = True
    
    try:
        while True:
            start = time.time()
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            with sim_lock:
                sim_time['t'] = iteration * dt
                t_sim = sim_time['t']
            
            with data_lock:
                imu = sensor_data['imu'].copy()
                usbl = sensor_data['usbl'].copy()
                depth = sensor_data['depth'].copy()
            
            theta = OMEGA * t_sim
            p_true = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE).flatten()
            
            # Get IMU measurements
            gyro_meas = np.array(imu['gyro']).reshape((3,1))
            acc_meas = np.array(imu['acc_body']).reshape((3,1))
            yaw_meas = imu['euler'][2]
            
            # Propagate
            eskf.propagate(gyro_meas, acc_meas)
            
            # USBL update
            usbl_fix_used = False
            if (iteration % USBL_FIX_EVERY == 0) and usbl['valid']:
                z_pos = np.array(usbl['pos_world']).reshape((3,1))
                if first_fix:
                    # Warm-start
                    eskf.p = z_pos.copy()
                    eskf.v[:] = 0.0
                    Rwb = compute_body_frame(theta, TILT_ANGLE)
                    eskf.q = rotation_to_quaternion(Rwb)
                    first_fix = False
                else:
                    eskf.update_pos(z_pos)
                usbl_fix_used = True
            
            # Yaw update
            if (iteration % YAW_UPDATE_EVERY == 0):
                eskf.update_yaw(yaw_meas)
            
            # Get state
            with eskf_lock:
                eskf_state['pos'] = eskf.p.flatten().tolist()
                eskf_state['vel'] = eskf.v.flatten().tolist()
                eskf_state['q'] = eskf.q.tolist()
                eskf_state['bg'] = eskf.bg.flatten().tolist()
                eskf_state['ba'] = eskf.ba.flatten().tolist()
            
            pos_est = eskf.p.flatten()
            error_norm = np.linalg.norm(pos_est - p_true)
            
            log_data(timestamp, t_sim, iteration, p_true, imu, usbl, depth,
                    usbl_fix_used, eskf_state, error_norm)
            
            if ENABLE_PLOTTING:
                with plot_lock:
                    plot_data['time'].append(t_sim)
                    plot_data['true_pos'].append(p_true.copy())
                    plot_data['est_pos'].append(pos_est.copy())
                    plot_data['true_depth'].append(-p_true[2])
                    plot_data['est_depth'].append(-pos_est[2])
                    plot_data['meas_depth'].append(depth['depth'])
                    
                    if usbl_fix_used:
                        plot_data['meas_pos'].append((t_sim, np.array(usbl['pos_world'])))
                    if len(plot_data['time']) > 0:
                        t_min = plot_data['time'][0]
                        plot_data['meas_pos'] = [(t, p) for t, p in plot_data['meas_pos'] if t >= t_min]
                    plot_data['error'].append(error_norm)
            
            if iteration > 0:
                print('\033[18A\033[J', end='')
            
            print(f"\n{'='*80}")
            print(f"[{timestamp}] Iteration: {iteration:4d}  Time: {t_sim:6.2f}s")
            print(f"{'='*80}")
            print(f"TRUE   Pos: [{p_true[0]:7.3f}, {p_true[1]:7.3f}, {p_true[2]:7.3f}]")
            
            print(f"IMU    Gyro: [{gyro_meas[0,0]:7.4f}, {gyro_meas[1,0]:7.4f}, {gyro_meas[2,0]:7.4f}] rad/s")
            print(f"       Acc:  [{acc_meas[0,0]:7.3f}, {acc_meas[1,0]:7.3f}, {acc_meas[2,0]:7.3f}] m/s²")
            print(f"       Yaw:  {yaw_meas:7.4f} rad")
            
            status = "FIX" if usbl_fix_used else "----"
            print(f"USBL   [{status}] Pos: [{usbl['pos_world'][0]:7.3f}, {usbl['pos_world'][1]:7.3f}, {usbl['pos_world'][2]:7.3f}]")
            print(f"DEPTH  [{('OK' if depth['valid'] else 'FAIL'):4s}] Z: {depth['depth']:7.3f} m")
            
            print(f"\nESKF   Pos: [{pos_est[0]:7.3f}, {pos_est[1]:7.3f}, {pos_est[2]:7.3f}]")
            vel_est = eskf.v.flatten()
            print(f"       Vel: [{vel_est[0]:7.3f}, {vel_est[1]:7.3f}, {vel_est[2]:7.3f}]")
            yaw_est = yaw_from_q(eskf.q)
            print(f"       Yaw: {yaw_est:7.4f} rad")
            print(f"\nERROR: {error_norm:7.4f} m", flush=True)
            
            iteration += 1
            
            elapsed = time.time() - start
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        running = False
        close_logging()
        time.sleep(0.5)

if __name__ == "__main__":
    main()