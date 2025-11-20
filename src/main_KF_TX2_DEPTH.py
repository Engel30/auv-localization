#!/usr/bin/env python3
import time
import threading
import numpy as np
import math
import os
import csv
from datetime import datetime
from collections import deque

from lib.KF.KF_profond import KF
from lib.KF.RMatrix import Rxyz

# ---- Sensori reali ----
try:
    from lib.MS5837.ms5837 import MS5837_30BA
    MS5837_AVAILABLE = True
except ImportError:
    MS5837_AVAILABLE = False

try:
    from lib.xsens.xsense import MTI670
    XSENSE_AVAILABLE = True
except ImportError:
    XSENSE_AVAILABLE = False

# ---- Matplotlib (compatibile 2.1.1) ----
try:
    import matplotlib.pyplot as plt
    # necessario per registrare la proiezione 3D sulle versioni vecchie
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("[WARNING] matplotlib not available, plotting disabled")

# ==================== CONFIG ====================
CONTROL_LOOP_MS = 50
ENABLE_PLOTTING = True and MATPLOTLIB_AVAILABLE
ENABLE_LOGGING = False  # Flag per abilitare il logging
USE_REAL_SENSORS = False
SHOW_FULL_GROUND_TRUTH = False

# Porte / bus sensori reali
IMU_PORT = '/dev/ttyUSB0'
DEPTH_I2C_BUS = 0

RADIUS = 10.0
OMEGA = 0.15
TILT_ANGLE = np.deg2rad(2.0)

SIGMA_ACC = 0.03
SIGMA_POS = 0.15
SIGMA_DEPTH = 0.05
SIGMA_YAW = np.deg2rad(1.0)

USBL_FIX_EVERY = 5
PLOT_HISTORY = 150

LOG_DIR = "log"

# ==================== GLOBALS ====================
sensor_data = {
    'imu': {'euler': [0.0, 0.0, 0.0], 'acc_body': [0.0, 0.0, 0.0], 'valid': False},
    'usbl': {'pos_world': [0.0, 0.0, 0.0], 'valid': False},
    'depth': {'depth': 0.0, 'valid': False}
}
data_lock = threading.Lock()
running = True

sim_time = {'t': 0.0}
sim_lock = threading.Lock()

kf_state = {
    'pos': [0.0, 0.0, 0.0],
    'vel': [0.0, 0.0, 0.0],
    'acc': [0.0, 0.0, 0.0]
}
kf_lock = threading.Lock()

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

# Logging globals
log_file = None
log_writer = None
log_lock = threading.Lock()

# ==================== HELPER FUNCTIONS ====================
def compute_tilted_circle_position(theta, radius, tilt_angle, z_center=-5.0):
    """Compute position on tilted circular trajectory"""
    x_flat = radius * math.cos(theta)
    y_flat = radius * math.sin(theta)
    z_flat = z_center
    
    x = x_flat * math.cos(tilt_angle) - (z_flat) * math.sin(tilt_angle)
    y = y_flat
    z = x_flat * math.sin(tilt_angle) + (z_flat) * math.cos(tilt_angle)
    
    return np.array([[x], [y], [z]])

def compute_tilted_circle_acceleration(theta, radius, omega, tilt_angle):
    """Compute centripetal acceleration on tilted circular trajectory"""
    a_x_flat = -radius * (omega ** 2) * math.cos(theta)
    a_y_flat = -radius * (omega ** 2) * math.sin(theta)
    a_z_flat = 0.0
    
    a_x = a_x_flat * math.cos(tilt_angle) - a_z_flat * math.sin(tilt_angle)
    a_y = a_y_flat
    a_z = a_x_flat * math.sin(tilt_angle) + a_z_flat * math.cos(tilt_angle)
    
    return np.array([[a_x], [a_y], [a_z]])

def generate_full_circle(radius, tilt_angle, z_center=-5.0, num_points=200):
    """Generate complete tilted circle for visualization"""
    thetas = np.linspace(0, 2*np.pi, num_points)
    positions = []
    for theta in thetas:
        pos = compute_tilted_circle_position(theta, radius, tilt_angle, z_center)
        positions.append(pos.flatten())
    return np.array(positions)

def init_logging():
    """Initialize logging system"""
    global log_file, log_writer
    
    if not ENABLE_LOGGING:
        return
    
    # Create log directory if it doesn't exist
    os.makedirs(LOG_DIR, exist_ok=True)
    
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = os.path.join(LOG_DIR, f"kf_simulation_{timestamp}.csv")
    
    # Open file and create CSV writer
    log_file = open(log_filename, 'w', newline='')
    log_writer = csv.writer(log_file)
    
    # Write header
    header = [
        'timestamp',
        'sim_time',
        'iteration',
        'true_pos_x', 'true_pos_y', 'true_pos_z',
        'imu_valid', 'imu_roll', 'imu_pitch', 'imu_yaw',
        'imu_acc_x', 'imu_acc_y', 'imu_acc_z',
        'usbl_valid', 'usbl_pos_x', 'usbl_pos_y', 'usbl_pos_z',
        'usbl_fix_used',
        'depth_valid', 'depth_measurement',
        'kf_pos_x', 'kf_pos_y', 'kf_pos_z',
        'kf_vel_x', 'kf_vel_y', 'kf_vel_z',
        'kf_acc_x', 'kf_acc_y', 'kf_acc_z',
        'error_norm'
    ]
    log_writer.writerow(header)
    
    print(f"Logging enabled: {log_filename}")

def log_data(timestamp_str, sim_time_val, iteration, p_true, imu, usbl, depth, 
             usbl_fix_used, pos_est, vel_est, acc_est, error_norm):
    """Log data to CSV file"""
    if not ENABLE_LOGGING or log_writer is None:
        return
    
    with log_lock:
        row = [
            timestamp_str,
            f"{sim_time_val:.3f}",
            iteration,
            f"{p_true[0]:.6f}", f"{p_true[1]:.6f}", f"{p_true[2]:.6f}",
            int(imu['valid']),
            f"{imu['euler'][0]:.6f}", f"{imu['euler'][1]:.6f}", f"{imu['euler'][2]:.6f}",
            f"{imu['acc_body'][0]:.6f}", f"{imu['acc_body'][1]:.6f}", f"{imu['acc_body'][2]:.6f}",
            int(usbl['valid']),
            f"{usbl['pos_world'][0]:.6f}", f"{usbl['pos_world'][1]:.6f}", f"{usbl['pos_world'][2]:.6f}",
            int(usbl_fix_used),
            int(depth['valid']), f"{depth['depth']:.6f}",
            f"{pos_est[0]:.6f}", f"{pos_est[1]:.6f}", f"{pos_est[2]:.6f}",
            f"{vel_est[0]:.6f}", f"{vel_est[1]:.6f}", f"{vel_est[2]:.6f}",
            f"{acc_est[0]:.6f}", f"{acc_est[1]:.6f}", f"{acc_est[2]:.6f}",
            f"{error_norm:.6f}"
        ]
        log_writer.writerow(row)

def close_logging():
    """Close logging file"""
    global log_file
    
    if ENABLE_LOGGING and log_file is not None:
        log_file.close()
        print("\nLogging file closed")

# ==================== SIMULATION / SENSOR THREADS ====================
def imu_simulation_thread():
    """IMU: usa XSens reale se disponibile (USE_REAL_SENSORS=True), altrimenti simulazione tilted circle."""
    global running

    use_simulation = not USE_REAL_SENSORS
    sensor = None

    if USE_REAL_SENSORS and XSENSE_AVAILABLE:
        try:
            sensor = MTI670(IMU_PORT)
            print("[IMU] Sensore XSens inizializzato")
            use_simulation = False
        except Exception as e:
            print(f"[IMU] Errore inizializzazione IMU: {e}, uso simulazione")
            use_simulation = True
    elif USE_REAL_SENSORS:
        print("[IMU] Libreria XSens non disponibile, uso simulazione IMU")
        use_simulation = True
    else:
        print("[IMU] USE_REAL_SENSORS=False, uso simulazione IMU")
        use_simulation = True

    while running:
        if not use_simulation and sensor is not None:
            # Lettura reale IMU
            try:
                sensor.read_data()
                eul_deg = sensor.getEul()   # [roll, pitch, yaw] in gradi
                roll = math.radians(eul_deg[0])
                pitch = math.radians(eul_deg[1])
                yaw = math.radians(eul_deg[2])

                try:
                    acc_body = sensor.getFacc()  # gravity compensated
                except AttributeError:
                    acc_body = sensor.getAcc()

                with data_lock:
                    sensor_data['imu']['euler'] = [roll, pitch, yaw]
                    sensor_data['imu']['acc_body'] = [acc_body[0], acc_body[1], acc_body[2]]
                    sensor_data['imu']['valid'] = True
            except Exception as e:
                print(f"[IMU] Eccezione lettura IMU, passo a simulazione: {e}")
                use_simulation = True
                sensor = None
                continue

            time.sleep(0.01)
        else:
            # Simulazione IMU (tilted circle)
            with sim_lock:
                t = sim_time['t']
            theta = OMEGA * t

            # Accelerazione in world
            a_world = compute_tilted_circle_acceleration(theta, RADIUS, OMEGA, TILT_ANGLE)

            roll = 0.0
            pitch = 0.0
            yaw_true = theta + math.pi / 2.0

            Rwb = Rxyz(roll, pitch, yaw_true)
            a_body_true = Rwb.T @ a_world

            a_body_meas = a_body_true + np.random.normal(0.0, SIGMA_ACC, size=(3, 1))
            yaw_meas = yaw_true + np.random.normal(0.0, SIGMA_YAW)

            with data_lock:
                sensor_data['imu']['euler'] = [roll, pitch, yaw_meas]
                sensor_data['imu']['acc_body'] = [a_body_meas[0, 0], a_body_meas[1, 0], a_body_meas[2, 0]]
                sensor_data['imu']['valid'] = True

            time.sleep(0.005)

def usbl_simulation_thread():
    """USBL solo simulato (anche se USE_REAL_SENSORS=True)."""
    global running
    
    if USE_REAL_SENSORS:
        print("[USBL] Nessun driver reale, uso simulazione USBL")
    else:
        print("[USBL] USBL in modalità simulata")
    
    while running:
        with sim_lock:
            t = sim_time['t']
        
        theta = OMEGA * t
        
        p_true = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
        
        p_meas = p_true + np.random.normal(0.0, SIGMA_POS, size=(3,1))
        with data_lock:
            sensor_data['usbl']['pos_world'] = [p_meas[0,0], p_meas[1,0], p_meas[2,0]]
            sensor_data['usbl']['valid'] = True
        
        time.sleep(0.01)

def depth_simulation_thread():
    """Profondimetro: usa MS5837 reale se disponibile (USE_REAL_SENSORS=True), altrimenti simulazione."""
    global running

    sensor = None
    use_simulation = not USE_REAL_SENSORS

    if USE_REAL_SENSORS and MS5837_AVAILABLE:
        try:
            sensor = MS5837_30BA(bus=DEPTH_I2C_BUS)
            if sensor.init():
                print("[DEPTH] Sensore MS5837 inizializzato")
                use_simulation = False
            else:
                print("[DEPTH] Init MS5837 fallita, uso simulazione depth")
                use_simulation = True
        except Exception as e:
            print(f"[DEPTH] Errore inizializzazione MS5837: {e}, uso simulazione")
            use_simulation = True
    elif USE_REAL_SENSORS:
        print("[DEPTH] Libreria MS5837 non disponibile, uso simulazione depth")
        use_simulation = True
    else:
        print("[DEPTH] USE_REAL_SENSORS=False, uso simulazione depth")
        use_simulation = True

    while running:
        try:
            if not use_simulation and sensor is not None:
                # Lettura reale: depth() restituisce profondità positiva in metri
                if sensor.read():
                    depth_val = sensor.depth()
                    with data_lock:
                        sensor_data['depth']['depth'] = depth_val
                        sensor_data['depth']['valid'] = True
                else:
                    with data_lock:
                        sensor_data['depth']['valid'] = False
                time.sleep(0.1)
            else:
                # Simulazione depth sul tilted circle
                with sim_lock:
                    t = sim_time['t']
                theta = OMEGA * t

                p_true = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
                depth_true = -p_true[2, 0]  # profondità positiva verso il basso

                depth_meas = depth_true + np.random.normal(0.0, SIGMA_DEPTH)

                with data_lock:
                    sensor_data['depth']['depth'] = depth_meas
                    sensor_data['depth']['valid'] = True

                time.sleep(0.1)
        except Exception as e:
            print(f"[DEPTH] Eccezione thread profondità: {e}")
            time.sleep(0.1)

# ==================== PLOTTING ====================
def plot_thread():
    if not ENABLE_PLOTTING:
        return
    
    plt.ion()
    fig = plt.figure(figsize=(16, 10))
    
    from matplotlib.gridspec import GridSpec
    # NIENTE "figure=fig" per compatibilità Matplotlib 2.1.1
    gs = GridSpec(4, 2,
                  width_ratios=[2, 1],
                  height_ratios=[1, 1, 1, 1],
                  left=0.05, right=0.98,
                  top=0.96, bottom=0.05,
                  wspace=0.25, hspace=0.30)
    
    # 3D trajectory (left, spans all rows)
    ax1 = fig.add_subplot(gs[:, 0], projection='3d')
    
    if SHOW_FULL_GROUND_TRUTH:
        full_circle = generate_full_circle(RADIUS, TILT_ANGLE)
        line_true_full, = ax1.plot(full_circle[:,0], full_circle[:,1], full_circle[:,2], 
                                    'b--', label='Ground Truth', linewidth=2, alpha=0.6)
        line_true = None
    else:
        line_true, = ax1.plot([], [], [], 'b-', label='Ground Truth', linewidth=2.5, alpha=0.8)
        full_circle = None
    
    line_est, = ax1.plot([], [], [], 'r-', label='KF Estimate', linewidth=2.5, alpha=0.9)
    
    scatter_meas = ax1.scatter([], [], [], c='lime', s=80, marker='o', 
                               edgecolors='darkgreen', linewidths=2, label='USBL Fix', alpha=0.9)
    
    ax1.set_xlabel('X [m]', fontsize=11, fontweight='bold')
    ax1.set_ylabel('Y [m]', fontsize=11, fontweight='bold')
    ax1.set_zlabel('Z [m]', fontsize=11, fontweight='bold')
    # niente pad= per compatibilità 2.1.1
    ax1.set_title(f'3D Trajectory - Tilted Circle ({np.rad2deg(TILT_ANGLE):.0f}°)',
                  fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=10, framealpha=0.9)
    ax1.grid(True, alpha=0.3)
    ax1.set_facecolor('#f0f0f0')
    
    if SHOW_FULL_GROUND_TRUTH:
        margin = 2.0
        ax1.set_xlim(full_circle[:,0].min() - margin, full_circle[:,0].max() + margin)
        ax1.set_ylim(full_circle[:,1].min() - margin, full_circle[:,1].max() + margin)
        ax1.set_zlim(full_circle[:,2].min() - margin, full_circle[:,2].max() + margin)
    else:
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
    
    while running:
        with plot_lock:
            if len(plot_data['time']) > 0:
                ts = list(plot_data['time'])
                true_pos = list(plot_data['true_pos'])
                est_pos = list(plot_data['est_pos'])
                meas_pos = [p for t, p in plot_data['meas_pos']]
                errors = list(plot_data['error'])
                true_depth = list(plot_data['true_depth'])
                est_depth = list(plot_data['est_depth'])
                meas_depth = list(plot_data['meas_depth'])
            else:
                ts = []
        
        if not ts:
            plt.pause(0.05)
            continue
        
        true_arr = np.array(true_pos)
        est_arr = np.array(est_pos)
        
        # 3D trajectory
        if SHOW_FULL_GROUND_TRUTH:
            line_est.set_data(est_arr[:,0], est_arr[:,1])
            line_est.set_3d_properties(est_arr[:,2])
        else:
            line_true.set_data(true_arr[:,0], true_arr[:,1])
            line_true.set_3d_properties(true_arr[:,2])
            
            line_est.set_data(est_arr[:,0], est_arr[:,1])
            line_est.set_3d_properties(est_arr[:,2])
            
            all_x = np.concatenate([true_arr[:,0], est_arr[:,0]])
            all_y = np.concatenate([true_arr[:,1], est_arr[:,1]])
            all_z = np.concatenate([true_arr[:,2], est_arr[:,2]])
            
            x_margin = (all_x.max() - all_x.min()) * 0.1 + 1
            y_margin = (all_y.max() - all_y.min()) * 0.1 + 1
            z_margin = max((all_z.max() - all_z.min()) * 0.2, 1)
            
            ax1.set_xlim(all_x.min() - x_margin, all_x.max() + x_margin)
            ax1.set_ylim(all_y.min() - y_margin, all_y.max() + y_margin)
            ax1.set_zlim(all_z.min() - z_margin, all_z.max() + z_margin)
        
        if len(meas_pos) > 0:
            meas_arr = np.array(meas_pos)
            # Compatibile con Matplotlib 2.x
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
        # niente .clear() per compatibilità
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
        
        # Depth data
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
    print("KALMAN FILTER SENSOR FUSION - KF_profond.py")
    print(f"Control Loop: {CONTROL_LOOP_MS}ms")
    print(f"Trajectory: Tilted Circle ({np.rad2deg(TILT_ANGLE):.0f}°)")
    print(f"Full Ground Truth: {'ENABLED' if SHOW_FULL_GROUND_TRUTH else 'DISABLED'}")
    print(f"Plotting: {'ENABLED' if ENABLE_PLOTTING else 'DISABLED'}")
    print(f"Sensors: {'REAL+SIM' if USE_REAL_SENSORS else 'SIM ONLY'}")
    print(f"Logging: {'ENABLED' if ENABLE_LOGGING else 'DISABLED'}")
    print("=" * 80)
    
    # Initialize logging
    init_logging()
    
    Q = np.eye(3) * 0.05
    R1 = np.eye(3) * (SIGMA_ACC ** 2)
    R2 = np.eye(3) * (SIGMA_POS ** 2)
    R3 = np.array([[SIGMA_DEPTH ** 2]])
    
    C1 = np.array([
        [0,0,0, 0,0,0, 1,0,0],
        [0,0,0, 0,0,0, 0,1,0],
        [0,0,0, 0,0,0, 0,0,1],
    ], dtype=float)
    
    C2 = np.array([
        [1,0,0, 0,0,0, 0,0,0],
        [0,1,0, 0,0,0, 0,0,0],
        [0,0,1, 0,0,0, 0,0,0],
    ], dtype=float)
    
    C3 = np.array([
        [0,0,1, 0,0,0, 0,0,0],
    ], dtype=float)
    
    kf = KF(Q, R1, R2, R3, C1, C2, C3, T=CONTROL_LOOP_MS/1000.0)
    
    p_init = compute_tilted_circle_position(0, RADIUS, TILT_ANGLE)
    kf.setX(np.array([[p_init[0,0], p_init[1,0], p_init[2,0],
                       0.0, 0.0, 0.0,
                       0.0, 0.0, 0.0]]).T)
    
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
            
            with sim_lock:
                sim_time['t'] = iteration * loop_time
                t_sim = sim_time['t']
            
            with data_lock:
                imu = sensor_data['imu'].copy()
                usbl = sensor_data['usbl'].copy()
                depth = sensor_data['depth'].copy()
            
            usbl_fix_available = (iteration % USBL_FIX_EVERY == 0)
            
            theta = OMEGA * t_sim
            p_true_vec = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
            p_true = p_true_vec.flatten()
            
            roll, pitch, yaw = imu['euler']
            a_body = np.array(imu['acc_body']).reshape((3,1))
            
            if usbl_fix_available and usbl['valid']:
                p_world = np.array(usbl['pos_world']).reshape((3,1))
                kf.update(p_world, roll, pitch, yaw, mode='usbl', dt_override=loop_time)
            else:
                kf.update(a_body, roll, pitch, yaw, mode='imu', dt_override=loop_time)
            
            if depth['valid'] and (iteration % 3 == 0):
                # depth['depth'] è positiva verso il basso -> z = -depth
                depth_meas = np.array([[-depth['depth']]])
                kf.update(depth_meas, roll, pitch, yaw, mode='depth', dt_override=0.0)
            
            state = kf.getX()
            pos_est = state[0:3,0]
            vel_est = state[3:6,0]
            acc_est = state[6:9,0]
            
            with kf_lock:
                kf_state['pos'] = pos_est.tolist()
                kf_state['vel'] = vel_est.tolist()
                kf_state['acc'] = acc_est.tolist()
            
            error_norm = np.linalg.norm(pos_est - p_true)
            
            # Log data
            log_data(timestamp, t_sim, iteration, p_true, imu, usbl, depth,
                     usbl_fix_available, pos_est, vel_est, acc_est, error_norm)
            
            if ENABLE_PLOTTING:
                with plot_lock:
                    plot_data['time'].append(t_sim)
                    plot_data['true_pos'].append(p_true.copy())
                    plot_data['est_pos'].append(pos_est.copy())
                    plot_data['true_depth'].append(-p_true[2])
                    plot_data['est_depth'].append(-pos_est[2])
                    plot_data['meas_depth'].append(depth['depth'])
                    
                    if usbl_fix_available and usbl['valid']:
                        plot_data['meas_pos'].append((t_sim, np.array(usbl['pos_world'])))
                    if len(plot_data['time']) > 0:
                        t_min = plot_data['time'][0]
                        plot_data['meas_pos'] = [(t, p) for t, p in plot_data['meas_pos'] if t >= t_min]
                    plot_data['error'].append(error_norm)
            
            if iteration > 0:
                print('\033[16A\033[J', end='')
            
            print(f"\n{'='*80}")
            print(f"[{timestamp}] Iteration: {iteration:4d}  Time: {t_sim:6.2f}s")
            print(f"{'='*80}")
            
            print(f"TRUE   Pos: [{p_true[0]:7.3f}, {p_true[1]:7.3f}, {p_true[2]:7.3f}]")
            
            status = "OK" if imu['valid'] else "FAIL"
            print(f"IMU    [{status:4s}] Euler: [{roll:7.3f}, {pitch:7.3f}, {yaw:7.3f}]")
            print(f"              Acc_b: [{a_body[0,0]:7.3f}, {a_body[1,0]:7.3f}, {a_body[2,0]:7.3f}]")
            
            status = "FIX" if usbl_fix_available else "----"
            if usbl['valid']:
                print(f"USBL   [{status:4s}] Pos: [{usbl['pos_world'][0]:7.3f}, "
                      f"{usbl['pos_world'][1]:7.3f}, {usbl['pos_world'][2]:7.3f}]")
            else:
                print(f"USBL   [----] No data")
            
            status = "OK" if depth['valid'] else "FAIL"
            print(f"DEPTH  [{status:4s}] Z: {depth['depth']:7.3f} m")
            
            print(f"\nKF EST Pos: [{pos_est[0]:7.3f}, {pos_est[1]:7.3f}, {pos_est[2]:7.3f}]")
            print(f"       Vel: [{vel_est[0]:7.3f}, {vel_est[1]:7.3f}, {vel_est[2]:7.3f}]")
            print(f"       Acc: [{acc_est[0]:7.3f}, {acc_est[1]:7.3f}, {acc_est[2]:7.3f}]")
            print(f"\nERROR: {error_norm:7.4f} m", flush=True)
            
            iteration += 1
            
            elapsed = time.time() - start
            sleep_time = max(0, loop_time - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        running = False
        close_logging()
        time.sleep(0.5)

if __name__ == "__main__":
    main()
