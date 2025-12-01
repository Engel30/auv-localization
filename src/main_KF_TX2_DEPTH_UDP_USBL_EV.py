#!/usr/bin/env python3
import time
import threading
import numpy as np
import math
import os
import csv
import socket
import pickle
from datetime import datetime
from collections import deque

from lib.KF.KF_profond import KF
from lib.KF.RMatrix import Rxyz

# ---- USBL Library ----
try:
    from lib.USBL.USBL import USBLTransponder
    USBL_LIB_AVAILABLE = True
except ImportError:
    USBL_LIB_AVAILABLE = False
    print("[WARNING] usbl_lib not available, USBL will run in simulation mode only")

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
ENABLE_PLOTTING = False and MATPLOTLIB_AVAILABLE
ENABLE_LOGGING = False  # Flag per abilitare il logging
ENABLE_UDP_STREAM = True  # Flag per streammare dati via UDP
UDP_HOST = '127.0.0.1'
UDP_PORT = 5555
USE_REAL_SENSORS = True
SHOW_FULL_GROUND_TRUTH = False

# Porte / bus sensori reali
IMU_PORT = '/dev/ttyUSB0'
DEPTH_I2C_BUS = 0

# USBL Configuration
USBL_TRANSPONDER_ID = 3
USBL_TRANSCEIVER_ID = 2
USBL_IP = "192.168.0.232"
USBL_PORT = 9200
USBL_REQUEST_INTERVAL = 2.0  # seconds between position requests

RADIUS = 10.0
OMEGA = 0.15
TILT_ANGLE = np.deg2rad(2.0)

SIGMA_ACC = 0.03
SIGMA_POS = 0.15
SIGMA_DEPTH = 0.05
SIGMA_YAW = np.deg2rad(1.0)

PLOT_HISTORY = 150

LOG_DIR = "log"

# ==================== GLOBALS ====================
sensor_data = {
    'imu': {'euler': [0.0, 0.0, 0.0], 'acc_body': [0.0, 0.0, 0.0], 'valid': False},
    'usbl': {
        'pos_world': [0.0, 0.0, 0.0], 
        'valid': False, 
        'distance': 0.0, 
        'rssi': None, 
        'integrity': None,
        'new_data': False,  # NEW: Flag to indicate new USBL measurement
        'timestamp': 0.0    # NEW: Timestamp of last USBL update
    },
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

# UDP globals
udp_socket = None
udp_packet_count = 0

# USBL globals
usbl_transponder = None
last_usbl_request_time = 0.0  # NEW: Track last USBL request time

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
    log_filename = os.path.join(LOG_DIR, "kf_simulation_{}.csv".format(timestamp))
    
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
    
    print("Logging enabled: {}".format(log_filename))

def log_data(timestamp_str, sim_time_val, iteration, p_true, imu, usbl, depth, 
             usbl_fix_used, pos_est, vel_est, acc_est, error_norm):
    """Log data to CSV file"""
    if not ENABLE_LOGGING or log_writer is None:
        return
    
    with log_lock:
        row = [
            timestamp_str,
            "{:.3f}".format(sim_time_val),
            iteration,
            "{:.6f}".format(p_true[0]), "{:.6f}".format(p_true[1]), "{:.6f}".format(p_true[2]),
            int(imu['valid']),
            "{:.6f}".format(imu['euler'][0]), "{:.6f}".format(imu['euler'][1]), "{:.6f}".format(imu['euler'][2]),
            "{:.6f}".format(imu['acc_body'][0]), "{:.6f}".format(imu['acc_body'][1]), "{:.6f}".format(imu['acc_body'][2]),
            int(usbl['valid']),
            "{:.6f}".format(usbl['pos_world'][0]), "{:.6f}".format(usbl['pos_world'][1]), "{:.6f}".format(usbl['pos_world'][2]),
            int(usbl_fix_used),
            int(depth['valid']), "{:.6f}".format(depth['depth']),
            "{:.6f}".format(pos_est[0]), "{:.6f}".format(pos_est[1]), "{:.6f}".format(pos_est[2]),
            "{:.6f}".format(vel_est[0]), "{:.6f}".format(vel_est[1]), "{:.6f}".format(vel_est[2]),
            "{:.6f}".format(acc_est[0]), "{:.6f}".format(acc_est[1]), "{:.6f}".format(acc_est[2]),
            "{:.6f}".format(error_norm)
        ]
        log_writer.writerow(row)

def close_logging():
    """Close logging file"""
    global log_file
    
    if ENABLE_LOGGING and log_file is not None:
        log_file.close()
        print("\nLogging file closed")

def init_udp():
    """Initialize UDP socket"""
    global udp_socket
    
    if not ENABLE_UDP_STREAM:
        return
    
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("UDP streaming enabled: {}:{}".format(UDP_HOST, UDP_PORT))

def send_udp_data():
    """Send plot data via UDP"""
    global udp_packet_count
    
    if not ENABLE_UDP_STREAM or udp_socket is None:
        return
    
    try:
        with plot_lock:
            data_to_send = {
                'time': list(plot_data['time']),
                'true_pos': [p.tolist() for p in plot_data['true_pos']],
                'est_pos': [p.tolist() for p in plot_data['est_pos']],
                'meas_pos': [(t, p.tolist()) for t, p in plot_data['meas_pos']],
                'error': list(plot_data['error']),
                'true_depth': list(plot_data['true_depth']),
                'est_depth': list(plot_data['est_depth']),
                'meas_depth': list(plot_data['meas_depth'])
            }
        
        pickled_data = pickle.dumps(data_to_send)
        
        # Split into chunks if needed (UDP max ~65KB)
        MAX_CHUNK = 60000
        if len(pickled_data) < MAX_CHUNK:
            udp_socket.sendto(pickled_data, (UDP_HOST, UDP_PORT))
            udp_packet_count += 1
        
    except Exception as e:
        pass  # Silently ignore UDP errors

# ==================== SENSOR THREADS ====================
def imu_simulation_thread():
    """IMU: usa MTI670 reale se disponibile (USE_REAL_SENSORS=True), altrimenti simulazione."""
    global running

    sensor = None
    use_simulation = not USE_REAL_SENSORS

    if USE_REAL_SENSORS and XSENSE_AVAILABLE:
        try:
            sensor = MTI670(port=IMU_PORT)
            if sensor.init():
                print("[IMU] Sensore Xsens MTI670 inizializzato")
                use_simulation = False
            else:
                print("[IMU] Init MTI670 fallita, uso simulazione IMU")
                use_simulation = True
        except Exception as e:
            print("[IMU] Errore inizializzazione MTI670: {}, uso simulazione".format(e))
            use_simulation = True
    elif USE_REAL_SENSORS:
        print("[IMU] Libreria Xsens non disponibile, uso simulazione IMU")
        use_simulation = True
    else:
        print("[IMU] USE_REAL_SENSORS=False, uso simulazione IMU")
        use_simulation = True

    while running:
        if not use_simulation and sensor is not None:
            try:
                packet = sensor.read_data_packet()
                if packet:
                    euler = packet.get('euler', [0.0, 0.0, 0.0])
                    acc = packet.get('acceleration', [0.0, 0.0, 0.0])
                    with data_lock:
                        sensor_data['imu']['euler'] = euler
                        sensor_data['imu']['acc_body'] = acc
                        sensor_data['imu']['valid'] = True
                else:
                    with data_lock:
                        sensor_data['imu']['valid'] = False
            except Exception as e:
                print("[IMU] Errore lettura sensore: {}".format(e))
                with data_lock:
                    sensor_data['imu']['valid'] = False
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

def usbl_thread():
    """USBL: usa transponder reale se disponibile (USE_REAL_SENSORS=True), altrimenti simulazione.
    EVENT-DRIVEN: Updates only when a new USBL request is successful."""
    global running, usbl_transponder, last_usbl_request_time
    
    use_simulation = not USE_REAL_SENSORS
    
    if USE_REAL_SENSORS and USBL_LIB_AVAILABLE:
        try:
            print("[USBL] Initializing real USBL transponder...")
            usbl_transponder = USBLTransponder(
                transponder_id=USBL_TRANSPONDER_ID,
                transceiver_id=USBL_TRANSCEIVER_ID,
                usbl_ip=USBL_IP,
                usbl_port=USBL_PORT,
                request_interval=USBL_REQUEST_INTERVAL
            )
            usbl_transponder.start()
            
            # Wait a bit to see if connection succeeds
            time.sleep(1.0)
            
            if usbl_transponder.is_connected():
                print("[USBL] Real USBL connected")
                use_simulation = False
            else:
                print("[USBL] USBL not connected, falling back to simulation")
                use_simulation = True
        except Exception as e:
            print("[USBL] Error initializing USBL: {}, using simulation".format(e))
            use_simulation = True
    elif USE_REAL_SENSORS:
        print("[USBL] USBL library not available, using simulation")
        use_simulation = True
    else:
        print("[USBL] USE_REAL_SENSORS=False, using simulation")
        use_simulation = True
    
    # Main loop - EVENT DRIVEN
    last_request_time = time.time()
    
    while running:
        current_time = time.time()
        
        if not use_simulation and usbl_transponder is not None:
            # Read from real USBL
            try:
                # Check if it's time for a new request
                if (current_time - last_request_time) >= USBL_REQUEST_INTERVAL:
                    last_request_time = current_time
                    
                    # Request position
                    pos_data = usbl_transponder.get_position()
                    
                    # Check if still connected
                    if not usbl_transponder.is_connected():
                        print("[USBL] Connection lost, switching to simulation")
                        use_simulation = True
                        continue
                    
                    # Update sensor data with new measurement
                    with data_lock:
                        if pos_data['valid']:
                            # New valid USBL data received
                            sensor_data['usbl']['pos_world'] = pos_data['pos_world']
                            sensor_data['usbl']['valid'] = True
                            sensor_data['usbl']['distance'] = pos_data.get('distance', 0.0)
                            sensor_data['usbl']['rssi'] = pos_data.get('rssi')
                            sensor_data['usbl']['integrity'] = pos_data.get('integrity')
                            sensor_data['usbl']['new_data'] = True  # Mark as new data
                            sensor_data['usbl']['timestamp'] = current_time
                        else:
                            # Request failed, keep last valid data but mark as no new data
                            sensor_data['usbl']['new_data'] = False
                            # sensor_data['usbl']['valid'] remains from last successful request
                
                time.sleep(0.05)
            except Exception as e:
                print("[USBL] Error reading USBL: {}".format(e))
                with data_lock:
                    sensor_data['usbl']['new_data'] = False
                time.sleep(0.1)
        else:
            # Simulation mode - REQUEST at fixed interval
            if (current_time - last_request_time) >= USBL_REQUEST_INTERVAL:
                last_request_time = current_time
                
                with sim_lock:
                    t = sim_time['t']
                
                theta = OMEGA * t
                p_true = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
                p_meas = p_true + np.random.normal(0.0, SIGMA_POS, size=(3,1))
                
                with data_lock:
                    sensor_data['usbl']['pos_world'] = [p_meas[0,0], p_meas[1,0], p_meas[2,0]]
                    sensor_data['usbl']['valid'] = True
                    sensor_data['usbl']['distance'] = 0.0
                    sensor_data['usbl']['rssi'] = None
                    sensor_data['usbl']['integrity'] = None
                    sensor_data['usbl']['new_data'] = True  # Mark as new data
                    sensor_data['usbl']['timestamp'] = current_time
            
            time.sleep(0.01)

def depth_simulation_thread():
    """Profondimetro: usa MS5837 reale se disponibile (USE_REAL_SENSORS=True), altrimenti simulazione."""
    global running

    sensor = None
    use_simulation = not USE_REAL_SENSORS

    if USE_REAL_SENSORS and MS5837_AVAILABLE:
        try:
            sensor = MS5837_30BA()
            if sensor.init(bus=DEPTH_I2C_BUS):
                print("[DEPTH] Sensore MS5837 inizializzato")
                use_simulation = False
            else:
                print("[DEPTH] Init MS5837 fallita, uso simulazione depth")
                use_simulation = True
        except Exception as e:
            print("[DEPTH] Errore inizializzazione MS5837: {}, uso simulazione".format(e))
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

                time.sleep(0.01)
        except Exception as e:
            print("[DEPTH] Errore: {}".format(e))
            time.sleep(0.1)

# ==================== PLOTTING THREAD ====================
def plot_thread():
    """Real-time plotting thread (matplotlib 2.1.1 compatible)"""
    if not MATPLOTLIB_AVAILABLE:
        return
    
    fig = plt.figure(figsize=(14, 10))
    
    # 3D trajectory
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('3D Trajectory')
    
    # 2D trajectory (X-Y)
    ax2 = fig.add_subplot(2, 3, 2)
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('Top View (X-Y)')
    ax2.grid(True)
    ax2.axis('equal')
    
    # Error plot
    ax3 = fig.add_subplot(2, 3, 3)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Position Error [m]')
    ax3.set_title('Estimation Error')
    ax3.grid(True)
    
    # Position components over time
    ax4 = fig.add_subplot(2, 3, 4)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Position [m]')
    ax4.set_title('Position Components')
    ax4.grid(True)
    
    # Depth plot
    ax5 = fig.add_subplot(2, 3, 5)
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('Depth [m]')
    ax5.set_title('Depth Tracking')
    ax5.grid(True)
    ax5.invert_yaxis()
    
    # Velocity components
    ax6 = fig.add_subplot(2, 3, 6)
    ax6.set_xlabel('Time [s]')
    ax6.set_ylabel('Velocity [m/s]')
    ax6.set_title('Velocity Components')
    ax6.grid(True)
    
    # Initialize plot elements
    line_true_3d, = ax1.plot([], [], [], 'b-', label='True', linewidth=1.5)
    line_est_3d, = ax1.plot([], [], [], 'r-', label='Estimated', linewidth=1.5)
    scatter_meas_3d = ax1.scatter([], [], [], c='g', marker='o', s=30, label='USBL Meas')
    
    line_true_2d, = ax2.plot([], [], 'b-', label='True', linewidth=1.5)
    line_est_2d, = ax2.plot([], [], 'r-', label='Estimated', linewidth=1.5)
    scatter_meas_2d = ax2.scatter([], [], c='g', marker='o', s=30, label='USBL Meas')
    
    line_error, = ax3.plot([], [], 'r-', linewidth=1.5)
    
    line_x, = ax4.plot([], [], 'r-', label='X', linewidth=1.5)
    line_y, = ax4.plot([], [], 'g-', label='Y', linewidth=1.5)
    line_z, = ax4.plot([], [], 'b-', label='Z', linewidth=1.5)
    
    line_true_depth, = ax5.plot([], [], 'b-', label='True', linewidth=1.5)
    line_est_depth, = ax5.plot([], [], 'r-', label='Estimated', linewidth=1.5)
    line_meas_depth, = ax5.plot([], [], 'go', label='Measured', markersize=3)
    
    for ax in [ax1, ax2, ax3, ax4, ax5, ax6]:
        ax.legend(loc='upper right')
    
    # Show full ground truth trajectory if enabled
    if SHOW_FULL_GROUND_TRUTH:
        full_circle = generate_full_circle(RADIUS, TILT_ANGLE)
        ax1.plot(full_circle[:,0], full_circle[:,1], full_circle[:,2], 
                'k--', alpha=0.3, linewidth=1, label='Ground Truth Path')
        ax2.plot(full_circle[:,0], full_circle[:,1], 
                'k--', alpha=0.3, linewidth=1, label='Ground Truth Path')
    
    plt.tight_layout()
    plt.ion()
    plt.show()
    
    while running:
        try:
            with plot_lock:
                if len(plot_data['time']) == 0:
                    time.sleep(0.1)
                    continue
                
                ts = list(plot_data['time'])
                true_arr = np.array([p for p in plot_data['true_pos']])
                est_arr = np.array([p for p in plot_data['est_pos']])
                errors = list(plot_data['error'])
                true_depth = list(plot_data['true_depth'])
                est_depth = list(plot_data['est_depth'])
                meas_depth = list(plot_data['meas_depth'])
                meas_pos = plot_data['meas_pos'][:]
            
            # Update 3D trajectory
            line_true_3d.set_data(true_arr[:,0], true_arr[:,1])
            line_true_3d.set_3d_properties(true_arr[:,2])
            line_est_3d.set_data(est_arr[:,0], est_arr[:,1])
            line_est_3d.set_3d_properties(est_arr[:,2])
            
            if meas_pos:
                meas_arr = np.array([p for t, p in meas_pos])
                scatter_meas_3d._offsets3d = (meas_arr[:,0], meas_arr[:,1], meas_arr[:,2])
            
            all_data = np.concatenate([true_arr, est_arr])
            margin = (all_data.max() - all_data.min()) * 0.1 + 1.0
            ax1.set_xlim(all_data[:,0].min() - margin, all_data[:,0].max() + margin)
            ax1.set_ylim(all_data[:,1].min() - margin, all_data[:,1].max() + margin)
            ax1.set_zlim(all_data[:,2].min() - margin, all_data[:,2].max() + margin)
            
            # Update 2D trajectory
            line_true_2d.set_data(true_arr[:,0], true_arr[:,1])
            line_est_2d.set_data(est_arr[:,0], est_arr[:,1])
            
            if meas_pos:
                meas_arr = np.array([p for t, p in meas_pos])
                scatter_meas_2d.set_offsets(meas_arr[:,:2])
            
            xy_margin = (all_data[:,:2].max() - all_data[:,:2].min()) * 0.1 + 1.0
            ax2.set_xlim(all_data[:,0].min() - xy_margin, all_data[:,0].max() + xy_margin)
            ax2.set_ylim(all_data[:,1].min() - xy_margin, all_data[:,1].max() + xy_margin)
            
            # Update error
            line_error.set_data(ts, errors)
            error_margin = max(errors) * 0.15 + 0.1
            ax3.set_ylim(0, max(errors) + error_margin)
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
        except Exception as e:
            print("[PLOT] Error: {}".format(e))
            time.sleep(0.1)
    
    plt.close('all')

# ==================== MAIN CONTROL LOOP ====================
def main():
    global running
    
    print("=" * 80)
    print("KALMAN FILTER SENSOR FUSION - EVENT-DRIVEN USBL")
    print("Control Loop: {}ms".format(CONTROL_LOOP_MS))
    print("Trajectory: Tilted Circle ({:.0f}°)".format(np.rad2deg(TILT_ANGLE)))
    print("USBL Request Interval: {:.1f}s".format(USBL_REQUEST_INTERVAL))
    print("Full Ground Truth: {}".format('ENABLED' if SHOW_FULL_GROUND_TRUTH else 'DISABLED'))
    print("Plotting: {}".format('ENABLED' if ENABLE_PLOTTING else 'DISABLED'))
    print("Sensors: {}".format('REAL+SIM' if USE_REAL_SENSORS else 'SIM ONLY'))
    print("Logging: {}".format('ENABLED' if ENABLE_LOGGING else 'DISABLED'))
    print("UDP Stream: {}".format('ENABLED' if ENABLE_UDP_STREAM else 'DISABLED'))
    print("=" * 80)
    
    # Initialize logging
    init_logging()
    
    # Initialize UDP streaming
    init_udp()
    
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
        threading.Thread(target=usbl_thread, daemon=True),
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
                
                # EVENT-DRIVEN: Check if new USBL data is available
                usbl_new_data = usbl['new_data']
                
                # Reset the new_data flag after reading it
                if usbl_new_data:
                    sensor_data['usbl']['new_data'] = False
            
            theta = OMEGA * t_sim
            p_true_vec = compute_tilted_circle_position(theta, RADIUS, TILT_ANGLE)
            p_true = p_true_vec.flatten()
            
            roll, pitch, yaw = imu['euler']
            a_body = np.array(imu['acc_body']).reshape((3,1))
            
            # EVENT-DRIVEN USBL UPDATE: Only use USBL when new data arrives
            if usbl_new_data and usbl['valid']:
                p_world = np.array(usbl['pos_world']).reshape((3,1))
                kf.update(p_world, roll, pitch, yaw, mode='usbl', dt_override=loop_time)
                usbl_fix_used = True
            else:
                kf.update(a_body, roll, pitch, yaw, mode='imu', dt_override=loop_time)
                usbl_fix_used = False
            
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
                     usbl_fix_used, pos_est, vel_est, acc_est, error_norm)
            
            if ENABLE_PLOTTING or ENABLE_UDP_STREAM:
                with plot_lock:
                    plot_data['time'].append(t_sim)
                    plot_data['true_pos'].append(p_true.copy())
                    plot_data['est_pos'].append(pos_est.copy())
                    plot_data['true_depth'].append(-p_true[2])
                    plot_data['est_depth'].append(-pos_est[2])
                    plot_data['meas_depth'].append(depth['depth'])
                    
                    if usbl_fix_used and usbl['valid']:
                        plot_data['meas_pos'].append((t_sim, np.array(usbl['pos_world'])))
                    if len(plot_data['time']) > 0:
                        t_min = plot_data['time'][0]
                        plot_data['meas_pos'] = [(t, p) for t, p in plot_data['meas_pos'] if t >= t_min]
                    plot_data['error'].append(error_norm)
            
            # Send UDP data
            send_udp_data()
            
            if iteration > 0:
                print('\033[16A\033[J', end='')
            
            print("\n{}".format('='*80))
            print("[{}] Iteration: {:4d}  Time: {:6.2f}s".format(timestamp, iteration, t_sim))
            print("{}".format('='*80))
            
            print("TRUE   Pos: [{:7.3f}, {:7.3f}, {:7.3f}]".format(p_true[0], p_true[1], p_true[2]))
            
            status = "OK" if imu['valid'] else "FAIL"
            print("IMU    [{}] Euler: [{:7.3f}, {:7.3f}, {:7.3f}]".format(status, roll, pitch, yaw))
            print("              Acc_b: [{:7.3f}, {:7.3f}, {:7.3f}]".format(a_body[0,0], a_body[1,0], a_body[2,0]))
            
            # USBL status with real/sim indicator and EVENT indicator
            if usbl_transponder is not None and usbl_transponder.is_connected():
                usbl_mode = "REAL"
            else:
                usbl_mode = "SIM "
            
            status = "FIX" if usbl_fix_used else "----"
            if usbl['valid']:
                extra_info = ""
                if usbl['integrity'] is not None:
                    extra_info = " Int:{}".format(usbl['integrity'])
                print("USBL   [{}][{}] Pos: [{:7.3f}, {:7.3f}, {:7.3f}]{}".format(
                    status, usbl_mode, usbl['pos_world'][0], usbl['pos_world'][1], 
                    usbl['pos_world'][2], extra_info))
            else:
                print("USBL   [----][{}] No data".format(usbl_mode))
            
            status = "OK" if depth['valid'] else "FAIL"
            print("DEPTH  [{}] Z: {:7.3f} m".format(status, depth['depth']))
            
            print("\nKF EST Pos: [{:7.3f}, {:7.3f}, {:7.3f}]".format(pos_est[0], pos_est[1], pos_est[2]))
            print("       Vel: [{:7.3f}, {:7.3f}, {:7.3f}]".format(vel_est[0], vel_est[1], vel_est[2]))
            print("       Acc: [{:7.3f}, {:7.3f}, {:7.3f}]".format(acc_est[0], acc_est[1], acc_est[2]))
            print("\nERROR: {:7.4f} m".format(error_norm), flush=True)
            
            iteration += 1
            
            elapsed = time.time() - start
            sleep_time = max(0, loop_time - elapsed)
            time.sleep(sleep_time)
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        running = False
        
        # Stop USBL transponder if running
        if usbl_transponder is not None:
            usbl_transponder.stop()
        
        close_logging()
        time.sleep(0.5)

if __name__ == "__main__":
    main()