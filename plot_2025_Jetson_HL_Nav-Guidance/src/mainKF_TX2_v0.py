#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF State Estimation: IMU + Depth + USBL Transponder
Main script per veicolo con comunicazione acustica verso mission control
"""

import asyncio
import time
import threading
import math
import json
import os
import csv
from collections import deque
from datetime import datetime

import numpy as np

# =============================================================================
#  CONFIGURATION
# =============================================================================
CONTROL_LOOP_HZ = 50.0
DT_DEFAULT = 1.0 / CONTROL_LOOP_HZ

IMU_PORT = "/dev/ttyUSB0"
DEPTH_I2C_BUS = 1
PLOT_HISTORY = 600

ENABLE_GUI = False
ENABLE_LOGGING = True
LOG_DIRECTORY = "sensor_logs"

# USBL Transponder Configuration
USBL_IP = "192.168.0.232"
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0
MIN_INTEGRITY = 50

TRANSPONDER_ID = 3
TRANSCEIVER_ID = 2

USBL_SEND_INTERVAL = 2.0
USBL_TRANSCEIVER_POS = np.array([0.0, 0.0, 0.0])

# Serial Control Configuration
SERIAL_ENABLED = True
SERIAL_PORT = "/dev/ttyS0"
SERIAL_BAUDRATE = 115200

CALIBRATION_FILE = "imu_bias_calibration.json"
ACC_BIAS = np.zeros(3)
GYR_BIAS = np.zeros(3)

if os.path.exists(CALIBRATION_FILE):
    try:
        with open(CALIBRATION_FILE, "r") as f:
            calib = json.load(f)
        if "acc_bias" in calib:
            ACC_BIAS = np.array(calib["acc_bias"], dtype=float)
        if "gyr_bias" in calib:
            GYR_BIAS = np.array(calib["gyr_bias"], dtype=float)
        print("[CALIB] Loaded: {}".format(CALIBRATION_FILE))
    except Exception as e:
        print("[CALIB] Error: {}".format(e))

# =============================================================================
#  SENSOR IMPORTS
# =============================================================================
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

try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("[WARNING] matplotlib non disponibile")

try:
    from lib.USBL.usbl_transponder import USBLTransponderThread
    USBL_AVAILABLE = True
except ImportError:
    USBL_AVAILABLE = False
    print("[WARNING] usbl_transponder library non disponibile")

try:
    from lib.serial.serial_control import SerialControlThread
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARNING] serial_control library non disponibile")

# =============================================================================
#  DATA LOGGING CLASS
# =============================================================================
class SensorLogger(object):
    def __init__(self, log_dir=LOG_DIRECTORY):
        self.enabled = ENABLE_LOGGING
        if not self.enabled:
            return
        
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        self.imu_file = os.path.join(log_dir, "imu_{}.csv".format(timestamp))
        self.depth_file = os.path.join(log_dir, "depth_{}.csv".format(timestamp))
        self.usbl_file = os.path.join(log_dir, "usbl_{}.csv".format(timestamp))
        self.ekf_file = os.path.join(log_dir, "ekf_{}.csv".format(timestamp))
        
        self.t0 = None
        self.imu_count = 0
        self.depth_count = 0
        self.usbl_count = 0
        self.ekf_count = 0
        
        self.imu_fp = open(self.imu_file, 'w')
        self.depth_fp = open(self.depth_file, 'w')
        self.usbl_fp = open(self.usbl_file, 'w')
        self.ekf_fp = open(self.ekf_file, 'w')
        
        self.imu_writer = csv.writer(self.imu_fp)
        self.depth_writer = csv.writer(self.depth_fp)
        self.usbl_writer = csv.writer(self.usbl_fp)
        self.ekf_writer = csv.writer(self.ekf_fp)
        
        self.imu_writer.writerow(['timestamp_abs', 'timestamp_rel', 'roll', 'pitch', 'yaw',
                                  'acc_x', 'acc_y', 'acc_z', 'gyr_x', 'gyr_y', 'gyr_z'])
        self.depth_writer.writerow(['timestamp_abs', 'timestamp_rel', 'depth', 'temperature', 'pressure'])
        self.usbl_writer.writerow(['timestamp_abs', 'timestamp_rel', 'range', 'range_3d', 'integrity', 'rssi',
                                   'east', 'north', 'up', 'mode', 'use_range_only'])
        self.ekf_writer.writerow(['timestamp_abs', 'timestamp_rel', 'x', 'y', 'z',
                                  'vx', 'vy', 'vz', 'ax', 'ay', 'az', 'roll', 'pitch', 'yaw'])
        
        for fp in [self.imu_fp, self.depth_fp, self.usbl_fp, self.ekf_fp]:
            fp.flush()
        
        self.lock = threading.Lock()
        print("[LOG] Logging abilitato")
    
    def _get_timestamps(self):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        return now, now - self.t0
    
    def log_imu(self, roll, pitch, yaw, acc, gyr):
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.imu_writer.writerow([t_abs, t_rel, roll, pitch, yaw,
                                      acc[0], acc[1], acc[2], gyr[0], gyr[1], gyr[2]])
            self.imu_fp.flush()
            self.imu_count += 1
    
    def log_depth(self, depth, temperature=0.0, pressure=0.0):
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.depth_writer.writerow([t_abs, t_rel, depth, temperature, pressure])
            self.depth_fp.flush()
            self.depth_count += 1
    
    def log_usbl(self, range_m, integrity=0, rssi=0, east=0, north=0, up=0, 
                 range_3d=0, mode='ANGLES', use_range_only=False):
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.usbl_writer.writerow([t_abs, t_rel, range_m, range_3d, integrity, rssi, 
                                       east, north, up, mode, use_range_only])
            self.usbl_fp.flush()
            self.usbl_count += 1
    
    def log_ekf(self, pos, vel, acc, roll, pitch, yaw):
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.ekf_writer.writerow([t_abs, t_rel, pos[0], pos[1], pos[2],
                                      vel[0], vel[1], vel[2], acc[0], acc[1], acc[2],
                                      roll, pitch, yaw])
            self.ekf_fp.flush()
            self.ekf_count += 1
    
    def close(self):
        if not self.enabled:
            return
        print("[LOG] Campioni: IMU={} Depth={} USBL={} EKF={}".format(
            self.imu_count, self.depth_count, self.usbl_count, self.ekf_count))
        for fp in [self.imu_fp, self.depth_fp, self.usbl_fp, self.ekf_fp]:
            fp.close()


sensor_logger = None

# =============================================================================
#  ROTATION MATRIX
# =============================================================================
def Rxyz(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])

# =============================================================================
#  EKF
# =============================================================================
Q = np.diag([0.001, 0.001, 0.001, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1])
R_acc = np.diag([0.05, 0.05, 0.05])
R_depth = np.array([[0.001]])
R_range = np.array([[0.25]])


class RangeEKF(object):
    def __init__(self, Q, R_imu, R_depth, R_range, dt=0.02):
        self.n = 9
        self.x = np.zeros((self.n, 1))
        self.P = np.eye(self.n) * 0.1
        self.Q = Q.copy()
        self.R_imu = R_imu.copy()
        self.R_depth = R_depth.copy()
        self.R_range = R_range.copy()
        self.dt = dt
        
        self.C_imu = np.zeros((3, 9))
        self.C_imu[0, 6] = 1.0
        self.C_imu[1, 7] = 1.0
        self.C_imu[2, 8] = 1.0
        
        self.C_depth = np.zeros((1, 9))
        self.C_depth[0, 2] = 1.0
        
        self.usbl_pos = USBL_TRANSCEIVER_POS.copy()
    
    def set_state(self, x0):
        self.x = x0.reshape((self.n, 1))
    
    def set_covariance(self, P0):
        self.P = P0.copy()
    
    def get_state(self):
        return self.x.copy()
    
    def _build_F(self, dt):
        F = np.eye(self.n)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        F[3, 6] = dt
        F[4, 7] = dt
        F[5, 8] = dt
        F[0, 6] = 0.5 * dt * dt
        F[1, 7] = 0.5 * dt * dt
        F[2, 8] = 0.5 * dt * dt
        return F
    
    def predict(self, dt=None):
        if dt is None:
            dt = self.dt
        if dt <= 0:
            return
        F = self._build_F(dt)
        self.x = F.dot(self.x)
        self.P = F.dot(self.P).dot(F.T) + self.Q * dt
    
    def update_imu(self, acc_world):
        z = acc_world.reshape((3, 1))
        H = self.C_imu
        R = self.R_imu
        y = z - H.dot(self.x)
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        I_KH = np.eye(self.n) - K.dot(H)
        self.P = I_KH.dot(self.P).dot(I_KH.T) + K.dot(R).dot(K.T)
    
    def update_depth(self, depth_z):
        z = np.array([[depth_z]])
        H = self.C_depth
        R = self.R_depth
        y = z - H.dot(self.x)
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        I_KH = np.eye(self.n) - K.dot(H)
        self.P = I_KH.dot(self.P).dot(I_KH.T) + K.dot(R).dot(K.T)
    
    def update_range(self, range_meas):
        px, py, pz = self.x[0, 0], self.x[1, 0], self.x[2, 0]
        dx = px - self.usbl_pos[0]
        dy = py - self.usbl_pos[1]
        dz = pz - self.usbl_pos[2]
        r_pred = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if r_pred < 1e-6:
            r_pred = 1e-6
        
        h = np.array([[r_pred]])
        H = np.zeros((1, self.n))
        H[0, 0] = dx / r_pred
        H[0, 1] = dy / r_pred
        H[0, 2] = dz / r_pred
        
        z = np.array([[range_meas]])
        R = self.R_range
        y = z - h
        S = H.dot(self.P).dot(H.T) + R
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        self.x = self.x + K.dot(y)
        I_KH = np.eye(self.n) - K.dot(H)
        self.P = I_KH.dot(self.P).dot(I_KH.T) + K.dot(R).dot(K.T)
        
        return r_pred, y[0, 0]

# =============================================================================
#  GLOBAL DATA
# =============================================================================
sensor_data = {
    'imu': {'euler': [0.0, 0.0, 0.0], 'acc_body': [0.0, 0.0, 0.0],
            'gyr_body': [0.0, 0.0, 0.0], 'valid': False},
    'depth': {'depth': 0.0, 'valid': False, 'temperature': 0.0, 'pressure': 0.0},
    'usbl': {'range': 0.0, 'range_3d': 0.0, 'valid': False, 'timestamp': 0.0, 'new_data': False,
             'integrity': 0, 'rssi': 0, 'east': 0.0, 'north': 0.0, 'up': 0.0,
             'mode': 'ANGLES', 'use_range_only': False}
}
data_lock = threading.Lock()

kf_state = {
    'pos': [0.0, 0.0, 0.0], 'vel': [0.0, 0.0, 0.0], 'acc': [0.0, 0.0, 0.0],
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
    'x': 0.0, 'y': 0.0, 'z': 0.0
}
kf_lock = threading.Lock()

plot_data = {
    'time': deque(maxlen=PLOT_HISTORY), 'pos': deque(maxlen=PLOT_HISTORY),
    'depth': deque(maxlen=PLOT_HISTORY), 'yaw': deque(maxlen=PLOT_HISTORY),
    'range': deque(maxlen=PLOT_HISTORY), 'range_pred': deque(maxlen=PLOT_HISTORY)
}
plot_lock = threading.Lock()

running = True
usbl_transponder = None
serial_control = None

# Contatore messaggi per i log
msg_counter = {'rx': 0, 'tx': 0}

# =============================================================================
#  THREADS
# =============================================================================
def imu_thread():
    global running
    sensor = None
    use_sim = not XSENSE_AVAILABLE

    if XSENSE_AVAILABLE:
        try:
            sensor = MTI670(IMU_PORT)
            print("[IMU] XSens inizializzato")
        except Exception as e:
            print("[IMU] Errore: {}".format(e))
            use_sim = True

    if use_sim:
        print("[IMU] Nessun sensore, thread inattivo")
        while running:
            time.sleep(0.1)
        return

    while running:
        try:
            sensor.read_data()
            eul_deg = sensor.getEul()
            roll = math.radians(eul_deg[0])
            pitch = math.radians(eul_deg[1])
            yaw = math.radians(eul_deg[2])

            try:
                acc_raw = np.array(sensor.getFacc(), dtype=float)
            except AttributeError:
                acc_raw = np.array(sensor.getAcc(), dtype=float)

            gyr_raw = np.array(sensor.getGyro(), dtype=float)
            acc_corr = acc_raw - ACC_BIAS
            gyr_corr = gyr_raw - GYR_BIAS

            with data_lock:
                sensor_data['imu']['euler'] = [roll, pitch, yaw]
                sensor_data['imu']['acc_body'] = acc_corr.tolist()
                sensor_data['imu']['gyr_body'] = gyr_corr.tolist()
                sensor_data['imu']['valid'] = True

            if sensor_logger is not None:
                sensor_logger.log_imu(roll, pitch, yaw, acc_corr, gyr_corr)

            time.sleep(0.01)
        except Exception as e:
            print("[IMU] Eccezione: {}".format(e))
            sensor_data['imu']['valid'] = False
            time.sleep(0.1)


def depth_thread():
    global running
    sensor = None
    use_sim = not MS5837_AVAILABLE

    if MS5837_AVAILABLE:
        try:
            sensor = MS5837_30BA(bus=DEPTH_I2C_BUS)
            if sensor.init():
                print("[DEPTH] MS5837 inizializzato")
            else:
                use_sim = True
        except Exception as e:
            print("[DEPTH] Errore: {}".format(e))
            use_sim = True

    if use_sim:
        print("[DEPTH] Nessun sensore, thread inattivo")
        while running:
            time.sleep(0.2)
        return

    while running:
        try:
            if sensor.read():
                depth_val = sensor.depth()
                temp_val = sensor.temperature()
                press_val = sensor.pressure()
                
                with data_lock:
                    sensor_data['depth']['depth'] = depth_val
                    sensor_data['depth']['temperature'] = temp_val
                    sensor_data['depth']['pressure'] = press_val
                    sensor_data['depth']['valid'] = True
                
                if sensor_logger is not None:
                    sensor_logger.log_depth(depth_val, temp_val, press_val)
            else:
                with data_lock:
                    sensor_data['depth']['valid'] = False
            time.sleep(0.1)
        except Exception as e:
            print("[DEPTH] Eccezione: {}".format(e))
            sensor_data['depth']['valid'] = False
            time.sleep(0.2)


def on_mc_data_callback(data):
    """
    Callback quando arrivano dati dal transceiver.
    Stampa i log formattati nel terminale del main.
    """
    global msg_counter
    
    msg_counter['rx'] += 1
    
    usbl = data.get('usbl_data', {})
    ctrl = data.get('control_data', {})
    
    # Calcola ENU se mode e' ANGLES
    if usbl.get('mode') == 'LONG':
        e = usbl.get('enu', {}).get('east_m', 0)
        n = usbl.get('enu', {}).get('north_m', 0)
        u = usbl.get('enu', {}).get('up_m', 0)
    else:
        # Calcola da angoli
        bearing = usbl.get('angles', {}).get('bearing_rad', 0)
        elevation = usbl.get('angles', {}).get('elevation_rad', 0)
        distance = usbl.get('range_3d_m', 0)
        if distance > 0:
            horiz = distance * math.cos(elevation)
            e = horiz * math.sin(bearing)
            n = horiz * math.cos(bearing)
            u = distance * math.sin(elevation)
        else:
            e, n, u = 0, 0, 0
    
    range_3d = usbl.get('range_3d_m', 0)
    
    print("\n" + "=" * 70)
    print("[MSG #{}] TRANSCEIVER -> TRANSPONDER".format(msg_counter['rx']))
    print("=" * 70)
    print("  USBL Data (mode={}):".format(usbl.get('mode', 'UNKNOWN')))
    if usbl.get('mode') == 'LONG':
        print("    ENU:      E={:.3f}  N={:.3f}  U={:.3f} m".format(e, n, u))
    else:
        print("    Angles:   bearing={:.2f}deg  elev={:.2f}deg".format(
            usbl.get('angles', {}).get('bearing_deg', 0),
            usbl.get('angles', {}).get('elevation_deg', 0)))
        print("    Calc ENU: E={:.3f}  N={:.3f}  U={:.3f} m".format(e, n, u))
    print("    Range 3D: {:.3f} m".format(range_3d))
    print("    Quality:  integrity={}  RSSI={} dBm".format(
        usbl.get('integrity', 0), usbl.get('RSSI', 0)))
    print("  Control Data:")
    print("    enable={}  bias={:.2f}deg  freq={:.2f}Hz  use_range_only={}".format(
        ctrl.get('enable', False),
        ctrl.get('bias_angle_deg', 0),
        ctrl.get('frequency_hz', 0),
        ctrl.get('use_range_only', False)))
    print("-" * 70)


def on_telemetry_sent_callback(t):
    """
    Callback quando viene inviata telemetria al transceiver.
    Stampa i log formattati nel terminale del main.
    """
    global msg_counter
    
    msg_counter['tx'] += 1
    
    print("\n" + "=" * 70)
    print("[MSG #{}] TRANSPONDER -> TRANSCEIVER".format(msg_counter['tx']))
    print("=" * 70)
    print("  Telemetry:")
    print("    Position: x={:.3f}  y={:.3f}  z={:.3f} m".format(
        t['x'], t['y'], t['z']))
    print("    Orient:   roll={:.2f}  pitch={:.2f}  yaw={:.2f} deg".format(
        t['roll'], t['pitch'], t['yaw']))
    print("    Battery:  {:.1f}% @ {:.2f}V".format(t['battery_pct'], t['voltage']))
    print("    Sensors:  temp={:.1f}C  press={:.1f}mbar".format(
        t['temperature'], t['pressure']))
    print("-" * 70)


def usbl_thread():
    """Thread wrapper per USBL transponder."""
    global running, usbl_transponder, serial_control
    
    if not USBL_AVAILABLE:
        print("[USBL] Libreria non disponibile, thread inattivo")
        while running:
            time.sleep(0.5)
        return
    
    usbl_transponder = USBLTransponderThread(
        ip=USBL_IP,
        port=USBL_PORT,
        transceiver_id=TRANSCEIVER_ID,
        transponder_id=TRANSPONDER_ID,
        data_lock=data_lock,
        sensor_data_ref=sensor_data,
        logger=sensor_logger,
        send_interval=USBL_SEND_INTERVAL,
        log_directory=LOG_DIRECTORY,
        codec=CODEC,
        min_integrity=MIN_INTEGRITY,
        speed_of_sound=SPEED_OF_SOUND_MPS
    )
    
    usbl_transponder.telemetry_lock = kf_lock
    
    # Collega i callback per i log nel main
    usbl_transponder.on_mc_data_callback = on_mc_data_callback
    usbl_transponder.on_telemetry_sent_callback = on_telemetry_sent_callback
    
    # Collega il callback per la seriale
    if serial_control is not None:
        usbl_transponder.on_control_data_received = serial_control.on_control_data_received
        print("[USBL] Serial callback collegato")
    
    print("[USBL] Transponder {} -> Transceiver {}".format(TRANSPONDER_ID, TRANSCEIVER_ID))
    print("[USBL] Connecting to {}:{}".format(USBL_IP, USBL_PORT))
    
    usbl_transponder.run()


def serial_thread():
    """Thread per gestire la seriale che invia control data."""
    global running, serial_control
    
    if not SERIAL_AVAILABLE or not SERIAL_ENABLED:
        print("[SERIAL] Libreria non disponibile o disabilitata, thread inattivo")
        while running:
            time.sleep(0.5)
        return
    
    serial_control = SerialControlThread(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUDRATE
    )
    
    if serial_control.start():
        print("[SERIAL] Thread seriale avviato su {} @ {} baud".format(
            SERIAL_PORT, SERIAL_BAUDRATE))
        
        while running:
            time.sleep(0.5)
        
        serial_control.stop()
    else:
        print("[SERIAL] Impossibile avviare thread seriale")
        serial_control = None


def plot_thread():
    global running
    
    if not MATPLOTLIB_AVAILABLE or not ENABLE_GUI:
        return

    plt.ion()
    fig = plt.figure(figsize=(14, 10))

    ax_traj = fig.add_subplot(2, 3, 1, projection='3d')
    line_traj, = ax_traj.plot([], [], [], linewidth=2, label='Posizione EKF')
    ax_traj.scatter([USBL_TRANSCEIVER_POS[0]], [USBL_TRANSCEIVER_POS[1]], 
                   [USBL_TRANSCEIVER_POS[2]], c='orange', s=100, marker='^',
                   label='USBL Transceiver')
    ax_traj.set_xlabel('X [m]')
    ax_traj.set_ylabel('Y [m]')
    ax_traj.set_zlabel('Z [m]')
    ax_traj.set_title('Traiettoria 3D')
    ax_traj.legend(loc='upper left', fontsize=8)
    ax_traj.grid(True)

    ax_depth = fig.add_subplot(2, 3, 3)
    line_depth, = ax_depth.plot([], [], 'b-', linewidth=2)
    ax_depth.set_xlabel('Time [s]')
    ax_depth.set_ylabel('Depth [m]')
    ax_depth.set_title('Profondita')
    ax_depth.grid(True)

    ax_yaw = fig.add_subplot(2, 3, 4)
    line_yaw, = ax_yaw.plot([], [], 'b-', linewidth=2)
    ax_yaw.set_xlabel('Time [s]')
    ax_yaw.set_ylabel('Yaw [deg]')
    ax_yaw.set_title('Yaw')
    ax_yaw.grid(True)

    ax_range = fig.add_subplot(2, 3, 5)
    line_range, = ax_range.plot([], [], 'ro', markersize=4, label='Range USBL')
    line_range_pred, = ax_range.plot([], [], 'b-', linewidth=1, label='Range predicted')
    ax_range.set_xlabel('Time [s]')
    ax_range.set_ylabel('Range [m]')
    ax_range.set_title('USBL Range')
    ax_range.grid(True)
    ax_range.legend()

    ax_xy = fig.add_subplot(2, 3, 6)
    line_xy, = ax_xy.plot([], [], 'b-', linewidth=2)
    ax_xy.scatter([USBL_TRANSCEIVER_POS[0]], [USBL_TRANSCEIVER_POS[1]], 
                 c='orange', s=100, marker='^', label='USBL')
    ax_xy.set_xlabel('X [m]')
    ax_xy.set_ylabel('Y [m]')
    ax_xy.set_title('Vista XY')
    ax_xy.grid(True)
    ax_xy.legend()

    plt.tight_layout()

    while running:
        with plot_lock:
            has_data = len(plot_data['time']) > 1
            if has_data:
                ts = np.array(plot_data['time'])
                pos_arr = np.array(plot_data['pos'])
                depth_arr = np.array(plot_data['depth'])
                yaw_arr = np.array(plot_data['yaw'])
                range_arr = np.array(plot_data['range'])
                range_pred_arr = np.array(plot_data['range_pred'])

        if has_data:
            line_traj.set_data(pos_arr[:, 0], pos_arr[:, 1])
            line_traj.set_3d_properties(pos_arr[:, 2])

            line_depth.set_data(ts, -depth_arr)
            line_yaw.set_data(ts, np.degrees(yaw_arr))

            valid_idx = range_arr > 0
            if np.any(valid_idx):
                line_range.set_data(ts[valid_idx], range_arr[valid_idx])
            line_range_pred.set_data(ts, range_pred_arr)

            line_xy.set_data(pos_arr[:, 0], pos_arr[:, 1])

        plt.pause(0.05)

    plt.ioff()
    plt.show()

# =============================================================================
#  MAIN
# =============================================================================
def main():
    global running, sensor_logger, usbl_transponder, serial_control

    print("=" * 70)
    print("EKF: IMU + Depth + USBL Transponder")
    print("=" * 70)
    print("USBL: {}:{} (Transponder {} -> Transceiver {})".format(
        USBL_IP, USBL_PORT, TRANSPONDER_ID, TRANSCEIVER_ID))
    print("Serial: {} @ {} baud (Enabled: {})".format(
        SERIAL_PORT, SERIAL_BAUDRATE, SERIAL_ENABLED))
    print("Loop: {} Hz".format(CONTROL_LOOP_HZ))
    print("GUI: {}".format("ENABLED" if ENABLE_GUI else "DISABLED"))
    print("Logging: {}".format("ENABLED" if ENABLE_LOGGING else "DISABLED"))
    print("=" * 70)
    print("")
    print("Waiting for messages...")
    print("")

    sensor_logger = SensorLogger()
    time.sleep(0.2)

    ekf = RangeEKF(Q, R_acc, R_depth, R_range, dt=DT_DEFAULT)
    ekf.set_state(np.zeros(9))
    ekf.set_covariance(np.diag([1.0, 1.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]))

    threads = []
    
    if SERIAL_ENABLED and SERIAL_AVAILABLE:
        serial_t = threading.Thread(target=serial_thread, daemon=True)
        serial_t.start()
        threads.append(serial_t)
        time.sleep(0.5)
    
    threads.extend([
        threading.Thread(target=imu_thread, daemon=True),
        threading.Thread(target=depth_thread, daemon=True),
        threading.Thread(target=usbl_thread, daemon=True),
    ])

    if MATPLOTLIB_AVAILABLE and ENABLE_GUI:
        threads.append(threading.Thread(target=plot_thread, daemon=True))

    for t in threads[1 if SERIAL_ENABLED and SERIAL_AVAILABLE else 0:]:
        t.start()

    time.sleep(1.0)

    t0 = time.time()
    last_loop = t0
    last_range_meas = 0.0

    try:
        while True:
            now = time.time()
            dt = now - last_loop
            last_loop = now

            if dt <= 0.0 or dt > 1.0:
                dt = DT_DEFAULT

            with data_lock:
                imu = sensor_data['imu'].copy()
                depth = sensor_data['depth'].copy()
                usbl = sensor_data['usbl'].copy()
                if sensor_data['usbl']['new_data']:
                    sensor_data['usbl']['new_data'] = False

            roll, pitch, yaw = imu['euler']

            ekf.predict(dt)

            if imu['valid']:
                a_body = np.array(imu['acc_body']).reshape((3, 1))
                R = Rxyz(roll, pitch, yaw)
                a_world = R.dot(a_body)
                ekf.update_imu(a_world)

            if depth['valid']:
                ekf.update_depth(-depth['depth'])

            if usbl['new_data'] and usbl['valid']:
                range_for_filter = usbl.get('range_3d', 0.0)
                if range_for_filter <= 0:
                    range_for_filter = usbl['range']
                
                last_range_meas = range_for_filter
                r_pred, innov = ekf.update_range(range_for_filter)
                
                print("\n[EKF] Range update: meas={:.3f}m  pred={:.3f}m  innov={:.3f}m".format(
                    range_for_filter, r_pred, innov))

            state = ekf.get_state()
            pos = state[0:3, 0]
            vel = state[3:6, 0]
            acc = state[6:9, 0]

            if sensor_logger is not None:
                sensor_logger.log_ekf(pos, vel, acc, roll, pitch, yaw)

            with kf_lock:
                kf_state['pos'] = pos.tolist()
                kf_state['vel'] = vel.tolist()
                kf_state['acc'] = acc.tolist()
                kf_state['roll'] = math.degrees(roll)
                kf_state['pitch'] = math.degrees(pitch)
                kf_state['yaw'] = math.degrees(yaw)
                kf_state['x'] = pos[0]
                kf_state['y'] = pos[1]
                kf_state['z'] = pos[2]
            
            if usbl_transponder:
                usbl_transponder.update_telemetry(
                    pos[0], pos[1], pos[2],
                    math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                    temperature=depth.get('temperature', 20.0),
                    pressure=depth.get('pressure', 1013.0)
                )

            dx = pos[0] - USBL_TRANSCEIVER_POS[0]
            dy = pos[1] - USBL_TRANSCEIVER_POS[1]
            dz = pos[2] - USBL_TRANSCEIVER_POS[2]
            current_range_pred = math.sqrt(dx*dx + dy*dy + dz*dz)

            t_rel = now - t0
            with plot_lock:
                plot_data['time'].append(t_rel)
                plot_data['pos'].append(pos.copy())
                plot_data['depth'].append(depth['depth'] if depth['valid'] else -pos[2])
                plot_data['yaw'].append(yaw)
                plot_data['range'].append(last_range_meas if usbl['valid'] else 0.0)
                plot_data['range_pred'].append(current_range_pred)

            elapsed = time.time() - now
            sleep_time = DT_DEFAULT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[MAIN] Chiusura...")
    finally:
        running = False
        
        if usbl_transponder:
            usbl_transponder.stop()
            stats = usbl_transponder.transponder.get_stats()
            print("\n[USBL] Sent: {} Received: {}".format(
                stats['messages_sent'], stats['messages_received']))
        
        if serial_control:
            serial_control.stop()
            stats = serial_control.get_stats()
            print("[SERIAL] Sent: {} Errors: {}".format(
                stats['messages_sent'], stats['errors']))
        
        if sensor_logger is not None:
            sensor_logger.close()
        
        print("\n" + "=" * 70)
        print("MESSAGE SUMMARY")
        print("=" * 70)
        print("TX (Transponder -> Transceiver): {}".format(msg_counter['tx']))
        print("RX (Transceiver -> Transponder): {}".format(msg_counter['rx']))
        print("=" * 70)
        
        time.sleep(0.5)


if __name__ == "__main__":
    main()