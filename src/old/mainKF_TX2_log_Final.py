#!/usr/bin/env python3
"""
EKF State Estimation: IMU + Depth + Range-Only USBL
Async USBL communication compatible with EvoLogics S2C protocol
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

# ---- GUI ENABLE/DISABLE ----------------------------------------------------
ENABLE_GUI = True  # Set to False to disable plotting interface

# ---- DATA LOGGING -----------------------------------------------------------
ENABLE_LOGGING = True  # Set to False to disable CSV logging
LOG_DIRECTORY = "sensor_logs"  # Directory for log files

# USBL Configuration
USBL_IP = "192.168.0.232"
USBL_PORT = 9200
CODEC = "utf-8"
SPEED_OF_SOUND_MPS = 1500.0
MIN_INTEGRITY = 50

TRANSPONDER_ID = 3
TRANSCEIVER_ID = 2

# USBL transceiver position in world frame [m]
USBL_TRANSCEIVER_POS = np.array([0.0, 0.0, 0.0])

# Request interval [s]
USBL_REQUEST_INTERVAL = 2.0

# Calibration file
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
        print("[CALIB] Loaded:", CALIBRATION_FILE)
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

# =============================================================================
#  DATA LOGGING CLASS
# =============================================================================
class SensorLogger:
    """Logs sensor data to CSV files with synchronized timestamps."""
    
    def __init__(self, log_dir=LOG_DIRECTORY):
        self.enabled = ENABLE_LOGGING
        if not self.enabled:
            return
        
        # Create log directory
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Generate timestamp for filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # File paths
        self.imu_file = os.path.join(log_dir, "imu_{}.csv".format(timestamp))
        self.depth_file = os.path.join(log_dir, "depth_{}.csv".format(timestamp))
        self.usbl_file = os.path.join(log_dir, "usbl_{}.csv".format(timestamp))
        self.ekf_file = os.path.join(log_dir, "ekf_{}.csv".format(timestamp))
        
        # Reference time (will be set on first log)
        self.t0 = None
        
        # Open files and create writers
        self.imu_fp = open(self.imu_file, 'w', newline='')
        self.depth_fp = open(self.depth_file, 'w', newline='')
        self.usbl_fp = open(self.usbl_file, 'w', newline='')
        self.ekf_fp = open(self.ekf_file, 'w', newline='')
        
        self.imu_writer = csv.writer(self.imu_fp)
        self.depth_writer = csv.writer(self.depth_fp)
        self.usbl_writer = csv.writer(self.usbl_fp)
        self.ekf_writer = csv.writer(self.ekf_fp)
        
        # Write headers
        self.imu_writer.writerow([
            'timestamp_abs', 'timestamp_rel',
            'roll', 'pitch', 'yaw',
            'acc_x', 'acc_y', 'acc_z',
            'gyr_x', 'gyr_y', 'gyr_z'
        ])
        
        self.depth_writer.writerow([
            'timestamp_abs', 'timestamp_rel',
            'depth'
        ])
        
        self.usbl_writer.writerow([
            'timestamp_abs', 'timestamp_rel',
            'range', 'integrity', 'rssi'
        ])
        
        self.ekf_writer.writerow([
            'timestamp_abs', 'timestamp_rel',
            'x', 'y', 'z',
            'vx', 'vy', 'vz',
            'ax', 'ay', 'az',
            'roll', 'pitch', 'yaw'
        ])
        
        # Flush headers
        self.imu_fp.flush()
        self.depth_fp.flush()
        self.usbl_fp.flush()
        self.ekf_fp.flush()
        
        # Thread lock for writing
        self.lock = threading.Lock()
        
        print("[LOG] Logging abilitato")
        print("[LOG] IMU: {}".format(self.imu_file))
        print("[LOG] Depth: {}".format(self.depth_file))
        print("[LOG] USBL: {}".format(self.usbl_file))
        print("[LOG] EKF: {}".format(self.ekf_file))
    
    def _get_timestamps(self):
        """Get absolute and relative timestamps."""
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        return now, now - self.t0
    
    def log_imu(self, roll, pitch, yaw, acc, gyr):
        """Log IMU data."""
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.imu_writer.writerow([
                t_abs, t_rel,
                roll, pitch, yaw,
                acc[0], acc[1], acc[2],
                gyr[0], gyr[1], gyr[2]
            ])
            self.imu_fp.flush()
    
    def log_depth(self, depth):
        """Log depth data."""
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.depth_writer.writerow([
                t_abs, t_rel,
                depth
            ])
            self.depth_fp.flush()
    
    def log_usbl(self, range_m, integrity=0, rssi=0):
        """Log USBL data."""
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.usbl_writer.writerow([
                t_abs, t_rel,
                range_m, integrity, rssi
            ])
            self.usbl_fp.flush()
    
    def log_ekf(self, pos, vel, acc, roll, pitch, yaw):
        """Log EKF state estimate."""
        if not self.enabled:
            return
        t_abs, t_rel = self._get_timestamps()
        with self.lock:
            self.ekf_writer.writerow([
                t_abs, t_rel,
                pos[0], pos[1], pos[2],
                vel[0], vel[1], vel[2],
                acc[0], acc[1], acc[2],
                roll, pitch, yaw
            ])
            self.ekf_fp.flush()
    
    def close(self):
        """Close all log files."""
        if not self.enabled:
            return
        self.imu_fp.close()
        self.depth_fp.close()
        self.usbl_fp.close()
        self.ekf_fp.close()
        print("[LOG] Files chiusi")

# Global logger instance (initialized in main)
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
#  EKF NOISE MATRICES
# =============================================================================
Q = np.diag([0.001, 0.001, 0.001,
             0.01, 0.01, 0.01,
             0.1, 0.1, 0.1])

R_acc = np.diag([0.05, 0.05, 0.05])
R_depth = np.array([[0.001]])
R_range = np.array([[0.25]])

# =============================================================================
#  EKF CLASS
# =============================================================================
class RangeEKF:
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
    
    def get_covariance(self):
        return self.P.copy()
    
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
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q * dt
    
    def update_imu(self, acc_world, dt=None):
        if dt is not None and dt > 0:
            self.predict(dt)
        z = acc_world.reshape((3, 1))
        H = self.C_imu
        R = self.R_imu
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def update_depth(self, depth_z, dt=None):
        if dt is not None and dt > 0:
            self.predict(dt)
        z = np.array([[depth_z]])
        H = self.C_depth
        R = self.R_depth
        y = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def update_range(self, range_meas, dt=None):
        if dt is not None and dt > 0:
            self.predict(dt)
        
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
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        return r_pred, y[0, 0]

# =============================================================================
#  GLOBAL DATA
# =============================================================================
sensor_data = {
    'imu': {'euler': [0.0, 0.0, 0.0], 'acc_body': [0.0, 0.0, 0.0],
            'gyr_body': [0.0, 0.0, 0.0], 'valid': False},
    'depth': {'depth': 0.0, 'valid': False},
    'usbl': {'range': 0.0, 'valid': False, 'timestamp': 0.0, 'new_data': False}
}
data_lock = threading.Lock()

kf_state = {
    'pos': [0.0, 0.0, 0.0], 'vel': [0.0, 0.0, 0.0], 'acc': [0.0, 0.0, 0.0],
    'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0
}
kf_lock = threading.Lock()

plot_data = {
    'time': deque(maxlen=PLOT_HISTORY), 'pos': deque(maxlen=PLOT_HISTORY),
    'depth': deque(maxlen=PLOT_HISTORY), 'yaw': deque(maxlen=PLOT_HISTORY),
    'range': deque(maxlen=PLOT_HISTORY), 'range_pred': deque(maxlen=PLOT_HISTORY)
}
plot_lock = threading.Lock()

running = True
usbl_stats = {'requests': 0, 'responses': 0}

# =============================================================================
#  IMU THREAD
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

            # Log IMU data
            if sensor_logger is not None:
                sensor_logger.log_imu(roll, pitch, yaw, acc_corr, gyr_corr)

            time.sleep(0.01)
        except Exception as e:
            print("[IMU] Eccezione: {}".format(e))
            sensor_data['imu']['valid'] = False
            time.sleep(0.1)

# =============================================================================
#  DEPTH THREAD
# =============================================================================
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
                with data_lock:
                    sensor_data['depth']['depth'] = depth_val
                    sensor_data['depth']['valid'] = True
                
                # Log depth data
                if sensor_logger is not None:
                    sensor_logger.log_depth(depth_val)
            else:
                with data_lock:
                    sensor_data['depth']['valid'] = False
            time.sleep(0.1)
        except Exception as e:
            print("[DEPTH] Eccezione: {}".format(e))
            sensor_data['depth']['valid'] = False
            time.sleep(0.2)

# =============================================================================
#  USBL ASYNC COMMUNICATION (from working script)
# =============================================================================
def calculate_distance_3d(east, north, up):
    return math.sqrt(east**2 + north**2 + up**2)


def encode_message(target_id, payload):
    payload_str = json.dumps(payload, separators=(',', ':'))
    if len(payload_str) > 63:
        raise ValueError("Message too long: {} bytes".format(len(payload_str)))
    payload_str = "[{}".format(payload_str)
    command = "AT*SENDIM,p0,{},{},ack,{}".format(len(payload_str), target_id, payload_str)
    return "+++{}\n".format(command).encode(CODEC)


def decode_response(message):
    try:
        str_msg = message.decode(CODEC).strip()
        if not str_msg or not str_msg.startswith("+++"):
            return "UNKNOWN", {}
        
        parts = str_msg.split(":")
        
        if "AT" in str_msg and len(parts) >= 3:
            args = parts[2].split(",")
            keyword = args[0]
            
            if keyword == "DELIVEREDIM":
                return "DELIVEREDIM", {"target_id": args[1] if len(args) > 1 else "?"}
            
            elif keyword == "FAILEDIM":
                return "FAILEDIM", {"target_id": args[1] if len(args) > 1 else "?"}
            
            elif keyword == "RECVIM" and len(args) >= 9:
                try:
                    sender_id = args[3]
                    rssi = int(args[7])
                    integrity = int(args[8])
                    payload = None
                    if ',[' in str_msg:
                        _, payload = str_msg.rsplit(',[', 1)
                        payload = payload.strip()
                        if payload.startswith("{"):
                            try:
                                payload = json.loads(payload)
                            except:
                                pass
                    
                    return "RECVIM", {
                        "sender_id": sender_id, "integrity": integrity,
                        "valid": integrity > MIN_INTEGRITY, "rssi": rssi, "message": payload
                    }
                except (ValueError, IndexError) as e:
                    return "RECVIM_ERROR", {}
            
            elif keyword in {"SENDSTART", "SENDEND", "RECVSTART", "RECVEND", 
                           "USBLLONG", "USBLANGLES", "USBLPHYD", "USBLPHYP"}:
                return keyword, {"args": args}
        
        return "OTHER", {}
    except Exception as e:
        return "ERROR", {}


async def usbl_response_reader(reader, response_event):
    global running
    
    try:
        while running:
            line = await reader.readline()
            if not line:
                print("[USBL] Connection closed")
                break
            
            msg_type, data = decode_response(line)
            
            if msg_type == "DELIVEREDIM":
                print("[USBL] Request delivered")
            
            elif msg_type == "FAILEDIM":
                print("[USBL] Request FAILED")
                response_event.set()
            
            elif msg_type == "RECVIM":
                if data["valid"]:
                    msg = data["message"]
                    
                    # Check for position data (e, n, u) or range data (d or dist)
                    if msg:
                        range_val = None
                        
                        # Full position available -> compute range
                        if "e" in msg and "n" in msg and "u" in msg:
                            e, n, u = msg["e"], msg["n"], msg["u"]
                            range_val = calculate_distance_3d(e, n, u)
                            print("[USBL] Position: E={:.2f} N={:.2f} U={:.2f} -> Range={:.2f}m".format(
                                e, n, u, range_val))
                        
                        # Range-only data (custom format)
                        elif "d" in msg:
                            range_val = float(msg["d"])
                            print("[USBL] Range-only: {:.2f}m".format(range_val))
                        
                        elif "dist" in msg:
                            range_val = float(msg["dist"])
                            print("[USBL] Range-only: {:.2f}m".format(range_val))
                        
                        elif "range" in msg:
                            range_val = float(msg["range"])
                            print("[USBL] Range-only: {:.2f}m".format(range_val))
                        
                        if range_val is not None:
                            usbl_stats['responses'] += 1
                            with data_lock:
                                sensor_data['usbl']['range'] = range_val
                                sensor_data['usbl']['valid'] = True
                                sensor_data['usbl']['timestamp'] = time.time()
                                sensor_data['usbl']['new_data'] = True
                                sensor_data['usbl']['integrity'] = data.get('integrity', 0)
                                sensor_data['usbl']['rssi'] = data.get('rssi', 0)
                            
                            # Log USBL data
                            if sensor_logger is not None:
                                sensor_logger.log_usbl(
                                    range_val,
                                    data.get('integrity', 0),
                                    data.get('rssi', 0)
                                )
                            
                            response_event.set()
                else:
                    print("[USBL] Low integrity: {}".format(data.get('integrity', '?')))
                    
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print("[USBL] Reader error: {}".format(e))


async def usbl_requester(writer, response_event):
    global running
    
    await asyncio.sleep(1)
    request_count = 0
    
    try:
        while running:
            request_count += 1
            usbl_stats['requests'] = request_count
            response_event.clear()
            
            request_msg = {"r": request_count}
            
            print("[USBL] Sending request #{}".format(request_count))
            
            try:
                encoded = encode_message(TRANSCEIVER_ID, request_msg)
                writer.write(encoded)
                await writer.drain()
            except ValueError as e:
                print("[USBL] Encode error: {}".format(e))
                continue
            
            try:
                await asyncio.wait_for(response_event.wait(), timeout=10.0)
            except asyncio.TimeoutError:
                print("[USBL] Timeout")
            
            await asyncio.sleep(USBL_REQUEST_INTERVAL)
            
    except asyncio.CancelledError:
        pass
    except Exception as e:
        print("[USBL] Requester error: {}".format(e))


async def usbl_main():
    global running
    
    try:
        reader, writer = await asyncio.open_connection(USBL_IP, USBL_PORT)
        print("[USBL] Connected to {}:{}".format(USBL_IP, USBL_PORT))
        print("[USBL] Transponder {} -> Transceiver {}".format(TRANSPONDER_ID, TRANSCEIVER_ID))
    except Exception as e:
        print("[USBL] Connection failed: {}".format(e))
        return
    
    response_event = asyncio.Event()
    
    reader_task = asyncio.ensure_future(usbl_response_reader(reader, response_event))
    requester_task = asyncio.ensure_future(usbl_requester(writer, response_event))
    
    try:
        while running:
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass
    finally:
        reader_task.cancel()
        requester_task.cancel()
        try:
            await asyncio.gather(reader_task, requester_task)
        except asyncio.CancelledError:
            pass
        writer.close()


def usbl_thread():
    """Thread wrapper for async USBL communication."""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        loop.run_until_complete(usbl_main())
    except Exception as e:
        print("[USBL] Thread error: {}".format(e))
    finally:
        loop.close()

# =============================================================================
#  PLOT THREAD
# =============================================================================
def plot_thread():
    if not MATPLOTLIB_AVAILABLE or not ENABLE_GUI:
        return

    plt.ion()
    fig = plt.figure(figsize=(14, 10))

    ax_traj = fig.add_subplot(2, 3, 1, projection='3d')
    line_traj, = ax_traj.plot([], [], [], linewidth=2, label='Posizione EKF')
    
    axis_len_world = 0.3
    ax_traj.plot([0, axis_len_world], [0, 0], [0, 0], 'r-', linewidth=1)
    ax_traj.plot([0, 0], [0, axis_len_world], [0, 0], 'g-', linewidth=1)
    ax_traj.plot([0, 0], [0, 0], [0, axis_len_world], 'b-', linewidth=1)
    ax_traj.scatter([USBL_TRANSCEIVER_POS[0]], [USBL_TRANSCEIVER_POS[1]], 
                   [USBL_TRANSCEIVER_POS[2]], c='orange', s=100, marker='^',
                   label='USBL Transceiver')
    
    axis_len_body = 0.2
    body_x, = ax_traj.plot([], [], [], 'r-', linewidth=2)
    body_y, = ax_traj.plot([], [], [], 'g-', linewidth=2)
    body_z, = ax_traj.plot([], [], [], 'b-', linewidth=2)
    
    ax_traj.set_xlabel('X [m]')
    ax_traj.set_ylabel('Y [m]')
    ax_traj.set_zlabel('Z [m]')
    ax_traj.set_title('Traiettoria 3D')
    ax_traj.legend(loc='upper left', fontsize=8)
    ax_traj.grid(True)

    ax_ori = fig.add_subplot(2, 3, 2, projection='3d')
    ax_ori.plot([0, axis_len_world], [0, 0], [0, 0], 'r-', linewidth=1)
    ax_ori.plot([0, 0], [0, axis_len_world], [0, 0], 'g-', linewidth=1)
    ax_ori.plot([0, 0], [0, 0], [0, axis_len_world], 'b-', linewidth=1)
    body_x_o, = ax_ori.plot([], [], [], 'r-', linewidth=2)
    body_y_o, = ax_ori.plot([], [], [], 'g-', linewidth=2)
    body_z_o, = ax_ori.plot([], [], [], 'b-', linewidth=2)
    ax_ori.set_xlabel('X')
    ax_ori.set_ylabel('Y')
    ax_ori.set_zlabel('Z')
    ax_ori.set_title('Orientazione IMU')
    ax_ori.grid(True)
    ax_ori.set_xlim(-axis_len_world, axis_len_world)
    ax_ori.set_ylim(-axis_len_world, axis_len_world)
    ax_ori.set_zlim(-axis_len_world, axis_len_world)

    ax_depth = fig.add_subplot(2, 3, 3)
    line_depth, = ax_depth.plot([], [], 'b-', linewidth=2)
    ax_depth.set_xlabel('Time [s]')
    ax_depth.set_ylabel('Depth [m]')
    ax_depth.set_title('Profondita (negativo = sotto superficie)')
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
    ax_xy.axis('equal')

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

            margin = 0.5
            x_min, x_max = pos_arr[:, 0].min() - margin, pos_arr[:, 0].max() + margin
            y_min, y_max = pos_arr[:, 1].min() - margin, pos_arr[:, 1].max() + margin
            z_min, z_max = pos_arr[:, 2].min() - margin, pos_arr[:, 2].max() + margin
            ax_traj.set_xlim(x_min, x_max)
            ax_traj.set_ylim(y_min, y_max)
            ax_traj.set_zlim(z_min, z_max)

            with data_lock:
                roll, pitch, yaw = sensor_data['imu']['euler']

            R = Rxyz(roll, pitch, yaw)
            p = pos_arr[-1, :]

            ex = p + axis_len_body * R[:, 0]
            ey = p + axis_len_body * R[:, 1]
            ez = p + axis_len_body * R[:, 2]

            body_x.set_data([p[0], ex[0]], [p[1], ex[1]])
            body_x.set_3d_properties([p[2], ex[2]])
            body_y.set_data([p[0], ey[0]], [p[1], ey[1]])
            body_y.set_3d_properties([p[2], ey[2]])
            body_z.set_data([p[0], ez[0]], [p[1], ez[1]])
            body_z.set_3d_properties([p[2], ez[2]])

            origin = np.array([0.0, 0.0, 0.0])
            ex_o = origin + axis_len_body * R[:, 0]
            ey_o = origin + axis_len_body * R[:, 1]
            ez_o = origin + axis_len_body * R[:, 2]

            body_x_o.set_data([origin[0], ex_o[0]], [origin[1], ex_o[1]])
            body_x_o.set_3d_properties([origin[2], ex_o[2]])
            body_y_o.set_data([origin[0], ey_o[0]], [origin[1], ey_o[1]])
            body_y_o.set_3d_properties([origin[2], ey_o[2]])
            body_z_o.set_data([origin[0], ez_o[0]], [origin[1], ez_o[1]])
            body_z_o.set_3d_properties([origin[2], ez_o[2]])

            # Plot depth as negative (below surface)
            depth_plot = -depth_arr
            line_depth.set_data(ts, depth_plot)
            ax_depth.set_xlim(ts.min(), ts.max() + 0.1)
            d_min, d_max = depth_plot.min(), depth_plot.max()
            ax_depth.set_ylim(d_min - 0.2, d_max + 0.2)

            line_yaw.set_data(ts, np.degrees(yaw_arr))
            ax_yaw.set_xlim(ts.min(), ts.max() + 0.1)
            yaw_deg = np.degrees(yaw_arr)
            ax_yaw.set_ylim(yaw_deg.min() - 10, yaw_deg.max() + 10)

            valid_idx = range_arr > 0
            if np.any(valid_idx):
                line_range.set_data(ts[valid_idx], range_arr[valid_idx])
            line_range_pred.set_data(ts, range_pred_arr)
            ax_range.set_xlim(ts.min(), ts.max() + 0.1)
            r_all = np.concatenate([range_arr[valid_idx], range_pred_arr]) if np.any(valid_idx) else range_pred_arr
            if len(r_all) > 0:
                ax_range.set_ylim(max(0, r_all.min() - 0.5), r_all.max() + 0.5)

            line_xy.set_data(pos_arr[:, 0], pos_arr[:, 1])
            ax_xy.set_xlim(x_min, x_max)
            ax_xy.set_ylim(y_min, y_max)

        plt.pause(0.05)

    plt.ioff()
    plt.show()

# =============================================================================
#  MAIN
# =============================================================================
def main():
    global running, sensor_logger

    print("=" * 70)
    print("EKF: IMU + Depth + Range-Only USBL")
    print("=" * 70)
    print("USBL: {}:{} (Transponder {} -> Transceiver {})".format(
        USBL_IP, USBL_PORT, TRANSPONDER_ID, TRANSCEIVER_ID))
    print("Loop: {} Hz".format(CONTROL_LOOP_HZ))
    print("GUI: {}".format("ENABLED" if ENABLE_GUI else "DISABLED"))
    print("Logging: {}".format("ENABLED" if ENABLE_LOGGING else "DISABLED"))
    print("=" * 70)

    # Initialize logger
    sensor_logger = SensorLogger()

    ekf = RangeEKF(Q, R_acc, R_depth, R_range, dt=DT_DEFAULT)

    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    ekf.set_state(x0)

    P0 = np.diag([1.0, 1.0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    ekf.set_covariance(P0)

    threads = [
        threading.Thread(target=imu_thread, daemon=True),
        threading.Thread(target=depth_thread, daemon=True),
        threading.Thread(target=usbl_thread, daemon=True),
    ]

    if MATPLOTLIB_AVAILABLE and ENABLE_GUI:
        threads.append(threading.Thread(target=plot_thread, daemon=True))

    for t in threads:
        t.start()

    time.sleep(0.5)

    t0 = time.time()
    last_loop = t0
    last_usbl_time = 0.0
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
                # Reset new_data flag
                if sensor_data['usbl']['new_data']:
                    sensor_data['usbl']['new_data'] = False

            roll, pitch, yaw = imu['euler']

            # Prediction
            ekf.predict(dt)

            # IMU Update
            if imu['valid']:
                a_body = np.array(imu['acc_body']).reshape((3, 1))
                R = Rxyz(roll, pitch, yaw)
                a_world = R @ a_body
                ekf.update_imu(a_world)

            # Depth Update
            if depth['valid']:
                depth_z = -depth['depth']
                ekf.update_depth(depth_z)

            # Range-Only USBL Update (only on new data)
            if usbl['new_data'] and usbl['valid']:
                last_usbl_time = usbl['timestamp']
                last_range_meas = usbl['range']
                
                r_pred, innov = ekf.update_range(usbl['range'])
                print("\n[EKF] Range update: meas={:.2f}m pred={:.2f}m innov={:.3f}m".format(
                    usbl['range'], r_pred, innov))

            # Get state
            state = ekf.get_state()
            pos = state[0:3, 0]
            vel = state[3:6, 0]
            acc = state[6:9, 0]

            # Log EKF state
            if sensor_logger is not None:
                sensor_logger.log_ekf(pos, vel, acc, roll, pitch, yaw)

            dx = pos[0] - USBL_TRANSCEIVER_POS[0]
            dy = pos[1] - USBL_TRANSCEIVER_POS[1]
            dz = pos[2] - USBL_TRANSCEIVER_POS[2]
            current_range_pred = math.sqrt(dx*dx + dy*dy + dz*dz)

            with kf_lock:
                kf_state['pos'] = pos.tolist()
                kf_state['vel'] = vel.tolist()
                kf_state['acc'] = acc.tolist()
                kf_state['roll'] = roll
                kf_state['pitch'] = pitch
                kf_state['yaw'] = yaw

            t_rel = now - t0
            with plot_lock:
                plot_data['time'].append(t_rel)
                plot_data['pos'].append(pos.copy())
                plot_data['depth'].append(depth['depth'] if depth['valid'] else -pos[2])
                plot_data['yaw'].append(yaw)
                plot_data['range'].append(last_range_meas if usbl['valid'] else 0.0)
                plot_data['range_pred'].append(current_range_pred)

            print("\r t={:6.2f}s pos=[{:6.2f},{:6.2f},{:6.2f}]m rpy=[{:5.1f},{:5.1f},{:5.1f}]deg range={:5.2f}m".format(
                t_rel, pos[0], pos[1], pos[2],
                math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                last_range_meas), end='')

            elapsed = time.time() - now
            sleep_time = DT_DEFAULT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[MAIN] Chiusura...")
    finally:
        running = False
        print("\n[USBL] Requests: {} Responses: {}".format(
            usbl_stats['requests'], usbl_stats['responses']))
        
        # Close logger
        if sensor_logger is not None:
            sensor_logger.close()
        
        time.sleep(0.5)


if __name__ == "__main__":
    main()