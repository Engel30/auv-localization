#!/usr/bin/env python3
"""
EKF State Estimation: IMU + Depth + Range-Only USBL
====================================================
State vector: [x, y, z, vx, vy, vz, ax, ay, az]^T  (9 states)
Orientation (roll, pitch, yaw) is taken directly from IMU, not estimated.

Range-only USBL creates a sphere; depth slices it to a circle.
The EKF prior (predicted state + covariance) selects the most likely point.
"""

import time
import threading
import math
from collections import deque
import json
import os

import numpy as np

# =============================================================================
#  ROTATION MATRIX UTILITY
# =============================================================================
def Rxyz(roll, pitch, yaw):
    """
    Rotation matrix from body frame to world frame (ZYX Euler convention).
    R_world_body: v_world = R @ v_body
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [-sp,    cp*sr,             cp*cr]
    ])
    return R

# =============================================================================
#  SENSORI REALI
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

# =============================================================================
#  EVOLOGICS S2C MODEM (USBL RANGE)
# =============================================================================
EVOLOGICS_AVAILABLE = False
try:
    import socket
    EVOLOGICS_AVAILABLE = True
except ImportError:
    pass

# =============================================================================
#  MATPLOTLIB PER VISUALIZZAZIONE
# =============================================================================
try:
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False
    print("[WARNING] matplotlib non disponibile: visualizzazione disabilitata")

# =============================================================================
#  CONFIGURAZIONE
# =============================================================================
CONTROL_LOOP_HZ = 50.0
DT_DEFAULT = 1.0 / CONTROL_LOOP_HZ

IMU_PORT = "/dev/ttyUSB0"
DEPTH_I2C_BUS = 1

PLOT_HISTORY = 600  # ~12 s a 50 Hz

# ---- USBL Configuration -----------------------------------------------------
# Set to True for real USBL, False for simulation
USE_REAL_USBL = False

# EvoLogics modem settings (TCP connection)
USBL_HOST = "192.168.0.197"  # Default EvoLogics IP
USBL_PORT = 9200             # Default AT command port
USBL_TIMEOUT = 5.0           # Socket timeout [s]

# USBL transceiver position in world frame [m]
USBL_TRANSCEIVER_POS = np.array([0.0, 0.0, 0.0])

# Simulated USBL settings (for testing without hardware)
USBL_SIM_PERIOD = 1.0        # [s] simulated fix rate
USBL_SIM_NOISE_STD = 0.1     # [m] range noise std dev

# ---- FILE DI CALIBRAZIONE IMU -----------------------------------------------
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
        print("[CALIB] File di calibrazione trovato:", CALIBRATION_FILE)
        print("[CALIB] ACC_BIAS =", ACC_BIAS)
        print("[CALIB] GYR_BIAS =", GYR_BIAS)
    except Exception as e:
        print("[CALIB] Errore nella lettura di {}: {}".format(CALIBRATION_FILE, e))
else:
    print("[CALIB] Nessun file di calibrazione trovato, uso bias nulli.")

# =============================================================================
#  EKF NOISE MATRICES
# =============================================================================
# Process noise Q (acceleration random walk)
# Tuned for underwater robot dynamics
Q_pos = 0.001      # Position process noise [m²]
Q_vel = 0.01       # Velocity process noise [(m/s)²]
Q_acc = 0.1        # Acceleration process noise [(m/s²)²]

Q = np.diag([Q_pos, Q_pos, Q_pos,
             Q_vel, Q_vel, Q_vel,
             Q_acc, Q_acc, Q_acc])

# IMU accelerometer measurement noise
R_acc = np.diag([0.05, 0.05, 0.05])  # [(m/s²)²]

# Depth sensor measurement noise
R_depth = np.array([[0.001]])  # [m²] - depth sensors are very accurate

# USBL range measurement noise
R_range = np.array([[0.25]])  # [m²] - ~0.5m std dev typical for acoustic

# =============================================================================
#  EKF CLASS FOR RANGE-ONLY USBL
# =============================================================================
class RangeEKF:
    """
    Extended Kalman Filter for 9-state position estimation.
    
    State: [x, y, z, vx, vy, vz, ax, ay, az]^T
    
    Supports:
    - IMU acceleration update (linear)
    - Depth update (linear)  
    - Range-only USBL update (nonlinear)
    """
    
    def __init__(self, Q, R_imu, R_depth, R_range, dt=0.02):
        self.n = 9  # state dimension
        
        # State vector and covariance
        self.x = np.zeros((self.n, 1))
        self.P = np.eye(self.n) * 0.1
        
        # Noise matrices
        self.Q = Q.copy()
        self.R_imu = R_imu.copy()
        self.R_depth = R_depth.copy()
        self.R_range = R_range.copy()
        
        # Default timestep
        self.dt = dt
        
        # Measurement matrices (for linear updates)
        # C_imu: measures acceleration [ax, ay, az]
        self.C_imu = np.zeros((3, 9))
        self.C_imu[0, 6] = 1.0
        self.C_imu[1, 7] = 1.0
        self.C_imu[2, 8] = 1.0
        
        # C_depth: measures z position
        self.C_depth = np.zeros((1, 9))
        self.C_depth[0, 2] = 1.0
        
        # USBL transceiver position
        self.usbl_pos = USBL_TRANSCEIVER_POS.copy()
    
    def set_state(self, x0):
        """Set initial state."""
        self.x = x0.reshape((self.n, 1))
    
    def set_covariance(self, P0):
        """Set initial covariance."""
        self.P = P0.copy()
    
    def get_state(self):
        """Return current state estimate."""
        return self.x.copy()
    
    def get_covariance(self):
        """Return current covariance."""
        return self.P.copy()
    
    def _build_F(self, dt):
        """
        State transition matrix for constant acceleration model.
        x(k+1) = F @ x(k) + process_noise
        """
        F = np.eye(self.n)
        # Position depends on velocity
        F[0, 3] = dt  # x += vx * dt
        F[1, 4] = dt  # y += vy * dt
        F[2, 5] = dt  # z += vz * dt
        # Velocity depends on acceleration
        F[3, 6] = dt  # vx += ax * dt
        F[4, 7] = dt  # vy += ay * dt
        F[5, 8] = dt  # vz += az * dt
        # Position also depends on acceleration (1/2 * a * dt²)
        F[0, 6] = 0.5 * dt * dt
        F[1, 7] = 0.5 * dt * dt
        F[2, 8] = 0.5 * dt * dt
        return F
    
    def predict(self, dt=None):
        """EKF prediction step."""
        if dt is None:
            dt = self.dt
        if dt <= 0:
            return
            
        F = self._build_F(dt)
        
        # State prediction
        self.x = F @ self.x
        
        # Covariance prediction
        self.P = F @ self.P @ F.T + self.Q * dt
    
    def update_imu(self, acc_world, dt=None):
        """
        Update with IMU acceleration measurement (in world frame).
        
        Parameters
        ----------
        acc_world : ndarray (3,1)
            Acceleration in world frame [ax, ay, az]^T [m/s²]
        dt : float, optional
            Time since last update for prediction
        """
        if dt is not None and dt > 0:
            self.predict(dt)
        
        z = acc_world.reshape((3, 1))
        H = self.C_imu
        R = self.R_imu
        
        # Innovation
        y = z - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def update_depth(self, depth_z, dt=None):
        """
        Update with depth sensor measurement.
        
        Parameters
        ----------
        depth_z : float
            Depth as z coordinate (negative = below surface) [m]
        dt : float, optional
            Time since last update for prediction
        """
        if dt is not None and dt > 0:
            self.predict(dt)
        
        z = np.array([[depth_z]])
        H = self.C_depth
        R = self.R_depth
        
        # Innovation
        y = z - H @ self.x
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    
    def update_range(self, range_meas, dt=None):
        """
        Update with range-only USBL measurement (nonlinear).
        
        This is the key function for range-only localization.
        Range creates a sphere; combined with depth (z), it's a circle.
        The EKF prior selects the most likely point on that circle.
        
        Parameters
        ----------
        range_meas : float
            Measured range to USBL transceiver [m]
        dt : float, optional
            Time since last update for prediction
        """
        if dt is not None and dt > 0:
            self.predict(dt)
        
        # Current state estimate
        px, py, pz = self.x[0, 0], self.x[1, 0], self.x[2, 0]
        
        # Vector from USBL to robot
        dx = px - self.usbl_pos[0]
        dy = py - self.usbl_pos[1]
        dz = pz - self.usbl_pos[2]
        
        # Predicted range
        r_pred = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Avoid division by zero
        if r_pred < 1e-6:
            r_pred = 1e-6
        
        # Measurement model: h(x) = ||x - x_usbl||
        h = np.array([[r_pred]])
        
        # Jacobian of range measurement
        # H = [dx/r, dy/r, dz/r, 0, 0, 0, 0, 0, 0]
        H = np.zeros((1, self.n))
        H[0, 0] = dx / r_pred
        H[0, 1] = dy / r_pred
        H[0, 2] = dz / r_pred
        
        # Measurement
        z = np.array([[range_meas]])
        R = self.R_range
        
        # Innovation
        y = z - h
        
        # Innovation covariance
        S = H @ self.P @ H.T + R
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update (Joseph form)
        I_KH = np.eye(self.n) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        
        return r_pred, y[0, 0]  # Return predicted range and innovation

# =============================================================================
#  GLOBAL DATA STRUCTURES
# =============================================================================
sensor_data = {
    'imu': {
        'euler': [0.0, 0.0, 0.0],
        'acc_body': [0.0, 0.0, 0.0],
        'gyr_body': [0.0, 0.0, 0.0],
        'valid': False
    },
    'depth': {'depth': 0.0, 'valid': False},
    'usbl': {'range': 0.0, 'valid': False, 'timestamp': 0.0}
}
data_lock = threading.Lock()

kf_state = {
    'pos': [0.0, 0.0, 0.0],
    'vel': [0.0, 0.0, 0.0],
    'acc': [0.0, 0.0, 0.0],
    'roll': 0.0,
    'pitch': 0.0,
    'yaw': 0.0
}
kf_lock = threading.Lock()

plot_data = {
    'time': deque(maxlen=PLOT_HISTORY),
    'pos': deque(maxlen=PLOT_HISTORY),
    'depth': deque(maxlen=PLOT_HISTORY),
    'yaw': deque(maxlen=PLOT_HISTORY),
    'range': deque(maxlen=PLOT_HISTORY),
    'range_pred': deque(maxlen=PLOT_HISTORY)
}
plot_lock = threading.Lock()

running = True

# =============================================================================
#  THREAD: LETTURA IMU
# =============================================================================
def imu_thread():
    global running

    use_simulation = False
    sensor = None

    if XSENSE_AVAILABLE:
        try:
            sensor = MTI670(IMU_PORT)
            print("[IMU] Sensore XSens inizializzato")
        except Exception as e:
            print("[IMU] Errore inizializzazione IMU: {}".format(e))
            use_simulation = True
    else:
        print("[IMU] Libreria XSens non disponibile")
        use_simulation = True

    if use_simulation:
        print("[IMU] ATTENZIONE: nessun sensore reale, thread IMU inattivo")
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

            time.sleep(0.01)
        except Exception as e:
            print("[IMU] Eccezione lettura IMU: {}".format(e))
            sensor_data['imu']['valid'] = False
            time.sleep(0.1)

# =============================================================================
#  THREAD: LETTURA PROFONDIMETRO
# =============================================================================
def depth_thread():
    global running

    sensor = None
    use_simulation = False

    if MS5837_AVAILABLE:
        try:
            sensor = MS5837_30BA(bus=DEPTH_I2C_BUS)
            if sensor.init():
                print("[DEPTH] Sensore MS5837 inizializzato")
            else:
                print("[DEPTH] Init MS5837 fallita")
                use_simulation = True
        except Exception as e:
            print("[DEPTH] Errore inizializzazione MS5837: {}".format(e))
            use_simulation = True
    else:
        print("[DEPTH] Libreria MS5837 non disponibile")
        use_simulation = True

    if use_simulation:
        print("[DEPTH] ATTENZIONE: nessun sensore reale, thread depth inattivo")
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
            else:
                with data_lock:
                    sensor_data['depth']['valid'] = False
            time.sleep(0.1)
        except Exception as e:
            print("[DEPTH] Eccezione profondimetro: {}".format(e))
            sensor_data['depth']['valid'] = False
            time.sleep(0.2)

# =============================================================================
#  THREAD: USBL RANGE READER (EvoLogics S2C Modem)
# =============================================================================
def usbl_thread():
    """
    Read range measurements from EvoLogics S2C USBL modem.
    
    Uses instant messages with acknowledgement to get propagation time,
    which is then converted to range using sound speed.
    
    Expected notification format after AT*SENDIM:
    DELIVEREDIM,<addr>
    
    Then query AT?T for propagation time in microseconds.
    """
    global running
    
    if not USE_REAL_USBL:
        print("[USBL] Real USBL disabled, using simulation")
        _usbl_simulation_thread()
        return
    
    if not EVOLOGICS_AVAILABLE:
        print("[USBL] Socket not available, falling back to simulation")
        _usbl_simulation_thread()
        return
    
    sock = None
    connected = False
    
    # Sound speed [m/s] - should match modem setting (AT?CA)
    SOUND_SPEED = 1500.0
    
    # Remote transponder address
    REMOTE_ADDR = 2
    
    while running:
        # Connect to modem
        if not connected:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(USBL_TIMEOUT)
                sock.connect((USBL_HOST, USBL_PORT))
                connected = True
                print("[USBL] Connesso a {}:{}".format(USBL_HOST, USBL_PORT))
                
                # Switch to command mode
                time.sleep(0.5)
                sock.sendall(b"+++")
                time.sleep(1.5)
                _flush_socket(sock)
                
            except Exception as e:
                print("[USBL] Errore connessione: {}".format(e))
                connected = False
                time.sleep(2.0)
                continue
        
        try:
            # Send instant message with ACK to trigger range measurement
            cmd = "AT*SENDIM,1,{},ack,-\n".format(REMOTE_ADDR)
            sock.sendall(cmd.encode())
            
            # Wait for DELIVEREDIM or FAILEDIM
            response = _read_until_timeout(sock, timeout=3.0)
            
            if b"DELIVEREDIM" in response:
                # Query propagation time
                sock.sendall(b"AT?T\n")
                time.sleep(0.1)
                t_response = _read_until_timeout(sock, timeout=1.0)
                
                # Parse propagation time (microseconds)
                try:
                    # Response format: "123456\r\n" (microseconds)
                    t_str = t_response.decode().strip()
                    # Filter out any AT command echoes
                    for line in t_str.split('\n'):
                        line = line.strip()
                        if line.isdigit():
                            prop_time_us = float(line)
                            # Convert to range: one-way time * sound speed
                            # Note: AT?T returns round-trip time for burst data,
                            # but for IM ACK it's effectively one-way
                            range_m = (prop_time_us * 1e-6) * SOUND_SPEED / 2.0
                            
                            with data_lock:
                                sensor_data['usbl']['range'] = range_m
                                sensor_data['usbl']['valid'] = True
                                sensor_data['usbl']['timestamp'] = time.time()
                            
                            print("[USBL] Range: {:.2f} m (t={:.0f} us)".format(
                                range_m, prop_time_us))
                            break
                except (ValueError, AttributeError) as e:
                    print("[USBL] Errore parsing tempo: {}".format(e))
            
            elif b"FAILEDIM" in response:
                print("[USBL] Delivery failed")
                with data_lock:
                    sensor_data['usbl']['valid'] = False
            
            # Wait before next ping
            time.sleep(USBL_SIM_PERIOD)
            
        except socket.timeout:
            print("[USBL] Timeout")
            with data_lock:
                sensor_data['usbl']['valid'] = False
        except Exception as e:
            print("[USBL] Errore: {}".format(e))
            connected = False
            if sock:
                sock.close()
            time.sleep(1.0)
    
    if sock:
        sock.close()

def _flush_socket(sock):
    """Flush any pending data from socket."""
    sock.setblocking(False)
    try:
        while True:
            data = sock.recv(1024)
            if not data:
                break
    except:
        pass
    sock.setblocking(True)
    sock.settimeout(USBL_TIMEOUT)

def _read_until_timeout(sock, timeout=1.0):
    """Read from socket until timeout."""
    sock.settimeout(timeout)
    data = b""
    try:
        while True:
            chunk = sock.recv(1024)
            if not chunk:
                break
            data += chunk
    except socket.timeout:
        pass
    sock.settimeout(USBL_TIMEOUT)
    return data

def _usbl_simulation_thread():
    """Simulated USBL for testing without hardware."""
    global running
    
    print("[USBL-SIM] Thread simulazione USBL attivo")
    
    while running:
        time.sleep(USBL_SIM_PERIOD)
        
        # Get current estimated position to simulate range
        with kf_lock:
            pos = kf_state['pos']
        
        # True range to USBL transceiver
        dx = pos[0] - USBL_TRANSCEIVER_POS[0]
        dy = pos[1] - USBL_TRANSCEIVER_POS[1]
        dz = pos[2] - USBL_TRANSCEIVER_POS[2]
        true_range = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Add noise
        noisy_range = true_range + np.random.normal(0, USBL_SIM_NOISE_STD)
        noisy_range = max(0.1, noisy_range)  # Range must be positive
        
        with data_lock:
            sensor_data['usbl']['range'] = noisy_range
            sensor_data['usbl']['valid'] = True
            sensor_data['usbl']['timestamp'] = time.time()

# =============================================================================
#  THREAD: VISUALIZZAZIONE
# =============================================================================
def plot_thread():
    if not MATPLOTLIB_AVAILABLE:
        print("[PLOT] Matplotlib non disponibile")
        return

    plt.ion()
    fig = plt.figure(figsize=(14, 10))

    # -------------------------------------------------------------------------
    # 3D TRAIETTORIA + FRAME IMU
    # -------------------------------------------------------------------------
    ax_traj = fig.add_subplot(2, 3, 1, projection='3d')
    line_traj, = ax_traj.plot([], [], [], linewidth=2, label='Posizione EKF')
    
    axis_len_world = 0.3
    ax_traj.plot([0, axis_len_world], [0, 0], [0, 0], 'r-', linewidth=1)
    ax_traj.plot([0, 0], [0, axis_len_world], [0, 0], 'g-', linewidth=1)
    ax_traj.plot([0, 0], [0, 0], [0, axis_len_world], 'b-', linewidth=1)
    
    # USBL transceiver marker
    ax_traj.scatter([USBL_TRANSCEIVER_POS[0]], [USBL_TRANSCEIVER_POS[1]], 
                   [USBL_TRANSCEIVER_POS[2]], c='orange', s=100, marker='^',
                   label='USBL Transceiver')
    
    axis_len_body = 0.2
    body_x, = ax_traj.plot([], [], [], 'r-', linewidth=2, label='X corpo')
    body_y, = ax_traj.plot([], [], [], 'g-', linewidth=2, label='Y corpo')
    body_z, = ax_traj.plot([], [], [], 'b-', linewidth=2, label='Z corpo')
    
    ax_traj.set_xlabel('X [m]')
    ax_traj.set_ylabel('Y [m]')
    ax_traj.set_zlabel('Z [m]')
    ax_traj.set_title('Traiettoria 3D + Frame Robot')
    ax_traj.legend(loc='upper left', fontsize=8)
    ax_traj.grid(True)

    # -------------------------------------------------------------------------
    # ORIENTAZIONE IMU ALL'ORIGINE
    # -------------------------------------------------------------------------
    ax_ori = fig.add_subplot(2, 3, 2, projection='3d')
    ax_ori.plot([0, axis_len_world], [0, 0], [0, 0], 'r-', linewidth=1)
    ax_ori.plot([0, 0], [0, axis_len_world], [0, 0], 'g-', linewidth=1)
    ax_ori.plot([0, 0], [0, 0], [0, axis_len_world], 'b-', linewidth=1)
    
    body_x_o, = ax_ori.plot([], [], [], 'r-', linewidth=2, label='X corpo')
    body_y_o, = ax_ori.plot([], [], [], 'g-', linewidth=2, label='Y corpo')
    body_z_o, = ax_ori.plot([], [], [], 'b-', linewidth=2, label='Z corpo')
    
    ax_ori.set_xlabel('X')
    ax_ori.set_ylabel('Y')
    ax_ori.set_zlabel('Z')
    ax_ori.set_title('Orientazione IMU')
    ax_ori.legend(fontsize=8)
    ax_ori.grid(True)
    ax_ori.set_xlim(-axis_len_world, axis_len_world)
    ax_ori.set_ylim(-axis_len_world, axis_len_world)
    ax_ori.set_zlim(-axis_len_world, axis_len_world)

    # -------------------------------------------------------------------------
    # Profondità vs tempo
    # -------------------------------------------------------------------------
    ax_depth = fig.add_subplot(2, 3, 3)
    line_depth, = ax_depth.plot([], [], 'b-', linewidth=2, label='Depth')
    ax_depth.set_xlabel('Time [s]')
    ax_depth.set_ylabel('Depth [m]')
    ax_depth.set_title('Profondità')
    ax_depth.invert_yaxis()
    ax_depth.grid(True)
    ax_depth.legend()

    # -------------------------------------------------------------------------
    # Yaw vs tempo
    # -------------------------------------------------------------------------
    ax_yaw = fig.add_subplot(2, 3, 4)
    line_yaw, = ax_yaw.plot([], [], 'b-', linewidth=2, label='Yaw')
    ax_yaw.set_xlabel('Time [s]')
    ax_yaw.set_ylabel('Yaw [deg]')
    ax_yaw.set_title('Orientazione (Yaw)')
    ax_yaw.grid(True)
    ax_yaw.legend()

    # -------------------------------------------------------------------------
    # Range vs tempo (USBL)
    # -------------------------------------------------------------------------
    ax_range = fig.add_subplot(2, 3, 5)
    line_range, = ax_range.plot([], [], 'ro', markersize=4, label='Range USBL')
    line_range_pred, = ax_range.plot([], [], 'b-', linewidth=1, label='Range predicted')
    ax_range.set_xlabel('Time [s]')
    ax_range.set_ylabel('Range [m]')
    ax_range.set_title('USBL Range')
    ax_range.grid(True)
    ax_range.legend()

    # -------------------------------------------------------------------------
    # XY position (top-down view)
    # -------------------------------------------------------------------------
    ax_xy = fig.add_subplot(2, 3, 6)
    line_xy, = ax_xy.plot([], [], 'b-', linewidth=2, label='Traiettoria XY')
    ax_xy.scatter([USBL_TRANSCEIVER_POS[0]], [USBL_TRANSCEIVER_POS[1]], 
                 c='orange', s=100, marker='^', label='USBL')
    ax_xy.set_xlabel('X [m]')
    ax_xy.set_ylabel('Y [m]')
    ax_xy.set_title('Vista dall\'alto (XY)')
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
            # Traiettoria 3D
            line_traj.set_data(pos_arr[:, 0], pos_arr[:, 1])
            line_traj.set_3d_properties(pos_arr[:, 2])

            margin = 0.5
            x_min, x_max = pos_arr[:, 0].min() - margin, pos_arr[:, 0].max() + margin
            y_min, y_max = pos_arr[:, 1].min() - margin, pos_arr[:, 1].max() + margin
            z_min, z_max = pos_arr[:, 2].min() - margin, pos_arr[:, 2].max() + margin
            ax_traj.set_xlim(x_min, x_max)
            ax_traj.set_ylim(y_min, y_max)
            ax_traj.set_zlim(z_min, z_max)

            # Orientazione attuale
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

            # Frame orientazione all'origine
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

            # Profondità
            line_depth.set_data(ts, depth_arr)
            ax_depth.set_xlim(ts.min(), ts.max() + 0.1)
            d_min, d_max = depth_arr.min(), depth_arr.max()
            ax_depth.set_ylim(d_max + 0.2, d_min - 0.2)

            # Yaw
            line_yaw.set_data(ts, np.degrees(yaw_arr))
            ax_yaw.set_xlim(ts.min(), ts.max() + 0.1)
            yaw_deg = np.degrees(yaw_arr)
            ax_yaw.set_ylim(yaw_deg.min() - 10, yaw_deg.max() + 10)

            # Range
            # Filter valid range measurements (non-zero)
            valid_idx = range_arr > 0
            if np.any(valid_idx):
                line_range.set_data(ts[valid_idx], range_arr[valid_idx])
            line_range_pred.set_data(ts, range_pred_arr)
            ax_range.set_xlim(ts.min(), ts.max() + 0.1)
            r_all = np.concatenate([range_arr[valid_idx], range_pred_arr])
            if len(r_all) > 0:
                ax_range.set_ylim(max(0, r_all.min() - 0.5), r_all.max() + 0.5)

            # XY view
            line_xy.set_data(pos_arr[:, 0], pos_arr[:, 1])
            ax_xy.set_xlim(x_min, x_max)
            ax_xy.set_ylim(y_min, y_max)

        plt.pause(0.05)

    plt.ioff()
    plt.show()

# =============================================================================
#  MAIN: EKF REAL-TIME (IMU + DEPTH + RANGE-ONLY USBL)
# =============================================================================
def main():
    global running

    print("=" * 80)
    print("EKF State Estimation: IMU + Depth + Range-Only USBL")
    print("=" * 80)
    print("State: [x, y, z, vx, vy, vz, ax, ay, az]")
    print("Orientation (roll, pitch, yaw) from IMU directly")
    print("")
    print("USBL Mode: {}".format("REAL" if USE_REAL_USBL else "SIMULATION"))
    print("USBL Transceiver: {}".format(USBL_TRANSCEIVER_POS))
    print("Control Loop: {} Hz".format(CONTROL_LOOP_HZ))
    print("=" * 80)

    # Initialize EKF
    ekf = RangeEKF(Q, R_acc, R_depth, R_range, dt=DT_DEFAULT)

    # Initial state: at origin, stationary
    x0 = np.array([0.0, 0.0, 0.0,   # position
                   0.0, 0.0, 0.0,   # velocity
                   0.0, 0.0, 0.0])  # acceleration
    ekf.set_state(x0)

    # Initial covariance - moderate uncertainty
    P0 = np.diag([1.0, 1.0, 0.1,     # position (more certain in z due to depth)
                  0.1, 0.1, 0.1,     # velocity
                  0.1, 0.1, 0.1])    # acceleration
    ekf.set_covariance(P0)

    # Start sensor threads
    threads = [
        threading.Thread(target=imu_thread, daemon=True),
        threading.Thread(target=depth_thread, daemon=True),
        threading.Thread(target=usbl_thread, daemon=True),
    ]

    if MATPLOTLIB_AVAILABLE:
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

            # Get sensor data
            with data_lock:
                imu = sensor_data['imu'].copy()
                depth = sensor_data['depth'].copy()
                usbl = sensor_data['usbl'].copy()

            roll, pitch, yaw = imu['euler']

            # --- EKF Prediction (always) ---
            ekf.predict(dt)

            # --- IMU Update ---
            if imu['valid']:
                # Transform body acceleration to world frame
                a_body = np.array(imu['acc_body']).reshape((3, 1))
                R = Rxyz(roll, pitch, yaw)
                a_world = R @ a_body
                
                ekf.update_imu(a_world)

            # --- Depth Update ---
            if depth['valid']:
                depth_z = -depth['depth']  # z = -depth (negative below surface)
                ekf.update_depth(depth_z)

            # --- Range-Only USBL Update ---
            range_pred = 0.0
            range_innov = 0.0
            
            if usbl['valid'] and usbl['timestamp'] > last_usbl_time:
                last_usbl_time = usbl['timestamp']
                last_range_meas = usbl['range']
                
                range_pred, range_innov = ekf.update_range(usbl['range'])
                
                # Debug output
                # print("\n[USBL] Range={:.2f}m, Predicted={:.2f}m, Innovation={:.3f}m".format(
                #     usbl['range'], range_pred, range_innov))

            # --- Get estimated state ---
            state = ekf.get_state()
            pos = state[0:3, 0]
            vel = state[3:6, 0]
            acc = state[6:9, 0]

            # Compute current predicted range for plotting
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

            # Console output
            print("\r t={:6.2f}s | pos=[{:6.2f}, {:6.2f}, {:6.2f}]m | "
                  "rpy=[{:5.1f}, {:5.1f}, {:5.1f}]deg | "
                  "range={:5.2f}m".format(
                      t_rel, pos[0], pos[1], pos[2],
                      math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                      last_range_meas), end='')

            # Maintain loop rate
            elapsed = time.time() - now
            sleep_time = DT_DEFAULT - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[MAIN] Interruzione, chiusura...")
    finally:
        running = False
        time.sleep(0.5)


if __name__ == "__main__":
    main()