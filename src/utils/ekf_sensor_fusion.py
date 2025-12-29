#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Sensor Fusion per AUV
Fonde IMU, USBL, e Depth sensor con Extended Kalman Filter

Caratteristiche:
- Analisi automatica matrici covarianza dai dati
- Modello constant-velocity (più stabile per AUV)
- Correzione bias IMU
- Update USBL range-only (ignora angles)
"""

import os
import glob
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import LineCollection

# ============================================================================
# CONFIGURAZIONE
# ============================================================================

# Coordinate riferimenti (in metri)
BOA_COORDINATES = (2.0, 0.0)
NORTH_MARKER_COORDINATES = (0.0, 8.0)
POOL_RADIUS = 8.0

DATA_DIR = "sensor_logs"

# FLAGS
APPLY_BIAS_CORRECTION = True
VERBOSE = True

# ============================================================================
# FUNZIONI UTILITÀ
# ============================================================================

def find_latest_csv(directory, prefix):
    pattern = os.path.join(directory, f"{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"Nessun file trovato con pattern: {pattern}")
    return max(files, key=os.path.getmtime)

# ============================================================================
# ANALISI AUTOMATICA COVARIANZE
# ============================================================================

def analyze_sensor_noise(imu_data, usbl_data, depth_data, imu_calibration=None):
    """
    Analizza i dati dei sensori per stimare le matrici di covarianza
    
    Args:
        imu_data: DataFrame con dati IMU
        usbl_data: DataFrame con dati USBL
        depth_data: DataFrame con dati depth
        imu_calibration: dict con calibrazione statica IMU (opzionale)
                        {'bias_acc': [x,y,z], 'var_acc': [x,y,z], 
                         'bias_gyr': [x,y,z], 'var_gyr': [x,y,z]}
    """
    print("\n" + "="*70)
    print("ANALISI RUMORE SENSORI")
    print("="*70)
    
    # --- IMU ---
    if imu_calibration is not None:
        # Usa calibrazione statica precalcolata
        print("\n✓ Usando calibrazione statica IMU")
        bias_acc = np.array(imu_calibration['bias_acc'])
        bias_gyr = np.array(imu_calibration['bias_gyr'])
        var_acc = np.array(imu_calibration['var_acc'])
        var_gyr = np.array(imu_calibration['var_gyr'])
        
        print(f"\nIMU (da calibrazione statica):")
        print(f"  Bias accelerometro: [{bias_acc[0]:.6f}, {bias_acc[1]:.6f}, {bias_acc[2]:.6f}] m/s²")
        print(f"  Std accelerometro:  [{np.sqrt(var_acc[0]):.6f}, {np.sqrt(var_acc[1]):.6f}, {np.sqrt(var_acc[2]):.6f}] m/s²")
        print(f"  Bias giroscopio:    [{bias_gyr[0]:.6f}, {bias_gyr[1]:.6f}, {bias_gyr[2]:.6f}] rad/s")
        print(f"  Std giroscopio:     [{np.sqrt(var_gyr[0]):.6f}, {np.sqrt(var_gyr[1]):.6f}, {np.sqrt(var_gyr[2]):.6f}] rad/s")
    else:
        # Fallback: stima dai dati in movimento (IMPRECISO)
        print("\n⚠ ATTENZIONE: Stimando bias/noise da dati in movimento")
        print("  Questo NON è corretto! Usa calibrazione statica.")
        
        acc_x = imu_data['acc_x'].values
        acc_y = imu_data['acc_y'].values
        acc_z = imu_data['acc_z'].values
        
        gyr_x = imu_data['gyr_x'].values
        gyr_y = imu_data['gyr_y'].values
        gyr_z = imu_data['gyr_z'].values
        
        bias_acc = np.array([np.mean(acc_x), np.mean(acc_y), np.mean(acc_z)])
        bias_gyr = np.array([np.mean(gyr_x), np.mean(gyr_y), np.mean(gyr_z)])
        var_acc = np.array([np.var(acc_x), np.var(acc_y), np.var(acc_z)])
        var_gyr = np.array([np.var(gyr_x), np.var(gyr_y), np.var(gyr_z)])
        
        print(f"\nIMU (stima approssimativa):")
        print(f"  Bias accelerometro: [{bias_acc[0]:.4f}, {bias_acc[1]:.4f}, {bias_acc[2]:.4f}] m/s²")
        print(f"  Std accelerometro:  [{np.sqrt(var_acc[0]):.4f}, {np.sqrt(var_acc[1]):.4f}, {np.sqrt(var_acc[2]):.4f}] m/s²")
        print(f"  Bias giroscopio:    [{bias_gyr[0]:.4f}, {bias_gyr[1]:.4f}, {bias_gyr[2]:.4f}] rad/s")
        print(f"  Std giroscopio:     [{np.sqrt(var_gyr[0]):.4f}, {np.sqrt(var_gyr[1]):.4f}, {np.sqrt(var_gyr[2]):.4f}] rad/s")
    
    # --- USBL ---
    usbl_range = usbl_data['range'].values
    var_usbl = np.var(usbl_range)
    
    print(f"\nUSBL:")
    print(f"  Range medio: {np.mean(usbl_range):.3f} m")
    print(f"  Std range:   {np.sqrt(var_usbl):.3f} m")
    
    # --- DEPTH ---
    if 'depth' in depth_data.columns:
        depth = depth_data['depth'].values
        var_depth = np.var(depth)
        print(f"\nDepth:")
        print(f"  Profondità media: {np.mean(depth):.3f} m")
        print(f"  Std depth:        {np.sqrt(var_depth):.4f} m")
    else:
        var_depth = 0.01  # Default
        print(f"\nDepth: usando varianza default {var_depth}")
    
    return {
        'bias_acc': bias_acc,
        'bias_gyr': bias_gyr,
        'var_acc': var_acc,
        'var_gyr': var_gyr,
        'var_usbl': var_usbl,
        'var_depth': var_depth
    }

# ============================================================================
# EXTENDED KALMAN FILTER
# ============================================================================

class EKF_AUV:
    """
    Extended Kalman Filter per AUV
    
    Stato (6D): [x, y, z, vx, vy, vz]
    - Posizione 3D
    - Velocità 3D
    
    Modello: Constant velocity (più stabile del constant acceleration)
    """
    
    def __init__(self, sensor_noise, dt=0.02, config=None, initial_state=None, boa_position=None):
        """
        Inizializza EKF
        
        Args:
            sensor_noise: dict con varianze sensori
            dt: time step [s]
            config: dict con parametri di configurazione (opzionale)
            initial_state: dict con posizione e velocità iniziali (opzionale)
                          {'position': [x, y, z], 'velocity': [vx, vy, vz]}
            boa_position: posizione BOA/transponder USBL [x, y, z] (opzionale)
                         Default: [0, 0, 0] (origine)
        """
        self.dt = dt
        self.sensor_noise = sensor_noise
        
        # Posizione BOA (per calcolo range USBL)
        if boa_position is None:
            self.boa_position = np.array([0.0, 0.0, 0.0])
        else:
            self.boa_position = np.array(boa_position)
            if VERBOSE:
                print(f"\nPosizione BOA: [{boa_position[0]:.2f}, {boa_position[1]:.2f}, {boa_position[2]:.2f}] m")
        
        # Carica configurazione (usa default se non fornita)
        if config is None:
            config = {
                'Q_POSITION': 0.01,
                'Q_VELOCITY': 0.5,
                'Q_VELOCITY_Z': 0.1,
                'R_USBL_FACTOR': 1.5,
                'R_DEPTH_MIN': 0.01,
                'P0_POSITION_XY': 1.0,
                'P0_POSITION_Z': 0.1,
                'P0_VELOCITY': 0.01,
                'ACC_INTEGRATION_WEIGHT': 0.1
            }
        
        self.config = config
        self.acc_weight = config.get('ACC_INTEGRATION_WEIGHT', 0.1)
        
        # Stato: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        
        # Applica stato iniziale se fornito
        if initial_state is not None:
            if 'position' in initial_state:
                pos = initial_state['position']
                self.x[0:3] = pos
                if VERBOSE:
                    print(f"\nPosizione iniziale: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] m")
            if 'velocity' in initial_state:
                vel = initial_state['velocity']
                self.x[3:6] = vel
                if VERBOSE:
                    print(f"Velocità iniziale:  [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}] m/s")
        
        # Covarianza stato
        self.P = np.diag([
            config.get('P0_POSITION_XY', 1.0),
            config.get('P0_POSITION_XY', 1.0),
            config.get('P0_POSITION_Z', 0.1),
            config.get('P0_VELOCITY', 0.01),
            config.get('P0_VELOCITY', 0.01),
            config.get('P0_VELOCITY', 0.01)
        ])
        
        # Process noise Q
        q_pos = config.get('Q_POSITION', 0.01)
        q_vel = config.get('Q_VELOCITY', 0.5)
        q_vel_z = config.get('Q_VELOCITY_Z', 0.1)
        
        self.Q = np.diag([
            q_pos, q_pos, q_pos,
            q_vel, q_vel, q_vel_z
        ])
        
        # Measurement noise R
        r_usbl_factor = config.get('R_USBL_FACTOR', 1.5)
        r_depth_min = config.get('R_DEPTH_MIN', 0.01)
        
        self.R_usbl = np.array([[sensor_noise['var_usbl'] * r_usbl_factor]])  
        self.R_depth = np.array([[max(sensor_noise['var_depth'], r_depth_min)]])
        
        if VERBOSE:
            print(f"\n" + "="*70)
            print("INIZIALIZZAZIONE EKF")
            print("="*70)
            print(f"\nMatrice Q (process noise):")
            print(f"  Posizione: {q_pos} m²")
            print(f"  Velocità:  {q_vel} m²/s²")
            print(f"  Velocità Z: {q_vel_z} m²/s²")
            print(f"\nMatrice R (measurement noise):")
            print(f"  USBL range: {self.R_usbl[0,0]:.4f} m²")
            print(f"  Depth:      {self.R_depth[0,0]:.4f} m²")
            print(f"\nACC integration weight: {self.acc_weight}")
        
    def predict(self, acc_body, gyro, roll, pitch, yaw):
        """
        Prediction step - integra IMU
        
        Per constant velocity model:
        x_k+1 = x_k + vx_k * dt
        vx_k+1 = vx_k + ax * dt (con poca fiducia in ax)
        
        Args:
            acc_body: accelerazione in body frame [ax, ay, az] (m/s²)
            gyro: velocità angolare [gx, gy, gz] (rad/s) 
            roll, pitch, yaw: orientamento (rad)
        """
        # Rotazione da body frame a world frame
        R_body_to_world = self._rotation_matrix(roll, pitch, yaw)
        acc_world = R_body_to_world @ acc_body
        
        # Modello constant velocity con piccola correzione da accelerazione
        # x_k+1 = x_k + v_k * dt + 0.5 * a_k * dt^2 (con peso ridotto)
        # v_k+1 = v_k + a_k * dt (con peso ridotto)
        
        # State transition
        x_pred = self.x.copy()
        x_pred[0] = self.x[0] + self.x[3] * self.dt  # x
        x_pred[1] = self.x[1] + self.x[4] * self.dt  # y
        x_pred[2] = self.x[2] + self.x[5] * self.dt  # z
        
        # Velocità: piccola correzione da accelerazione (peso configurabile)
        x_pred[3] = self.x[3] + acc_world[0] * self.dt * self.acc_weight  # vx
        x_pred[4] = self.x[4] + acc_world[1] * self.dt * self.acc_weight  # vy
        x_pred[5] = self.x[5] + acc_world[2] * self.dt * self.acc_weight  # vz
        
        # Jacobiano della transizione
        F = np.eye(6)
        F[0, 3] = self.dt  # dx/dvx
        F[1, 4] = self.dt  # dy/dvy
        F[2, 5] = self.dt  # dz/dvz
        
        # Covarianza prediction
        P_pred = F @ self.P @ F.T + self.Q
        
        self.x = x_pred
        self.P = P_pred
        
    def update_usbl(self, range_measured):
        """
        Update step - correzione con range USBL
        
        Misura: h(x) = sqrt((x-x_boa)² + (y-y_boa)² + (z-z_boa)²)
        Range calcolato dalla posizione della BOA
        
        Args:
            range_measured: distanza misurata dall'USBL (m)
        """
        # Calcola distanza dalla BOA
        dx = self.x[0] - self.boa_position[0]
        dy = self.x[1] - self.boa_position[1]
        dz = self.x[2] - self.boa_position[2]
        range_pred = np.sqrt(dx**2 + dy**2 + dz**2)
        
        # Evita divisione per zero
        if range_pred < 1e-6:
            range_pred = 1e-6
        
        # Jacobiano: H = [∂h/∂x, ∂h/∂y, ∂h/∂z, 0, 0, 0]
        H = np.zeros((1, 6))
        H[0, 0] = dx / range_pred  # ∂h/∂x
        H[0, 1] = dy / range_pred  # ∂h/∂y
        H[0, 2] = dz / range_pred  # ∂h/∂z
        
        # Innovation
        y = range_measured - range_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_usbl
        
        # Kalman gain
        K = self.P @ H.T / S[0, 0]
        
        # State update
        self.x = self.x + K.flatten() * y
        
        # Covariance update (Joseph form per stabilità numerica)
        I = np.eye(6)
        IKH = I - np.outer(K, H)
        self.P = IKH @ self.P @ IKH.T + np.outer(K, K) * self.R_usbl[0, 0]
        
    def update_depth(self, depth_measured):
        """
        Update step - correzione con depth sensor
        
        Misura: h(x) = z
        
        Args:
            depth_measured: profondità misurata (m)
        """
        # Jacobiano: H = [0, 0, 1, 0, 0, 0]
        H = np.zeros((1, 6))
        H[0, 2] = 1.0
        
        # Innovation
        y = depth_measured - self.x[2]
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_depth
        
        # Kalman gain
        K = self.P @ H.T / S[0, 0]
        
        # State update
        self.x = self.x + K.flatten() * y
        
        # Covariance update
        I = np.eye(6)
        self.P = (I - np.outer(K, H)) @ self.P
        
    def _rotation_matrix(self, roll, pitch, yaw):
        """Matrice di rotazione da body frame a world frame (ZYX)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        return R
    
    def get_state(self):
        """Ritorna stato corrente"""
        return self.x.copy()
    
    def get_covariance(self):
        """Ritorna covarianza corrente"""
        return self.P.copy()

# ============================================================================
# FUSIONE SENSORI
# ============================================================================

def run_sensor_fusion(imu_data, usbl_data, depth_data, sensor_noise, config=None, initial_state=None, boa_position=None):
    """
    Esegue la fusione sensoriale con EKF
    
    Args:
        config: dict con parametri di configurazione (opzionale)
        initial_state: dict con stato iniziale (opzionale)
        boa_position: posizione BOA [x, y, z] (opzionale)
    
    Returns:
        trajectory: array Nx7 [timestamp, x, y, z, vx, vy, vz]
    """
    print("\n" + "="*70)
    print("ESECUZIONE FUSIONE SENSORIALE")
    print("="*70)
    
    # Calcola dt medio dall'IMU (sensore più veloce)
    dt_imu = np.diff(imu_data['timestamp_rel'].values)
    dt_mean = np.mean(dt_imu)
    print(f"\nFrequenza IMU: {1/dt_mean:.1f} Hz (dt = {dt_mean:.4f} s)")
    
    # Inizializza EKF con config, stato iniziale e posizione BOA
    ekf = EKF_AUV(sensor_noise, dt=dt_mean, config=config, initial_state=initial_state, boa_position=boa_position)
    
    # Prepara dati
    time_imu = imu_data['timestamp_rel'].values
    time_usbl = usbl_data['timestamp_rel'].values
    time_depth = depth_data['timestamp_rel'].values if len(depth_data) > 0 else np.array([])
    
    # Bias correction
    bias_acc = sensor_noise['bias_acc'] if APPLY_BIAS_CORRECTION else np.zeros(3)
    bias_gyr = sensor_noise['bias_gyr'] if APPLY_BIAS_CORRECTION else np.zeros(3)
    
    # Storage trajectory
    trajectory = []
    
    # Indici per sincronizzazione sensori
    idx_usbl = 0
    idx_depth = 0
    
    # Ultima posizione per statistiche
    last_usbl_update = -999.0
    
    print(f"\nInizio fusione...")
    print(f"  Campioni IMU:   {len(imu_data)}")
    print(f"  Fix USBL:       {len(usbl_data)}")
    print(f"  Misure depth:   {len(depth_data)}")
    
    # Loop principale - itera su campioni IMU
    for i in range(len(imu_data)):
        t = time_imu[i]
        
        # Lettura IMU (corretta per bias)
        acc_body = np.array([
            imu_data.iloc[i]['acc_x'] - bias_acc[0],
            imu_data.iloc[i]['acc_y'] - bias_acc[1],
            imu_data.iloc[i]['acc_z'] - bias_acc[2]
        ])
        
        gyro = np.array([
            imu_data.iloc[i]['gyr_x'] - bias_gyr[0],
            imu_data.iloc[i]['gyr_y'] - bias_gyr[1],
            imu_data.iloc[i]['gyr_z'] - bias_gyr[2]
        ])
        
        roll = imu_data.iloc[i]['roll']
        pitch = imu_data.iloc[i]['pitch']
        yaw = imu_data.iloc[i]['yaw']
        
        # Prediction step
        ekf.predict(acc_body, gyro, roll, pitch, yaw)
        
        # Update USBL (se disponibile)
        if idx_usbl < len(usbl_data) and abs(t - time_usbl[idx_usbl]) < dt_mean:
            range_meas = usbl_data.iloc[idx_usbl]['range']
            ekf.update_usbl(range_meas)
            last_usbl_update = t
            idx_usbl += 1
            
            if VERBOSE and idx_usbl % 5 == 0:
                print(f"  t={t:6.2f}s: USBL update #{idx_usbl}, range={range_meas:.2f}m")
        
        # Update Depth (se disponibile)
        if len(time_depth) > 0 and idx_depth < len(depth_data):
            if abs(t - time_depth[idx_depth]) < dt_mean:
                if 'depth' in depth_data.columns:
                    depth_meas = depth_data.iloc[idx_depth]['depth']
                    ekf.update_depth(depth_meas)
                idx_depth += 1
        
        # Salva stato
        state = ekf.get_state()
        trajectory.append([t, state[0], state[1], state[2], 
                          state[3], state[4], state[5]])
    
    trajectory = np.array(trajectory)
    
    print(f"\nFusione completata!")
    print(f"  Traiettoria: {len(trajectory)} punti")
    print(f"  USBL update: {idx_usbl}/{len(usbl_data)}")
    
    return trajectory

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    print("="*70)
    print("EKF SENSOR FUSION - AUV NAVIGATION")
    print("="*70)
    
    # Carica configurazione
    config = None
    initial_state = None
    boa_position = None
    
    try:
        from ekf_config import (Q_POSITION, Q_VELOCITY, Q_VELOCITY_Z, 
                               R_USBL_FACTOR, R_DEPTH_MIN,
                               P0_POSITION_XY, P0_POSITION_Z, P0_VELOCITY,
                               ACC_INTEGRATION_WEIGHT,
                               INITIAL_POSITION, INITIAL_VELOCITY,
                               BOA_POSITION)
        config = {
            'Q_POSITION': Q_POSITION,
            'Q_VELOCITY': Q_VELOCITY,
            'Q_VELOCITY_Z': Q_VELOCITY_Z,
            'R_USBL_FACTOR': R_USBL_FACTOR,
            'R_DEPTH_MIN': R_DEPTH_MIN,
            'P0_POSITION_XY': P0_POSITION_XY,
            'P0_POSITION_Z': P0_POSITION_Z,
            'P0_VELOCITY': P0_VELOCITY,
            'ACC_INTEGRATION_WEIGHT': ACC_INTEGRATION_WEIGHT
        }
        initial_state = {
            'position': INITIAL_POSITION,
            'velocity': INITIAL_VELOCITY
        }
        boa_position = BOA_POSITION
        print("\n✓ Configurazione caricata da ekf_config.py")
    except ImportError:
        print("\n⚠ ekf_config.py non trovato, usando configurazione default")
    
    # Carica calibrazione IMU statica (se disponibile)
    imu_calibration = None
    try:
        import json
        with open('imu_calibration.json', 'r') as f:
            imu_calibration = json.load(f)
        print("✓ Calibrazione IMU statica caricata da imu_calibration.json")
    except FileNotFoundError:
        print("⚠ imu_calibration.json non trovato, userò stima approssimativa")
    
    # Carica dati
    print("\nCaricamento dati sensori...")
    imu_file = find_latest_csv(DATA_DIR, "imu")
    usbl_file = find_latest_csv(DATA_DIR, "usbl")
    depth_file = find_latest_csv(DATA_DIR, "depth")
    
    imu_data = pd.read_csv(imu_file)
    usbl_data = pd.read_csv(usbl_file)
    depth_data = pd.read_csv(depth_file)
    
    # Analisi rumore sensori (passa calibrazione IMU se disponibile)
    sensor_noise = analyze_sensor_noise(imu_data, usbl_data, depth_data, imu_calibration)
    
    # Esegui fusione con config, stato iniziale e posizione BOA
    trajectory = run_sensor_fusion(imu_data, usbl_data, depth_data, sensor_noise, config, initial_state, boa_position)
    
    # ========================================================================
    # VISUALIZZAZIONE
    # ========================================================================
    
    print("\n" + "="*70)
    print("VISUALIZZAZIONE")
    print("="*70)
    
    fig = plt.figure(figsize=(18, 7))
    
    # --- SUBPLOT 1: Mappa 2D ---
    ax1 = fig.add_subplot(131, aspect='equal')
    
    # Piscina
    pool = Circle((0, 0), POOL_RADIUS, fill=False, edgecolor='blue', 
                  linewidth=2, linestyle='--', label='Piscina (R=8m)')
    ax1.add_patch(pool)
    
    # BOA
    boa = Circle(BOA_COORDINATES, 0.3, color='red', alpha=0.7, label='BOA')
    ax1.add_patch(boa)
    
    # Marker nord
    ax1.plot(NORTH_MARKER_COORDINATES[0], NORTH_MARKER_COORDINATES[1], 
             's', color='gray', markersize=12, label='Nord', zorder=5)
    
    # Traccia EKF nuovo
    x_new = trajectory[:, 1]
    y_new = trajectory[:, 2]
    time_new = trajectory[:, 0]
    
    points = np.array([x_new, y_new]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(time_new.min(), time_new.max())
    lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=2.5, 
                       label='EKF Nuovo', zorder=4)
    lc.set_array(time_new[:-1])
    ax1.add_collection(lc)
    
    # Marker start/end
    ax1.plot(x_new[0], y_new[0], 'go', markersize=12, label='Start', zorder=6)
    ax1.plot(x_new[-1], y_new[-1], 'rs', markersize=12, label='End', zorder=6)
    
    # Colorbar
    cbar = plt.colorbar(lc, ax=ax1, label='Tempo [s]')
    
    # Assi
    ax1.set_xlabel('X [m]', fontsize=12)
    ax1.set_ylabel('Y [m]', fontsize=12)
    ax1.set_title('Traiettoria EKF (Constant Velocity)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=9)
    margin = 1.5
    ax1.set_xlim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax1.set_ylim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
    ax1.axhline(y=0, color='k', linewidth=0.5, alpha=0.3)
    ax1.axvline(x=0, color='k', linewidth=0.5, alpha=0.3)
    
    # --- SUBPLOT 2: Range USBL vs EKF ---
    ax2 = fig.add_subplot(132)
    
    time_usbl = usbl_data['timestamp_rel'].values
    usbl_range = usbl_data['range'].values
    
    # Range EKF (distanza dalla BOA)
    if boa_position is not None:
        dx = x_new - boa_position[0]
        dy = y_new - boa_position[1]
        dz = trajectory[:, 3] - boa_position[2]
        range_ekf = np.sqrt(dx**2 + dy**2 + dz**2)
    else:
        # Fallback: distanza dall'origine
        range_ekf = np.sqrt(x_new**2 + y_new**2 + trajectory[:, 3]**2)
    
    ax2.plot(time_usbl, usbl_range, 'o-', color='steelblue', 
             markersize=8, linewidth=2, label='USBL Measured', zorder=3)
    ax2.plot(time_new, range_ekf, '-', color='red', alpha=0.7,
             linewidth=1.5, label='EKF Estimate', zorder=2)
    
    ax2.set_xlabel('Tempo [s]', fontsize=12)
    ax2.set_ylabel('Range [m]', fontsize=12)
    ax2.set_title('Confronto Range USBL', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # Statistiche
    mean_range = np.mean(usbl_range)
    ax2.axhline(y=mean_range, color='blue', linestyle='--', 
                alpha=0.3, linewidth=1)
    
    # --- SUBPLOT 3: Velocità ---
    ax3 = fig.add_subplot(133)
    
    vx = trajectory[:, 4]
    vy = trajectory[:, 5]
    speed_2d = np.sqrt(vx**2 + vy**2)
    
    ax3.plot(time_new, vx, label='VX', alpha=0.7, linewidth=1.5)
    ax3.plot(time_new, vy, label='VY', alpha=0.7, linewidth=1.5)
    ax3.plot(time_new, speed_2d, 'k-', label='Speed 2D', linewidth=2)
    ax3.axhline(y=0, color='k', linestyle='--', alpha=0.3, linewidth=0.5)
    
    ax3.set_xlabel('Tempo [s]', fontsize=12)
    ax3.set_ylabel('Velocità [m/s]', fontsize=12)
    ax3.set_title('Velocità Stimata EKF', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    plt.tight_layout()
    
    # Statistiche finali
    print(f"\n=== STATISTICHE FINALI ===")
    print(f"\nTraiettoria EKF:")
    print(f"  Durata: {time_new[-1] - time_new[0]:.2f} s")
    
    # Distanza percorsa
    dist = np.sum(np.sqrt(np.diff(x_new)**2 + np.diff(y_new)**2))
    print(f"  Distanza percorsa: {dist:.2f} m")
    print(f"  Posizione iniziale: ({x_new[0]:.3f}, {y_new[0]:.3f}, {trajectory[0,3]:.3f}) m")
    print(f"  Posizione finale:   ({x_new[-1]:.3f}, {y_new[-1]:.3f}, {trajectory[-1,3]:.3f}) m")
    
    # Velocità
    print(f"\nVelocità:")
    print(f"  Media 2D: {np.mean(speed_2d):.3f} m/s")
    print(f"  Max 2D:   {np.max(speed_2d):.3f} m/s")
    
    # Errore range USBL
    range_errors = []
    for i, t_usbl in enumerate(time_usbl):
        idx = np.argmin(np.abs(time_new - t_usbl))
        
        # Calcola range dalla BOA
        if boa_position is not None:
            dx = x_new[idx] - boa_position[0]
            dy = y_new[idx] - boa_position[1]
            dz = trajectory[idx,3] - boa_position[2]
            range_ekf_at_usbl = np.sqrt(dx**2 + dy**2 + dz**2)
        else:
            # Fallback: distanza dall'origine
            range_ekf_at_usbl = np.sqrt(x_new[idx]**2 + y_new[idx]**2 + trajectory[idx,3]**2)
        
        error = abs(range_ekf_at_usbl - usbl_range[i])
        range_errors.append(error)
    
    range_errors = np.array(range_errors)
    print(f"\nErrore range USBL:")
    print(f"  Media: {np.mean(range_errors):.3f} m")
    print(f"  RMS:   {np.sqrt(np.mean(range_errors**2)):.3f} m")
    print(f"  Max:   {np.max(range_errors):.3f} m")
    
    plt.savefig('ekf_fusion_result.png', dpi=150, bbox_inches='tight')
    print(f"\nGrafico salvato: ekf_fusion_result.png")
    
    plt.show()