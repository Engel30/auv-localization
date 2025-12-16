#!/usr/bin/env python3
"""
Replay interattivo dei log EKF e sensori raw
Visualizza dati progressivamente con controlli play/pause/restart/velocità
"""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d import Axes3D
import time

# =============================================================================
#  CONFIGURAZIONE
# =============================================================================
LOG_DIRECTORY = "sensor_logs"

# =============================================================================
#  CARICAMENTO DATI
# =============================================================================
def load_csv_data(filepath):
    """Carica dati da file CSV."""
    if not os.path.exists(filepath):
        print("File non trovato: {}".format(filepath))
        return None
    
    data = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            try:
                data.append([float(x) if x != '' else np.nan for x in row])
            except ValueError:
                continue
    
    if len(data) == 0:
        print("Nessun dato in: {}".format(filepath))
        return None
    
    return np.array(data), header

def find_latest_logs(log_dir):
    """Trova i file di log più recenti."""
    if not os.path.exists(log_dir):
        print("Directory log non trovata: {}".format(log_dir))
        return None
    
    files = {
        'imu': None,
        'depth': None,
        'usbl': None,
        'ekf': None
    }
    
    for f in os.listdir(log_dir):
        if f.endswith('.csv'):
            for key in files.keys():
                if f.startswith(key):
                    filepath = os.path.join(log_dir, f)
                    if files[key] is None or os.path.getmtime(filepath) > os.path.getmtime(files[key]):
                        files[key] = filepath
    
    return files

# =============================================================================
#  REPLAY SIMULATOR CLASS
# =============================================================================
class ReplaySimulator:
    def __init__(self, ekf_data, imu_data, depth_data, usbl_data):
        self.ekf_data = ekf_data
        self.imu_data = imu_data
        self.depth_data = depth_data
        self.usbl_data = usbl_data
        
        # Trova tempo minimo e massimo
        self.t_min = 0.0
        self.t_max = 0.0
        
        if ekf_data is not None:
            self.t_max = max(self.t_max, ekf_data[-1, 1])
        if imu_data is not None:
            self.t_max = max(self.t_max, imu_data[-1, 1])
        if depth_data is not None:
            self.t_max = max(self.t_max, depth_data[-1, 1])
        if usbl_data is not None:
            self.t_max = max(self.t_max, usbl_data[-1, 1])
        
        # Stato simulazione
        self.current_time = 0.0
        self.is_playing = False
        self.speed = 1.0
        self.last_update_time = None
        
        # Timer
        self.timer = None
        
        # Setup plot
        self.setup_plots()
        self.setup_controls()
        
    def setup_plots(self):
        """Inizializza le figure e i plot."""
        
        # =====================================================================
        # FIGURA 1: EKF
        # =====================================================================
        self.fig_ekf = plt.figure(figsize=(16, 12))
        self.fig_ekf.suptitle('Risultati Filtro di Kalman (EKF) - REPLAY', 
                              fontsize=16, fontweight='bold')
        
        # 3D Trajectory
        self.ax_ekf_3d = self.fig_ekf.add_subplot(3, 3, 1, projection='3d')
        self.line_ekf_3d, = self.ax_ekf_3d.plot([], [], [], 'b-', linewidth=1.5)
        self.point_ekf_current = self.ax_ekf_3d.scatter([], [], [], c='r', s=100, marker='o')
        self.ax_ekf_3d.set_xlabel('X [m]')
        self.ax_ekf_3d.set_ylabel('Y [m]')
        self.ax_ekf_3d.set_zlabel('Z [m]')
        self.ax_ekf_3d.set_title('Traiettoria 3D')
        self.ax_ekf_3d.grid(True)
        
        # XY View
        self.ax_ekf_xy = self.fig_ekf.add_subplot(3, 3, 2)
        self.line_ekf_xy, = self.ax_ekf_xy.plot([], [], 'b-', linewidth=1.5)
        self.point_ekf_xy = self.ax_ekf_xy.scatter([], [], c='r', s=100, marker='o')
        self.ax_ekf_xy.set_xlabel('X [m]')
        self.ax_ekf_xy.set_ylabel('Y [m]')
        self.ax_ekf_xy.set_title('Vista XY')
        self.ax_ekf_xy.grid(True)
        self.ax_ekf_xy.axis('equal')
        
        # Depth
        self.ax_ekf_depth = self.fig_ekf.add_subplot(3, 3, 3)
        self.line_ekf_depth, = self.ax_ekf_depth.plot([], [], 'b-', linewidth=1.5)
        self.ax_ekf_depth.set_xlabel('Tempo [s]')
        self.ax_ekf_depth.set_ylabel('Profondità [m]')
        self.ax_ekf_depth.set_title('Profondità')
        self.ax_ekf_depth.grid(True)
        self.ax_ekf_depth.invert_yaxis()
        
        # Position XYZ
        self.ax_ekf_pos = self.fig_ekf.add_subplot(3, 3, 4)
        self.line_ekf_pos_x, = self.ax_ekf_pos.plot([], [], 'r-', label='X', linewidth=1.5)
        self.line_ekf_pos_y, = self.ax_ekf_pos.plot([], [], 'g-', label='Y', linewidth=1.5)
        self.line_ekf_pos_z, = self.ax_ekf_pos.plot([], [], 'b-', label='Z', linewidth=1.5)
        self.ax_ekf_pos.set_xlabel('Tempo [s]')
        self.ax_ekf_pos.set_ylabel('Posizione [m]')
        self.ax_ekf_pos.set_title('Posizione XYZ')
        self.ax_ekf_pos.legend()
        self.ax_ekf_pos.grid(True)
        
        # Velocity XYZ
        self.ax_ekf_vel = self.fig_ekf.add_subplot(3, 3, 5)
        self.line_ekf_vel_x, = self.ax_ekf_vel.plot([], [], 'r-', label='Vx', linewidth=1.5)
        self.line_ekf_vel_y, = self.ax_ekf_vel.plot([], [], 'g-', label='Vy', linewidth=1.5)
        self.line_ekf_vel_z, = self.ax_ekf_vel.plot([], [], 'b-', label='Vz', linewidth=1.5)
        self.ax_ekf_vel.set_xlabel('Tempo [s]')
        self.ax_ekf_vel.set_ylabel('Velocità [m/s]')
        self.ax_ekf_vel.set_title('Velocità XYZ')
        self.ax_ekf_vel.legend()
        self.ax_ekf_vel.grid(True)
        
        # Acceleration XYZ
        self.ax_ekf_acc = self.fig_ekf.add_subplot(3, 3, 6)
        self.line_ekf_acc_x, = self.ax_ekf_acc.plot([], [], 'r-', label='Ax', linewidth=1.5)
        self.line_ekf_acc_y, = self.ax_ekf_acc.plot([], [], 'g-', label='Ay', linewidth=1.5)
        self.line_ekf_acc_z, = self.ax_ekf_acc.plot([], [], 'b-', label='Az', linewidth=1.5)
        self.ax_ekf_acc.set_xlabel('Tempo [s]')
        self.ax_ekf_acc.set_ylabel('Accelerazione [m/s²]')
        self.ax_ekf_acc.set_title('Accelerazione XYZ')
        self.ax_ekf_acc.legend()
        self.ax_ekf_acc.grid(True)
        
        # Roll
        self.ax_ekf_roll = self.fig_ekf.add_subplot(3, 3, 7)
        self.line_ekf_roll, = self.ax_ekf_roll.plot([], [], 'r-', linewidth=1.5)
        self.ax_ekf_roll.set_xlabel('Tempo [s]')
        self.ax_ekf_roll.set_ylabel('Roll [deg]')
        self.ax_ekf_roll.set_title('Roll')
        self.ax_ekf_roll.grid(True)
        
        # Pitch
        self.ax_ekf_pitch = self.fig_ekf.add_subplot(3, 3, 8)
        self.line_ekf_pitch, = self.ax_ekf_pitch.plot([], [], 'g-', linewidth=1.5)
        self.ax_ekf_pitch.set_xlabel('Tempo [s]')
        self.ax_ekf_pitch.set_ylabel('Pitch [deg]')
        self.ax_ekf_pitch.set_title('Pitch')
        self.ax_ekf_pitch.grid(True)
        
        # Yaw
        self.ax_ekf_yaw = self.fig_ekf.add_subplot(3, 3, 9)
        self.line_ekf_yaw, = self.ax_ekf_yaw.plot([], [], 'b-', linewidth=1.5)
        self.ax_ekf_yaw.set_xlabel('Tempo [s]')
        self.ax_ekf_yaw.set_ylabel('Yaw [deg]')
        self.ax_ekf_yaw.set_title('Yaw')
        self.ax_ekf_yaw.grid(True)
        
        # =====================================================================
        # FIGURA 2: RAW SENSORS
        # =====================================================================
        self.fig_raw = plt.figure(figsize=(16, 10))
        self.fig_raw.suptitle('Misure Raw Sensori - REPLAY', 
                              fontsize=16, fontweight='bold')
        
        # IMU - Accelerometer
        self.ax_imu_acc = self.fig_raw.add_subplot(3, 3, 1)
        self.line_imu_acc_x, = self.ax_imu_acc.plot([], [], 'r-', label='Ax', linewidth=1)
        self.line_imu_acc_y, = self.ax_imu_acc.plot([], [], 'g-', label='Ay', linewidth=1)
        self.line_imu_acc_z, = self.ax_imu_acc.plot([], [], 'b-', label='Az', linewidth=1)
        self.ax_imu_acc.set_xlabel('Tempo [s]')
        self.ax_imu_acc.set_ylabel('Accelerazione [m/s²]')
        self.ax_imu_acc.set_title('IMU - Accelerometro')
        self.ax_imu_acc.legend()
        self.ax_imu_acc.grid(True)
        
        # IMU - Gyroscope
        self.ax_imu_gyr = self.fig_raw.add_subplot(3, 3, 2)
        self.line_imu_gyr_x, = self.ax_imu_gyr.plot([], [], 'r-', label='Gx', linewidth=1)
        self.line_imu_gyr_y, = self.ax_imu_gyr.plot([], [], 'g-', label='Gy', linewidth=1)
        self.line_imu_gyr_z, = self.ax_imu_gyr.plot([], [], 'b-', label='Gz', linewidth=1)
        self.ax_imu_gyr.set_xlabel('Tempo [s]')
        self.ax_imu_gyr.set_ylabel('Velocità angolare [rad/s]')
        self.ax_imu_gyr.set_title('IMU - Giroscopio')
        self.ax_imu_gyr.legend()
        self.ax_imu_gyr.grid(True)
        
        # IMU - Roll
        self.ax_imu_roll = self.fig_raw.add_subplot(3, 3, 3)
        self.line_imu_roll, = self.ax_imu_roll.plot([], [], 'r-', linewidth=1.5)
        self.ax_imu_roll.set_xlabel('Tempo [s]')
        self.ax_imu_roll.set_ylabel('Roll [deg]')
        self.ax_imu_roll.set_title('IMU - Roll')
        self.ax_imu_roll.grid(True)
        
        # IMU - Pitch
        self.ax_imu_pitch = self.fig_raw.add_subplot(3, 3, 4)
        self.line_imu_pitch, = self.ax_imu_pitch.plot([], [], 'g-', linewidth=1.5)
        self.ax_imu_pitch.set_xlabel('Tempo [s]')
        self.ax_imu_pitch.set_ylabel('Pitch [deg]')
        self.ax_imu_pitch.set_title('IMU - Pitch')
        self.ax_imu_pitch.grid(True)
        
        # IMU - Yaw
        self.ax_imu_yaw = self.fig_raw.add_subplot(3, 3, 5)
        self.line_imu_yaw, = self.ax_imu_yaw.plot([], [], 'b-', linewidth=1.5)
        self.ax_imu_yaw.set_xlabel('Tempo [s]')
        self.ax_imu_yaw.set_ylabel('Yaw [deg]')
        self.ax_imu_yaw.set_title('IMU - Yaw')
        self.ax_imu_yaw.grid(True)
        
        # Depth
        self.ax_depth = self.fig_raw.add_subplot(3, 3, 6)
        self.line_depth, = self.ax_depth.plot([], [], 'b-', linewidth=1.5)
        self.ax_depth.set_xlabel('Tempo [s]')
        self.ax_depth.set_ylabel('Profondità [m]')
        self.ax_depth.set_title('Depth Sensor')
        self.ax_depth.grid(True)
        
        # USBL - Range
        self.ax_usbl_range = self.fig_raw.add_subplot(3, 3, 7)
        self.line_usbl_range, = self.ax_usbl_range.plot([], [], 'ko-', 
                                                         linewidth=1.5, markersize=6, 
                                                         markerfacecolor='orange')
        self.ax_usbl_range.set_xlabel('Tempo [s]')
        self.ax_usbl_range.set_ylabel('Range [m]')
        self.ax_usbl_range.set_title('USBL - Range')
        self.ax_usbl_range.grid(True)
        
        # USBL - Integrity
        self.ax_usbl_int = self.fig_raw.add_subplot(3, 3, 8)
        self.line_usbl_int, = self.ax_usbl_int.plot([], [], 'go-', 
                                                     linewidth=1.5, markersize=6)
        self.ax_usbl_int.set_xlabel('Tempo [s]')
        self.ax_usbl_int.set_ylabel('Integrity [%]')
        self.ax_usbl_int.set_title('USBL - Integrity')
        self.ax_usbl_int.grid(True)
        self.ax_usbl_int.set_ylim([0, 105])
        
        # USBL - RSSI
        self.ax_usbl_rssi = self.fig_raw.add_subplot(3, 3, 9)
        self.line_usbl_rssi, = self.ax_usbl_rssi.plot([], [], 'ro-', 
                                                       linewidth=1.5, markersize=6)
        self.ax_usbl_rssi.set_xlabel('Tempo [s]')
        self.ax_usbl_rssi.set_ylabel('RSSI [dB]')
        self.ax_usbl_rssi.set_title('USBL - RSSI')
        self.ax_usbl_rssi.grid(True)
        
        plt.tight_layout()
        
    def setup_controls(self):
        """Inizializza i controlli interattivi."""
        
        # Aggiungi spazio per i controlli nella figura EKF
        self.fig_ekf.subplots_adjust(bottom=0.15)
        
        # Play/Pause Button
        ax_play = plt.axes([0.15, 0.05, 0.1, 0.04])
        self.btn_play = Button(ax_play, 'Play')
        self.btn_play.on_clicked(self.toggle_play)
        
        # Restart Button
        ax_restart = plt.axes([0.27, 0.05, 0.1, 0.04])
        self.btn_restart = Button(ax_restart, 'Restart')
        self.btn_restart.on_clicked(self.restart)
        
        # Speed Slider
        ax_speed = plt.axes([0.45, 0.05, 0.25, 0.03])
        self.slider_speed = Slider(ax_speed, 'Speed', 0.1, 10.0, 
                                    valinit=1.0, valstep=0.1)
        self.slider_speed.on_changed(self.update_speed)
        
        # Progress Slider
        ax_progress = plt.axes([0.15, 0.02, 0.7, 0.02])
        self.slider_progress = Slider(ax_progress, 'Time [s]', 
                                       self.t_min, self.t_max, 
                                       valinit=0.0)
        self.slider_progress.on_changed(self.seek)
        
        # Label per info
        self.info_text = self.fig_ekf.text(0.75, 0.07, '', 
                                           fontsize=10, family='monospace')
        
    def toggle_play(self, event):
        """Play/Pause toggle."""
        if self.is_playing:
            self.pause()
        else:
            self.play()
    
    def play(self):
        """Avvia la riproduzione."""
        if not self.is_playing:
            self.is_playing = True
            self.btn_play.label.set_text('Pause')
            self.last_update_time = time.time()
            if self.timer is None:
                self.timer = self.fig_ekf.canvas.new_timer(interval=50)  # 20 Hz
                self.timer.add_callback(self.update)
            self.timer.start()
    
    def pause(self):
        """Mette in pausa la riproduzione."""
        if self.is_playing:
            self.is_playing = False
            self.btn_play.label.set_text('Play')
            if self.timer:
                self.timer.stop()
    
    def restart(self, event):
        """Riavvia la simulazione dall'inizio."""
        was_playing = self.is_playing
        self.pause()
        self.current_time = 0.0
        self.slider_progress.set_val(0.0)
        self.update_plots()
        if was_playing:
            self.play()
    
    def update_speed(self, val):
        """Aggiorna la velocità di riproduzione."""
        self.speed = val
    
    def seek(self, val):
        """Salta a un tempo specifico."""
        was_playing = self.is_playing
        self.pause()
        self.current_time = val
        self.update_plots()
        if was_playing:
            self.play()
    
    def update(self):
        """Aggiorna la simulazione (chiamato dal timer)."""
        if not self.is_playing:
            return
        
        current_real_time = time.time()
        if self.last_update_time is not None:
            dt_real = current_real_time - self.last_update_time
            dt_sim = dt_real * self.speed
            self.current_time += dt_sim
        
        self.last_update_time = current_real_time
        
        # Fine della simulazione
        if self.current_time >= self.t_max:
            self.pause()
            self.current_time = self.t_max
        
        # Aggiorna slider (senza trigger callback)
        self.slider_progress.eventson = False
        self.slider_progress.set_val(self.current_time)
        self.slider_progress.eventson = True
        
        self.update_plots()
    
    def update_plots(self):
        """Aggiorna tutti i plot in base al tempo corrente."""
        
        # =====================================================================
        # AGGIORNA EKF
        # =====================================================================
        if self.ekf_data is not None:
            mask_ekf = self.ekf_data[:, 1] <= self.current_time
            if np.any(mask_ekf):
                data_ekf = self.ekf_data[mask_ekf]
                
                t_rel = data_ekf[:, 1]
                pos = data_ekf[:, 2:5]
                vel = data_ekf[:, 5:8]
                acc = data_ekf[:, 8:11]
                rpy = np.degrees(data_ekf[:, 11:14])
                
                # Traiettoria 3D
                self.line_ekf_3d.set_data(pos[:, 0], pos[:, 1])
                self.line_ekf_3d.set_3d_properties(pos[:, 2])
                self.point_ekf_current._offsets3d = ([pos[-1, 0]], [pos[-1, 1]], [pos[-1, 2]])
                
                # Auto-scale 3D
                if len(pos) > 1:
                    margin = 0.5
                    self.ax_ekf_3d.set_xlim(pos[:, 0].min() - margin, pos[:, 0].max() + margin)
                    self.ax_ekf_3d.set_ylim(pos[:, 1].min() - margin, pos[:, 1].max() + margin)
                    self.ax_ekf_3d.set_zlim(pos[:, 2].min() - margin, pos[:, 2].max() + margin)
                
                # XY View
                self.line_ekf_xy.set_data(pos[:, 0], pos[:, 1])
                self.point_ekf_xy.set_offsets([[pos[-1, 0], pos[-1, 1]]])
                
                if len(pos) > 1:
                    self.ax_ekf_xy.set_xlim(pos[:, 0].min() - margin, pos[:, 0].max() + margin)
                    self.ax_ekf_xy.set_ylim(pos[:, 1].min() - margin, pos[:, 1].max() + margin)
                
                # Depth
                self.line_ekf_depth.set_data(t_rel, -pos[:, 2])
                self.ax_ekf_depth.set_xlim(0, self.t_max)
                if len(pos) > 1:
                    d_margin = 0.2
                    self.ax_ekf_depth.set_ylim(-pos[:, 2].min() - d_margin, -pos[:, 2].max() + d_margin)
                
                # Position XYZ
                self.line_ekf_pos_x.set_data(t_rel, pos[:, 0])
                self.line_ekf_pos_y.set_data(t_rel, pos[:, 1])
                self.line_ekf_pos_z.set_data(t_rel, pos[:, 2])
                self.ax_ekf_pos.set_xlim(0, self.t_max)
                self.ax_ekf_pos.relim()
                self.ax_ekf_pos.autoscale_view(scalex=False)
                
                # Velocity XYZ
                self.line_ekf_vel_x.set_data(t_rel, vel[:, 0])
                self.line_ekf_vel_y.set_data(t_rel, vel[:, 1])
                self.line_ekf_vel_z.set_data(t_rel, vel[:, 2])
                self.ax_ekf_vel.set_xlim(0, self.t_max)
                self.ax_ekf_vel.relim()
                self.ax_ekf_vel.autoscale_view(scalex=False)
                
                # Acceleration XYZ
                self.line_ekf_acc_x.set_data(t_rel, acc[:, 0])
                self.line_ekf_acc_y.set_data(t_rel, acc[:, 1])
                self.line_ekf_acc_z.set_data(t_rel, acc[:, 2])
                self.ax_ekf_acc.set_xlim(0, self.t_max)
                self.ax_ekf_acc.relim()
                self.ax_ekf_acc.autoscale_view(scalex=False)
                
                # Roll, Pitch, Yaw
                self.line_ekf_roll.set_data(t_rel, rpy[:, 0])
                self.ax_ekf_roll.set_xlim(0, self.t_max)
                self.ax_ekf_roll.relim()
                self.ax_ekf_roll.autoscale_view(scalex=False)
                
                self.line_ekf_pitch.set_data(t_rel, rpy[:, 1])
                self.ax_ekf_pitch.set_xlim(0, self.t_max)
                self.ax_ekf_pitch.relim()
                self.ax_ekf_pitch.autoscale_view(scalex=False)
                
                self.line_ekf_yaw.set_data(t_rel, rpy[:, 2])
                self.ax_ekf_yaw.set_xlim(0, self.t_max)
                self.ax_ekf_yaw.relim()
                self.ax_ekf_yaw.autoscale_view(scalex=False)
        
        # =====================================================================
        # AGGIORNA IMU
        # =====================================================================
        if self.imu_data is not None:
            mask_imu = self.imu_data[:, 1] <= self.current_time
            if np.any(mask_imu):
                data_imu = self.imu_data[mask_imu]
                
                t_imu = data_imu[:, 1]
                rpy_imu = np.degrees(data_imu[:, 2:5])
                acc_imu = data_imu[:, 5:8]
                gyr_imu = data_imu[:, 8:11]
                
                # Accelerometer
                self.line_imu_acc_x.set_data(t_imu, acc_imu[:, 0])
                self.line_imu_acc_y.set_data(t_imu, acc_imu[:, 1])
                self.line_imu_acc_z.set_data(t_imu, acc_imu[:, 2])
                self.ax_imu_acc.set_xlim(0, self.t_max)
                self.ax_imu_acc.relim()
                self.ax_imu_acc.autoscale_view(scalex=False)
                
                # Gyroscope
                self.line_imu_gyr_x.set_data(t_imu, gyr_imu[:, 0])
                self.line_imu_gyr_y.set_data(t_imu, gyr_imu[:, 1])
                self.line_imu_gyr_z.set_data(t_imu, gyr_imu[:, 2])
                self.ax_imu_gyr.set_xlim(0, self.t_max)
                self.ax_imu_gyr.relim()
                self.ax_imu_gyr.autoscale_view(scalex=False)
                
                # Roll, Pitch, Yaw
                self.line_imu_roll.set_data(t_imu, rpy_imu[:, 0])
                self.ax_imu_roll.set_xlim(0, self.t_max)
                self.ax_imu_roll.relim()
                self.ax_imu_roll.autoscale_view(scalex=False)
                
                self.line_imu_pitch.set_data(t_imu, rpy_imu[:, 1])
                self.ax_imu_pitch.set_xlim(0, self.t_max)
                self.ax_imu_pitch.relim()
                self.ax_imu_pitch.autoscale_view(scalex=False)
                
                self.line_imu_yaw.set_data(t_imu, rpy_imu[:, 2])
                self.ax_imu_yaw.set_xlim(0, self.t_max)
                self.ax_imu_yaw.relim()
                self.ax_imu_yaw.autoscale_view(scalex=False)
        
        # =====================================================================
        # AGGIORNA DEPTH
        # =====================================================================
        if self.depth_data is not None:
            mask_depth = self.depth_data[:, 1] <= self.current_time
            if np.any(mask_depth):
                data_depth = self.depth_data[mask_depth]
                
                t_depth = data_depth[:, 1]
                depth_vals = data_depth[:, 2]
                
                self.line_depth.set_data(t_depth, depth_vals)
                self.ax_depth.set_xlim(0, self.t_max)
                self.ax_depth.relim()
                self.ax_depth.autoscale_view(scalex=False)
        
        # =====================================================================
        # AGGIORNA USBL
        # =====================================================================
        if self.usbl_data is not None:
            mask_usbl = self.usbl_data[:, 1] <= self.current_time
            if np.any(mask_usbl):
                data_usbl = self.usbl_data[mask_usbl]
                
                t_usbl = data_usbl[:, 1]
                range_usbl = data_usbl[:, 2]
                integrity_usbl = data_usbl[:, 3]
                rssi_usbl = data_usbl[:, 4]
                
                # Filtra valori validi
                valid_idx = range_usbl > 0
                
                if np.any(valid_idx):
                    self.line_usbl_range.set_data(t_usbl[valid_idx], range_usbl[valid_idx])
                    self.ax_usbl_range.set_xlim(0, self.t_max)
                    self.ax_usbl_range.relim()
                    self.ax_usbl_range.autoscale_view(scalex=False)
                    
                    self.line_usbl_int.set_data(t_usbl[valid_idx], integrity_usbl[valid_idx])
                    self.ax_usbl_int.set_xlim(0, self.t_max)
                    
                    self.line_usbl_rssi.set_data(t_usbl[valid_idx], rssi_usbl[valid_idx])
                    self.ax_usbl_rssi.set_xlim(0, self.t_max)
                    self.ax_usbl_rssi.relim()
                    self.ax_usbl_rssi.autoscale_view(scalex=False)
        
        # Aggiorna info text
        self.info_text.set_text('Time: {:.2f}s / {:.2f}s\nSpeed: {:.1f}x'.format(
            self.current_time, self.t_max, self.speed))
        
        # Ridisegna
        self.fig_ekf.canvas.draw_idle()
        self.fig_raw.canvas.draw_idle()
    
    def run(self):
        """Avvia l'interfaccia."""
        plt.show()

# =============================================================================
#  MAIN
# =============================================================================
def main():
    print("=" * 70)
    print("REPLAY INTERATTIVO DEI LOG")
    print("=" * 70)
    
    # Trova file più recenti
    files = find_latest_logs(LOG_DIRECTORY)
    if files is None:
        return
    
    print("\nFile trovati:")
    for key, path in files.items():
        if path:
            print("  {}: {}".format(key.upper(), os.path.basename(path)))
        else:
            print("  {}: NON TROVATO".format(key.upper()))
    
    # Carica dati
    print("\nCaricamento dati...")
    
    ekf_result = load_csv_data(files['ekf']) if files['ekf'] else None
    imu_result = load_csv_data(files['imu']) if files['imu'] else None
    depth_result = load_csv_data(files['depth']) if files['depth'] else None
    usbl_result = load_csv_data(files['usbl']) if files['usbl'] else None
    
    ekf_data = ekf_result[0] if ekf_result else None
    imu_data = imu_result[0] if imu_result else None
    depth_data = depth_result[0] if depth_result else None
    usbl_data = usbl_result[0] if usbl_result else None
    
    if ekf_data is None:
        print("\nERRORE: Nessun dato EKF disponibile!")
        return
    
    # Statistiche
    print("\nStatistiche dati:")
    if ekf_data is not None:
        print("  EKF: {} campioni, durata {:.2f}s".format(
            len(ekf_data), ekf_data[-1, 1] - ekf_data[0, 1]))
    if imu_data is not None:
        print("  IMU: {} campioni, durata {:.2f}s".format(
            len(imu_data), imu_data[-1, 1] - imu_data[0, 1]))
    if depth_data is not None:
        print("  Depth: {} campioni, durata {:.2f}s".format(
            len(depth_data), depth_data[-1, 1] - depth_data[0, 1]))
    if usbl_data is not None:
        valid_usbl = np.sum(usbl_data[:, 2] > 0)
        print("  USBL: {} campioni totali, {} validi".format(
            len(usbl_data), valid_usbl))
    
    print("\n" + "=" * 70)
    print("CONTROLLI:")
    print("  Play/Pause: Avvia/ferma la riproduzione")
    print("  Restart: Riavvia dall'inizio")
    print("  Speed slider: Controlla velocità (0.1x - 10x)")
    print("  Time slider: Salta a un tempo specifico")
    print("=" * 70)
    
    # Crea e avvia simulator
    simulator = ReplaySimulator(ekf_data, imu_data, depth_data, usbl_data)
    simulator.run()

if __name__ == "__main__":
    main()
