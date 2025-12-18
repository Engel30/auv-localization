#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Visualizer - 3D/2D visualization with sensor data display
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Rectangle
import os
from glob import glob

class EKFVisualizer:
    def __init__(self, log_dir='sensor_logs'):
        # Parametri acquario (in cm)
        self.tank_x = 120.0
        self.tank_y = 40.0
        self.tank_z = 40.0
        
        # Keypoint "albero" (in cm)
        self.tree_pos = np.array([-278.0, 34.0, 0.0])
        
        # Posizione USBL transceiver (configurabile, in cm)
        self.usbl_pos = np.array([60.0, 20.0, -20.0])  # Esempio centrale acquario
        
        # Carica i dati
        self.load_data(log_dir)
        
        # Stato playback
        self.is_playing = False
        self.current_idx = 0
        self.playback_speed = 1.0
        
        # Setup GUI
        self.setup_figure()
        
    def load_data(self, log_dir):
        """Carica i CSV dei sensori"""
        # Trova i file più recenti
        ekf_files = sorted(glob(os.path.join(log_dir, 'ekf_*.csv')))
        imu_files = sorted(glob(os.path.join(log_dir, 'imu_*.csv')))
        depth_files = sorted(glob(os.path.join(log_dir, 'depth_*.csv')))
        usbl_files = sorted(glob(os.path.join(log_dir, 'usbl_*.csv')))
        
        if not ekf_files:
            raise ValueError("Nessun file EKF trovato in {}".format(log_dir))
        
        # Carica l'ultimo log set
        self.df_ekf = pd.read_csv(ekf_files[-1])
        self.df_imu = pd.read_csv(imu_files[-1]) if imu_files else pd.DataFrame()
        self.df_depth = pd.read_csv(depth_files[-1]) if depth_files else pd.DataFrame()
        self.df_usbl = pd.read_csv(usbl_files[-1]) if usbl_files else pd.DataFrame()
        
        # Converti posizioni da metri a centimetri
        self.df_ekf['x'] = self.df_ekf['x'] * 100.0
        self.df_ekf['y'] = self.df_ekf['y'] * 100.0
        self.df_ekf['z'] = self.df_ekf['z'] * 100.0
        
        # Converti depth da metri a centimetri
        if not self.df_depth.empty:
            self.df_depth['depth'] = self.df_depth['depth'] * 100.0
        
        # Converti range USBL da metri a centimetri
        if not self.df_usbl.empty:
            self.df_usbl['range'] = self.df_usbl['range'] * 100.0
        
        print("Dati caricati:")
        print("  EKF: {} samples".format(len(self.df_ekf)))
        print("  IMU: {} samples".format(len(self.df_imu)))
        print("  Depth: {} samples".format(len(self.df_depth)))
        print("  USBL: {} samples".format(len(self.df_usbl)))
        
    def setup_figure(self):
        """Crea la figura con tutti i subplot"""
        self.fig = plt.figure(figsize=(18, 10))
        
        # Layout: 3D grande a sinistra, 2D e depth a destra, sensori sotto
        gs = self.fig.add_gridspec(3, 3, height_ratios=[2, 2, 1], width_ratios=[2, 1, 1])
        
        # Plot 3D (2x2)
        self.ax_3d = self.fig.add_subplot(gs[0:2, 0:2], projection='3d')
        
        # Plot 2D dall'alto (top right)
        self.ax_2d = self.fig.add_subplot(gs[0, 2])
        
        # Plot profondità (middle right)
        self.ax_depth = self.fig.add_subplot(gs[1, 2])
        
        # Plot sensori (bottom row)
        self.ax_imu = self.fig.add_subplot(gs[2, 0])
        self.ax_usbl = self.fig.add_subplot(gs[2, 1])
        self.ax_acc = self.fig.add_subplot(gs[2, 2])
        
        # Inizializza i plot
        self.init_3d_plot()
        self.init_2d_plot()
        self.init_depth_plot()
        self.init_sensor_plots()
        
        # Aggiungi controlli
        self.add_controls()
        
        plt.subplots_adjust(left=0.05, right=0.98, top=0.95, bottom=0.12, 
                          hspace=0.3, wspace=0.3)
        
    def init_3d_plot(self):
        """Inizializza il plot 3D"""
        ax = self.ax_3d
        ax.clear()
        
        # Determina i limiti basati sui dati
        x_data = self.df_ekf['x'].values
        y_data = self.df_ekf['y'].values
        z_data = self.df_ekf['z'].values
        
        x_min = min(x_data.min(), self.tree_pos[0], 0) - 20
        x_max = max(x_data.max(), self.tank_x) + 20
        y_min = min(y_data.min(), 0) - 20
        y_max = max(y_data.max(), self.tank_y, self.tree_pos[1]) + 20
        z_min = min(z_data.min(), -self.tank_z) - 20
        z_max = max(z_data.max(), 0) + 20
        
        ax.set_xlim([x_min, x_max])
        ax.set_ylim([y_min, y_max])
        ax.set_zlim([z_min, z_max])
        
        # Disegna l'acquario
        self.draw_tank(ax)
        
        # Disegna l'albero (keypoint)
        ax.scatter([self.tree_pos[0]], [self.tree_pos[1]], [self.tree_pos[2]], 
                  c='green', marker='s', s=200, label='Albero')
        
        # Disegna USBL transceiver
        ax.scatter([self.usbl_pos[0]], [self.usbl_pos[1]], [self.usbl_pos[2]], 
                  c='red', marker='o', s=300, label='USBL Transceiver')
        
        # Traiettoria completa (leggera)
        ax.plot(x_data, y_data, z_data, 'b-', alpha=0.2, linewidth=0.5)
        
        # Transponder iniziale (verrà aggiornato)
        self.transponder_scatter = ax.scatter([x_data[0]], [y_data[0]], [z_data[0]], 
                                             c='yellow', marker='o', s=200, label='Transponder')
        
        # Assi orientamento (verranno aggiornati)
        self.orientation_lines = []
        for _ in range(3):
            line, = ax.plot([0, 0], [0, 0], [0, 0], linewidth=2)
            self.orientation_lines.append(line)
        
        ax.set_xlabel('X (cm) - Sud')
        ax.set_ylabel('Y (cm) - Est')
        ax.set_zlabel('Z (cm) - Profondità')
        ax.set_title('Vista 3D - Acquario e Traiettoria')
        ax.legend(loc='upper left')
        
    def draw_tank(self, ax):
        """Disegna i bordi dell'acquario con superfici semitrasparenti"""
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        # Vertici acquario
        vertices = np.array([
            [0, 0, 0], [self.tank_x, 0, 0], [self.tank_x, self.tank_y, 0], [0, self.tank_y, 0],
            [0, 0, -self.tank_z], [self.tank_x, 0, -self.tank_z], 
            [self.tank_x, self.tank_y, -self.tank_z], [0, self.tank_y, -self.tank_z]
        ])
        
        # Definisci le 6 facce dell'acquario
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Top (superficie)
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Bottom
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front (Y=0)
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back (Y=tank_y)
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left (X=0)
            [vertices[1], vertices[2], vertices[6], vertices[5]]   # Right (X=tank_x)
        ]
        
        # Aggiungi le superfici semitrasparenti
        face_collection = Poly3DCollection(faces, alpha=0.1, facecolor='cyan', 
                                          edgecolor='blue', linewidth=2)
        ax.add_collection3d(face_collection)
        
        # Aggiungi anche i bordi per maggiore visibilità
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Top
            [4, 5], [5, 6], [6, 7], [7, 4],  # Bottom
            [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical
        ]
        
        for edge in edges:
            points = [vertices[edge[0]], vertices[edge[1]]]
            ax.plot3D(*zip(*points), 'blue', linewidth=2, alpha=0.6)
    
    def init_2d_plot(self):
        """Inizializza il plot 2D dall'alto"""
        ax = self.ax_2d
        ax.clear()
        
        # Determina limiti
        x_data = self.df_ekf['x'].values
        y_data = self.df_ekf['y'].values
        
        x_min = min(x_data.min(), self.tree_pos[0], 0) - 20
        x_max = max(x_data.max(), self.tank_x) + 20
        y_min = min(y_data.min(), 0) - 20
        y_max = max(y_data.max(), self.tank_y, self.tree_pos[1]) + 20
        
        ax.set_xlim([x_min, x_max])
        ax.set_ylim([y_min, y_max])
        
        # Disegna acquario con riempimento semitrasparente
        rect_fill = Rectangle((0, 0), self.tank_x, self.tank_y, 
                              fill=True, facecolor='cyan', alpha=0.1)
        ax.add_patch(rect_fill)
        
        rect_border = Rectangle((0, 0), self.tank_x, self.tank_y, 
                                fill=False, edgecolor='blue', linewidth=2)
        ax.add_patch(rect_border)
        
        # Albero
        ax.scatter([self.tree_pos[0]], [self.tree_pos[1]], 
                  c='green', marker='s', s=100, label='Albero')
        
        # USBL
        ax.scatter([self.usbl_pos[0]], [self.usbl_pos[1]], 
                  c='red', marker='o', s=150, label='USBL')
        
        # Traiettoria
        ax.plot(x_data, y_data, 'b-', alpha=0.3, linewidth=1)
        
        # Posizione corrente
        self.pos_2d, = ax.plot([x_data[0]], [y_data[0]], 'yo', markersize=10, label='Transponder')
        
        ax.set_xlabel('X (cm)')
        ax.set_ylabel('Y (cm)')
        ax.set_title('Vista 2D dall\'alto')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
    def init_depth_plot(self):
        """Inizializza il plot della profondità"""
        ax = self.ax_depth
        ax.clear()
        
        if not self.df_depth.empty:
            t = self.df_depth['timestamp_rel'].values
            z = -self.df_depth['depth'].values  # Negativo perché profondità
            ax.plot(t, z, 'b-', linewidth=1, label='Depth Sensor')
        
        if not self.df_ekf.empty:
            t_ekf = self.df_ekf['timestamp_rel'].values
            z_ekf = self.df_ekf['z'].values
            ax.plot(t_ekf, z_ekf, 'r-', linewidth=1, alpha=0.7, label='EKF Estimate')
        
        # Linea tempo corrente
        self.depth_time_line = ax.axvline(x=0, color='green', linestyle='--', linewidth=2)
        
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Profondità (cm)')
        ax.set_title('Profondità nel tempo')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.invert_yaxis()
        
    def init_sensor_plots(self):
        """Inizializza i plot dei sensori"""
        # IMU orientamento
        ax = self.ax_imu
        ax.clear()
        if not self.df_imu.empty:
            t = self.df_imu['timestamp_rel'].values
            ax.plot(t, self.df_imu['roll'].values, 'r-', linewidth=1, label='Roll', alpha=0.7)
            ax.plot(t, self.df_imu['pitch'].values, 'g-', linewidth=1, label='Pitch', alpha=0.7)
            ax.plot(t, self.df_imu['yaw'].values, 'b-', linewidth=1, label='Yaw', alpha=0.7)
            self.imu_time_line = ax.axvline(x=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Angolo (rad)')
        ax.set_title('IMU - Orientamento')
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)
        
        # USBL range
        ax = self.ax_usbl
        ax.clear()
        if not self.df_usbl.empty:
            t = self.df_usbl['timestamp_rel'].values
            r = self.df_usbl['range'].values
            ax.plot(t, r, 'b-', linewidth=1, label='Range')
            ax.scatter(t, r, c=self.df_usbl['rssi'].values, cmap='viridis', 
                      s=30, alpha=0.6, label='RSSI')
            self.usbl_time_line = ax.axvline(x=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Range (cm)')
        ax.set_title('USBL - Range')
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)
        
        # Accelerazioni
        ax = self.ax_acc
        ax.clear()
        if not self.df_imu.empty:
            t = self.df_imu['timestamp_rel'].values
            ax.plot(t, self.df_imu['acc_x'].values, 'r-', linewidth=1, label='Ax', alpha=0.7)
            ax.plot(t, self.df_imu['acc_y'].values, 'g-', linewidth=1, label='Ay', alpha=0.7)
            ax.plot(t, self.df_imu['acc_z'].values, 'b-', linewidth=1, label='Az', alpha=0.7)
            self.acc_time_line = ax.axvline(x=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel('Tempo (s)')
        ax.set_ylabel('Acc (m/s²)')
        ax.set_title('IMU - Accelerazioni')
        ax.legend(loc='upper right', fontsize=7)
        ax.grid(True, alpha=0.3)
        
    def add_controls(self):
        """Aggiungi controlli di playback"""
        # Pulsanti
        ax_play = plt.axes([0.2, 0.02, 0.08, 0.04])
        ax_restart = plt.axes([0.3, 0.02, 0.08, 0.04])
        
        self.btn_play = Button(ax_play, 'Play/Pause')
        self.btn_play.on_clicked(self.toggle_play)
        
        self.btn_restart = Button(ax_restart, 'Restart')
        self.btn_restart.on_clicked(self.restart)
        
        # Slider velocità
        ax_speed = plt.axes([0.45, 0.02, 0.15, 0.03])
        self.slider_speed = Slider(ax_speed, 'Velocità', 0.1, 5.0, valinit=1.0, valstep=0.1)
        self.slider_speed.on_changed(self.update_speed)
        
        # Slider posizione
        ax_pos = plt.axes([0.65, 0.02, 0.3, 0.03])
        self.slider_pos = Slider(ax_pos, 'Posizione', 0, len(self.df_ekf)-1, 
                                valinit=0, valstep=1)
        self.slider_pos.on_changed(self.update_position)
        
    def toggle_play(self, event):
        """Toggle play/pause"""
        self.is_playing = not self.is_playing
        if self.is_playing:
            self.animate()
    
    def restart(self, event):
        """Restart dall'inizio"""
        self.current_idx = 0
        self.slider_pos.set_val(0)
        self.update_visualization()
    
    def update_speed(self, val):
        """Aggiorna velocità di playback"""
        self.playback_speed = val
    
    def update_position(self, val):
        """Aggiorna posizione dalla slider"""
        self.current_idx = int(val)
        self.update_visualization()
    
    def update_visualization(self):
        """Aggiorna tutti i plot con l'indice corrente"""
        if self.current_idx >= len(self.df_ekf):
            self.is_playing = False
            return
        
        # Dati correnti
        row = self.df_ekf.iloc[self.current_idx]
        x, y, z = row['x'], row['y'], row['z']
        roll, pitch, yaw = row['roll'], row['pitch'], row['yaw']
        t = row['timestamp_rel']
        
        # Aggiorna 3D
        self.transponder_scatter._offsets3d = ([x], [y], [z])
        
        # Aggiorna assi orientamento (10cm di lunghezza)
        axis_len = 10.0
        R = self.rotation_matrix(roll, pitch, yaw)
        
        colors = ['r', 'g', 'b']
        axes_dirs = [np.array([1, 0, 0]), np.array([0, 1, 0]), np.array([0, 0, 1])]
        
        for i, (line, color, axis_dir) in enumerate(zip(self.orientation_lines, colors, axes_dirs)):
            axis_world = R.dot(axis_dir) * axis_len
            line.set_data([x, x + axis_world[0]], [y, y + axis_world[1]])
            line.set_3d_properties([z, z + axis_world[2]])
            line.set_color(color)
        
        # Aggiorna 2D
        self.pos_2d.set_data([x], [y])
        
        # Aggiorna linee temporali
        if hasattr(self, 'depth_time_line'):
            self.depth_time_line.set_xdata([t, t])
        if hasattr(self, 'imu_time_line'):
            self.imu_time_line.set_xdata([t, t])
        if hasattr(self, 'usbl_time_line'):
            self.usbl_time_line.set_xdata([t, t])
        if hasattr(self, 'acc_time_line'):
            self.acc_time_line.set_xdata([t, t])
        
        self.fig.canvas.draw_idle()
    
    def rotation_matrix(self, roll, pitch, yaw):
        """Calcola la matrice di rotazione da angoli di Eulero"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R_roll = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        R_pitch = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        R_yaw = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        
        return R_yaw.dot(R_pitch.dot(R_roll))
    
    def animate(self):
        """Animazione del playback"""
        if not self.is_playing:
            return
        
        self.current_idx += int(10 * self.playback_speed)  # Skip frames basato su velocità
        
        if self.current_idx >= len(self.df_ekf):
            self.current_idx = len(self.df_ekf) - 1
            self.is_playing = False
        
        self.slider_pos.set_val(self.current_idx)
        self.update_visualization()
        
        if self.is_playing:
            self.fig.canvas.mpl_connect('draw_event', lambda evt: plt.pause(0.001))
            self.fig.canvas.draw()
            plt.pause(0.001)
            self.animate()
    
    def show(self):
        """Mostra la GUI"""
        plt.show()


if __name__ == '__main__':
    import sys
    
    log_dir = sys.argv[1] if len(sys.argv) > 1 else 'sensor_logs'
    
    print("=== EKF Visualizer ===")
    print("Caricamento dati da: {}".format(log_dir))
    
    viz = EKFVisualizer(log_dir)
    viz.show()
