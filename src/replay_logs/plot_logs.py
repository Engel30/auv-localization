#!/usr/bin/env python3
"""
Plot dei log EKF e sensori raw
Visualizza risultati filtro di Kalman e misure grezze dai sensori
"""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from datetime import datetime

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
        header = next(reader)  # Skip header
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
#  PLOT FILTRO DI KALMAN
# =============================================================================
def plot_ekf_results(ekf_data, ekf_header):
    """Plot completo dei risultati EKF."""
    
    # Estrai colonne
    t_rel = ekf_data[:, 1]
    pos = ekf_data[:, 2:5]   # x, y, z
    vel = ekf_data[:, 5:8]   # vx, vy, vz
    acc = ekf_data[:, 8:11]  # ax, ay, az
    rpy = ekf_data[:, 11:14] # roll, pitch, yaw
    
    # Converti angoli in gradi
    rpy_deg = np.degrees(rpy)
    
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle('Risultati Filtro di Kalman (EKF)', fontsize=16, fontweight='bold')
    
    # =========================================================================
    # POSIZIONE 3D
    # =========================================================================
    ax1 = fig.add_subplot(3, 3, 1, projection='3d')
    ax1.plot(pos[:, 0], pos[:, 1], pos[:, 2], 'b-', linewidth=1.5, label='Traiettoria')
    ax1.scatter(pos[0, 0], pos[0, 1], pos[0, 2], c='g', s=100, marker='o', label='Start')
    ax1.scatter(pos[-1, 0], pos[-1, 1], pos[-1, 2], c='r', s=100, marker='x', label='End')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_zlabel('Z [m]')
    ax1.set_title('Traiettoria 3D')
    ax1.legend()
    ax1.grid(True)
    
    # =========================================================================
    # POSIZIONE XY (vista dall'alto)
    # =========================================================================
    ax2 = fig.add_subplot(3, 3, 2)
    ax2.plot(pos[:, 0], pos[:, 1], 'b-', linewidth=1.5)
    ax2.scatter(pos[0, 0], pos[0, 1], c='g', s=100, marker='o', label='Start')
    ax2.scatter(pos[-1, 0], pos[-1, 1], c='r', s=100, marker='x', label='End')
    ax2.set_xlabel('X [m]')
    ax2.set_ylabel('Y [m]')
    ax2.set_title('Vista XY (dall\'alto)')
    ax2.grid(True)
    ax2.axis('equal')
    ax2.legend()
    
    # =========================================================================
    # PROFONDITA' (Z nel tempo)
    # =========================================================================
    ax3 = fig.add_subplot(3, 3, 3)
    ax3.plot(t_rel, -pos[:, 2], 'b-', linewidth=1.5)
    ax3.set_xlabel('Tempo [s]')
    ax3.set_ylabel('Profondità [m]')
    ax3.set_title('Profondità (Z negativo)')
    ax3.grid(True)
    ax3.invert_yaxis()
    
    # =========================================================================
    # POSIZIONE XYZ nel tempo
    # =========================================================================
    ax4 = fig.add_subplot(3, 3, 4)
    ax4.plot(t_rel, pos[:, 0], 'r-', label='X', linewidth=1.5)
    ax4.plot(t_rel, pos[:, 1], 'g-', label='Y', linewidth=1.5)
    ax4.plot(t_rel, pos[:, 2], 'b-', label='Z', linewidth=1.5)
    ax4.set_xlabel('Tempo [s]')
    ax4.set_ylabel('Posizione [m]')
    ax4.set_title('Posizione XYZ nel Tempo')
    ax4.legend()
    ax4.grid(True)
    
    # =========================================================================
    # VELOCITA' XYZ
    # =========================================================================
    ax5 = fig.add_subplot(3, 3, 5)
    ax5.plot(t_rel, vel[:, 0], 'r-', label='Vx', linewidth=1.5)
    ax5.plot(t_rel, vel[:, 1], 'g-', label='Vy', linewidth=1.5)
    ax5.plot(t_rel, vel[:, 2], 'b-', label='Vz', linewidth=1.5)
    ax5.set_xlabel('Tempo [s]')
    ax5.set_ylabel('Velocità [m/s]')
    ax5.set_title('Velocità XYZ nel Tempo')
    ax5.legend()
    ax5.grid(True)
    
    # =========================================================================
    # ACCELERAZIONE XYZ
    # =========================================================================
    ax6 = fig.add_subplot(3, 3, 6)
    ax6.plot(t_rel, acc[:, 0], 'r-', label='Ax', linewidth=1.5)
    ax6.plot(t_rel, acc[:, 1], 'g-', label='Ay', linewidth=1.5)
    ax6.plot(t_rel, acc[:, 2], 'b-', label='Az', linewidth=1.5)
    ax6.set_xlabel('Tempo [s]')
    ax6.set_ylabel('Accelerazione [m/s²]')
    ax6.set_title('Accelerazione XYZ nel Tempo')
    ax6.legend()
    ax6.grid(True)
    
    # =========================================================================
    # ROLL
    # =========================================================================
    ax7 = fig.add_subplot(3, 3, 7)
    ax7.plot(t_rel, rpy_deg[:, 0], 'r-', linewidth=1.5)
    ax7.set_xlabel('Tempo [s]')
    ax7.set_ylabel('Roll [deg]')
    ax7.set_title('Roll nel Tempo')
    ax7.grid(True)
    
    # =========================================================================
    # PITCH
    # =========================================================================
    ax8 = fig.add_subplot(3, 3, 8)
    ax8.plot(t_rel, rpy_deg[:, 1], 'g-', linewidth=1.5)
    ax8.set_xlabel('Tempo [s]')
    ax8.set_ylabel('Pitch [deg]')
    ax8.set_title('Pitch nel Tempo')
    ax8.grid(True)
    
    # =========================================================================
    # YAW
    # =========================================================================
    ax9 = fig.add_subplot(3, 3, 9)
    ax9.plot(t_rel, rpy_deg[:, 2], 'b-', linewidth=1.5)
    ax9.set_xlabel('Tempo [s]')
    ax9.set_ylabel('Yaw [deg]')
    ax9.set_title('Yaw nel Tempo')
    ax9.grid(True)
    
    plt.tight_layout()
    return fig

# =============================================================================
#  PLOT SENSORI RAW
# =============================================================================
def plot_raw_sensors(imu_data, depth_data, usbl_data):
    """Plot delle misure raw dei sensori."""
    
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle('Misure Raw Sensori', fontsize=16, fontweight='bold')
    
    # =========================================================================
    # IMU - ACCELEROMETRO
    # =========================================================================
    if imu_data is not None:
        t_imu = imu_data[:, 1]
        rpy_imu = np.degrees(imu_data[:, 2:5])  # roll, pitch, yaw
        acc_imu = imu_data[:, 5:8]  # acc_x, acc_y, acc_z
        gyr_imu = imu_data[:, 8:11]  # gyr_x, gyr_y, gyr_z
        
        ax1 = fig.add_subplot(3, 3, 1)
        ax1.plot(t_imu, acc_imu[:, 0], 'r-', label='Ax', linewidth=1)
        ax1.plot(t_imu, acc_imu[:, 1], 'g-', label='Ay', linewidth=1)
        ax1.plot(t_imu, acc_imu[:, 2], 'b-', label='Az', linewidth=1)
        ax1.set_xlabel('Tempo [s]')
        ax1.set_ylabel('Accelerazione [m/s²]')
        ax1.set_title('IMU - Accelerometro')
        ax1.legend()
        ax1.grid(True)
        
        # =====================================================================
        # IMU - GIROSCOPIO
        # =====================================================================
        ax2 = fig.add_subplot(3, 3, 2)
        ax2.plot(t_imu, gyr_imu[:, 0], 'r-', label='Gx', linewidth=1)
        ax2.plot(t_imu, gyr_imu[:, 1], 'g-', label='Gy', linewidth=1)
        ax2.plot(t_imu, gyr_imu[:, 2], 'b-', label='Gz', linewidth=1)
        ax2.set_xlabel('Tempo [s]')
        ax2.set_ylabel('Velocità angolare [rad/s]')
        ax2.set_title('IMU - Giroscopio')
        ax2.legend()
        ax2.grid(True)
        
        # =====================================================================
        # IMU - ROLL
        # =====================================================================
        ax3 = fig.add_subplot(3, 3, 3)
        ax3.plot(t_imu, rpy_imu[:, 0], 'r-', linewidth=1.5)
        ax3.set_xlabel('Tempo [s]')
        ax3.set_ylabel('Roll [deg]')
        ax3.set_title('IMU - Roll')
        ax3.grid(True)
        
        # =====================================================================
        # IMU - PITCH
        # =====================================================================
        ax4 = fig.add_subplot(3, 3, 4)
        ax4.plot(t_imu, rpy_imu[:, 1], 'g-', linewidth=1.5)
        ax4.set_xlabel('Tempo [s]')
        ax4.set_ylabel('Pitch [deg]')
        ax4.set_title('IMU - Pitch')
        ax4.grid(True)
        
        # =====================================================================
        # IMU - YAW
        # =====================================================================
        ax5 = fig.add_subplot(3, 3, 5)
        ax5.plot(t_imu, rpy_imu[:, 2], 'b-', linewidth=1.5)
        ax5.set_xlabel('Tempo [s]')
        ax5.set_ylabel('Yaw [deg]')
        ax5.set_title('IMU - Yaw')
        ax5.grid(True)
    
    # =========================================================================
    # DEPTH SENSOR
    # =========================================================================
    if depth_data is not None:
        t_depth = depth_data[:, 1]
        depth_vals = depth_data[:, 2]
        
        ax6 = fig.add_subplot(3, 3, 6)
        ax6.plot(t_depth, depth_vals, 'b-', linewidth=1.5)
        ax6.set_xlabel('Tempo [s]')
        ax6.set_ylabel('Profondità [m]')
        ax6.set_title('Depth Sensor')
        ax6.grid(True)
    
    # =========================================================================
    # USBL - RANGE
    # =========================================================================
    if usbl_data is not None and len(usbl_data) > 0:
        t_usbl = usbl_data[:, 1]
        range_usbl = usbl_data[:, 2]
        integrity_usbl = usbl_data[:, 3]
        rssi_usbl = usbl_data[:, 4]
        
        # Filtra valori validi (range > 0)
        valid_idx = range_usbl > 0
        
        if np.any(valid_idx):
            ax7 = fig.add_subplot(3, 3, 7)
            ax7.plot(t_usbl[valid_idx], range_usbl[valid_idx], 'ko-', 
                    linewidth=1.5, markersize=6, markerfacecolor='orange')
            ax7.set_xlabel('Tempo [s]')
            ax7.set_ylabel('Range [m]')
            ax7.set_title('USBL - Range')
            ax7.grid(True)
            
            # =================================================================
            # USBL - INTEGRITY
            # =================================================================
            ax8 = fig.add_subplot(3, 3, 8)
            ax8.plot(t_usbl[valid_idx], integrity_usbl[valid_idx], 'go-', 
                    linewidth=1.5, markersize=6)
            ax8.set_xlabel('Tempo [s]')
            ax8.set_ylabel('Integrity [%]')
            ax8.set_title('USBL - Integrity')
            ax8.grid(True)
            ax8.set_ylim([0, 105])
            
            # =================================================================
            # USBL - RSSI
            # =================================================================
            ax9 = fig.add_subplot(3, 3, 9)
            ax9.plot(t_usbl[valid_idx], rssi_usbl[valid_idx], 'ro-', 
                    linewidth=1.5, markersize=6)
            ax9.set_xlabel('Tempo [s]')
            ax9.set_ylabel('RSSI [dB]')
            ax9.set_title('USBL - RSSI')
            ax9.grid(True)
        else:
            print("[WARNING] Nessun dato USBL valido (range > 0)")
    
    plt.tight_layout()
    return fig

# =============================================================================
#  MAIN
# =============================================================================
def main():
    print("=" * 70)
    print("PLOT DEI LOG EKF E SENSORI RAW")
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
    ekf_header = ekf_result[1] if ekf_result else None
    imu_data = imu_result[0] if imu_result else None
    depth_data = depth_result[0] if depth_result else None
    usbl_data = usbl_result[0] if usbl_result else None
    
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
        print("  USBL: {} campioni totali, {} validi (range > 0)".format(
            len(usbl_data), valid_usbl))
    
    # Plot EKF
    if ekf_data is not None:
        print("\nGenerazione plot EKF...")
        fig_ekf = plot_ekf_results(ekf_data, ekf_header)
        fig_ekf.savefig('plot_ekf_results.png', dpi=150, bbox_inches='tight')
        print("  Salvato: plot_ekf_results.png")
    else:
        print("\nNessun dato EKF disponibile!")
    
    # Plot sensori raw
    if imu_data is not None or depth_data is not None or usbl_data is not None:
        print("\nGenerazione plot sensori raw...")
        fig_raw = plot_raw_sensors(imu_data, depth_data, usbl_data)
        fig_raw.savefig('plot_raw_sensors.png', dpi=150, bbox_inches='tight')
        print("  Salvato: plot_raw_sensors.png")
    else:
        print("\nNessun dato sensori disponibile!")
    
    print("\n" + "=" * 70)
    print("Plot completati! Apertura finestre...")
    plt.show()

if __name__ == "__main__":
    main()
