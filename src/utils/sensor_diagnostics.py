#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Diagnostica Sensori - Analizza IMU, USBL e Depth
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ============================================================================
# CARICAMENTO DATI
# ============================================================================

def find_latest_csv(directory, prefix):
    pattern = os.path.join(directory, f"{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"Nessun file trovato con pattern: {pattern}")
    return max(files, key=os.path.getmtime)

DATA_DIR = "sensor_logs"

print("=== DIAGNOSTICA SENSORI ===\n")
print("Caricamento dati...")

imu_data = pd.read_csv(find_latest_csv(DATA_DIR, "imu"))
usbl_data = pd.read_csv(find_latest_csv(DATA_DIR, "usbl"))
depth_data = pd.read_csv(find_latest_csv(DATA_DIR, "depth"))

# ============================================================================
# ANALISI IMU - BIAS E RUMORE
# ============================================================================

print("\n1. ANALISI IMU")
print("-" * 50)

# Accelerazioni (dovrebbero essere vicine a [0, 0, -9.81] a robot fermo)
ax = imu_data['acc_x'].values
ay = imu_data['acc_y'].values
az = imu_data['acc_z'].values

print(f"\nAccelerometro [m/s²]:")
print(f"  X - Media: {np.mean(ax):7.3f}  Std: {np.std(ax):.3f}  Min/Max: {np.min(ax):.2f}/{np.max(ax):.2f}")
print(f"  Y - Media: {np.mean(ay):7.3f}  Std: {np.std(ay):.3f}  Min/Max: {np.min(ay):.2f}/{np.max(ay):.2f}")
print(f"  Z - Media: {np.mean(az):7.3f}  Std: {np.std(az):.3f}  Min/Max: {np.min(az):.2f}/{np.max(az):.2f}")
print(f"  Magnitudine - Media: {np.mean(np.sqrt(ax**2 + ay**2 + az**2)):.3f} m/s²")
print(f"  NOTA: Dovrebbe essere ~9.81 m/s² se il robot è fermo")

# Velocità angolari (dovrebbero essere ~0 a robot fermo)
gx = imu_data['gyr_x'].values
gy = imu_data['gyr_y'].values
gz = imu_data['gyr_z'].values

print(f"\nGiroscopio [rad/s]:")
print(f"  X - Media: {np.mean(gx):7.4f}  Std: {np.std(gx):.4f}  Min/Max: {np.min(gx):.3f}/{np.max(gx):.3f}")
print(f"  Y - Media: {np.mean(gy):7.4f}  Std: {np.std(gy):.4f}  Min/Max: {np.min(gy):.3f}/{np.max(gy):.3f}")
print(f"  Z - Media: {np.mean(gz):7.4f}  Std: {np.std(gz):.4f}  Min/Max: {np.min(gz):.3f}/{np.max(gz):.3f}")

# Identifica bias significativi
bias_threshold = 0.1  # m/s² per accelerometro
gyro_bias_threshold = 0.01  # rad/s per giroscopio

ax_bias = abs(np.mean(ax)) > bias_threshold
ay_bias = abs(np.mean(ay)) > bias_threshold
az_bias = abs(np.mean(az) + 9.81) > bias_threshold  # Dovrebbe essere -9.81

gx_bias = abs(np.mean(gx)) > gyro_bias_threshold
gy_bias = abs(np.mean(gy)) > gyro_bias_threshold
gz_bias = abs(np.mean(gz)) > gyro_bias_threshold

if ax_bias or ay_bias or az_bias:
    print(f"\n  ⚠️  BIAS ACCELEROMETRO RILEVATO!")
if gx_bias or gy_bias or gz_bias:
    print(f"  ⚠️  BIAS GIROSCOPIO RILEVATO!")

# Frequenza campionamento
imu_dt = np.diff(imu_data['timestamp_rel'].values)
print(f"\nFrequenza campionamento:")
print(f"  Media: {1/np.mean(imu_dt):.1f} Hz")
print(f"  Intervallo min/max: {np.min(imu_dt)*1000:.1f} / {np.max(imu_dt)*1000:.1f} ms")

# ============================================================================
# ANALISI USBL - FREQUENZA E CONSISTENZA
# ============================================================================

print("\n2. ANALISI USBL")
print("-" * 50)

usbl_times = usbl_data['timestamp_rel'].values
usbl_ranges = usbl_data['range'].values
usbl_dt = np.diff(usbl_times)

print(f"\nFrequenza fix USBL:")
print(f"  Numero fix: {len(usbl_data)}")
print(f"  Intervallo medio: {np.mean(usbl_dt):.2f} s")
print(f"  Intervallo min/max: {np.min(usbl_dt):.2f} / {np.max(usbl_dt):.2f} s")

print(f"\nRange USBL:")
print(f"  Media: {np.mean(usbl_ranges):.2f} m")
print(f"  Std: {np.std(usbl_ranges):.2f} m")
print(f"  Min/Max: {np.min(usbl_ranges):.2f} / {np.max(usbl_ranges):.2f} m")

# Variazione range tra misure consecutive
range_changes = np.abs(np.diff(usbl_ranges))
print(f"\nVariazione range tra fix consecutivi:")
print(f"  Media: {np.mean(range_changes):.3f} m")
print(f"  Max: {np.max(range_changes):.3f} m")

if np.max(range_changes) > 2.0:
    print(f"  ⚠️  SALTI RANGE ELEVATI! Possibili outlier o perdita tracking")

# ============================================================================
# ANALISI DEPTH
# ============================================================================

print("\n3. ANALISI DEPTH")
print("-" * 50)

depth_values = depth_data['depth'].values
depth_dt = np.diff(depth_data['timestamp_rel'].values)

print(f"\nProfondità [m]:")
print(f"  Media: {np.mean(depth_values):.3f} m")
print(f"  Std: {np.std(depth_values):.3f} m")
print(f"  Min/Max: {np.min(depth_values):.3f} / {np.max(depth_values):.3f} m")

print(f"\nFrequenza campionamento:")
print(f"  Media: {1/np.mean(depth_dt):.1f} Hz")

# ============================================================================
# VISUALIZZAZIONE DIAGNOSTICA
# ============================================================================

fig = plt.figure(figsize=(16, 10))
gs = GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.3)

# Plot 1: Accelerazioni IMU raw
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(imu_data['timestamp_rel'], ax, label='X', alpha=0.7)
ax1.plot(imu_data['timestamp_rel'], ay, label='Y', alpha=0.7)
ax1.plot(imu_data['timestamp_rel'], az, label='Z', alpha=0.7)
ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax1.axhline(y=-9.81, color='r', linestyle='--', alpha=0.3, label='Gravità')
ax1.set_xlabel('Tempo [s]')
ax1.set_ylabel('Accelerazione [m/s²]')
ax1.set_title('Accelerometro RAW')
ax1.legend()
ax1.grid(True, alpha=0.3)

# Plot 2: Giroscopio
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(imu_data['timestamp_rel'], np.rad2deg(gx), label='X', alpha=0.7)
ax2.plot(imu_data['timestamp_rel'], np.rad2deg(gy), label='Y', alpha=0.7)
ax2.plot(imu_data['timestamp_rel'], np.rad2deg(gz), label='Z', alpha=0.7)
ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax2.set_xlabel('Tempo [s]')
ax2.set_ylabel('Velocità angolare [°/s]')
ax2.set_title('Giroscopio RAW')
ax2.legend()
ax2.grid(True, alpha=0.3)

# Plot 3: Magnitudine accelerazione
ax3 = fig.add_subplot(gs[1, 0])
acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
ax3.plot(imu_data['timestamp_rel'], acc_mag, color='blue', alpha=0.7)
ax3.axhline(y=9.81, color='r', linestyle='--', alpha=0.5, label='Gravità teorica')
ax3.set_xlabel('Tempo [s]')
ax3.set_ylabel('Magnitudine [m/s²]')
ax3.set_title('Magnitudine Accelerazione')
ax3.legend()
ax3.grid(True, alpha=0.3)

# Plot 4: USBL Range
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(usbl_times, usbl_ranges, 'o-', label='Range USBL', markersize=6)
ax4.set_xlabel('Tempo [s]')
ax4.set_ylabel('Range [m]')
ax4.set_title('Range USBL')
ax4.legend()
ax4.grid(True, alpha=0.3)

# Plot 5: Depth
ax5 = fig.add_subplot(gs[2, 0])
ax5.plot(depth_data['timestamp_rel'], depth_values, color='blue', alpha=0.7)
ax5.set_xlabel('Tempo [s]')
ax5.set_ylabel('Profondità [m]')
ax5.set_title('Depth Sensor')
ax5.grid(True, alpha=0.3)

# Plot 6: Variazione Range USBL
ax6 = fig.add_subplot(gs[2, 1])
ax6.plot(usbl_times[:-1], range_changes, 'o-', color='orange', markersize=6)
ax6.set_xlabel('Tempo [s]')
ax6.set_ylabel('Variazione Range [m]')
ax6.set_title('Variazione Range tra fix USBL consecutivi')
ax6.grid(True, alpha=0.3)

plt.savefig('sensor_diagnostics.png', dpi=150, bbox_inches='tight')
print("\n\nGrafico diagnostico salvato: sensor_diagnostics.png")

# ============================================================================
# RIEPILOGO QUALITÀ SENSORI
# ============================================================================

print("\n" + "="*70)
print("RIEPILOGO QUALITÀ SENSORI")
print("="*70)

print("\nIMU:")
if ax_bias or ay_bias or az_bias:
    print("  ⚠️  Bias accelerometro presente")
else:
    print("  ✓  Accelerometro OK")

if gx_bias or gy_bias or gz_bias:
    print("  ⚠️  Bias giroscopio presente")
else:
    print("  ✓  Giroscopio OK")

print("\nUSBL:")
if len(usbl_data) < 10:
    print("  ⚠️  Pochi fix disponibili")
elif np.max(range_changes) > 2.0:
    print("  ⚠️  Salti range anomali")
else:
    print("  ✓  USBL consistente")

print("\nDepth:")
if np.std(depth_values) > 0.1:
    print("  ⚠️  Rumore elevato sul depth")
else:
    print("  ✓  Depth stabile")

print("\n" + "="*70)