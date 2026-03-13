#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensor Diagnostics - Analyses IMU, USBL and Depth sensor quality
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# ============================================================================
# DATA LOADING
# ============================================================================

def find_latest_csv(directory, prefix):
    pattern = os.path.join(directory, f"{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"No file found matching pattern: {pattern}")
    return max(files, key=os.path.getmtime)

DATA_DIR = "sensor_logs"

print("=== SENSOR DIAGNOSTICS ===\n")
print("Loading data...")

imu_data = pd.read_csv(find_latest_csv(DATA_DIR, "imu"))
usbl_data = pd.read_csv(find_latest_csv(DATA_DIR, "usbl"))
depth_data = pd.read_csv(find_latest_csv(DATA_DIR, "depth"))

# ============================================================================
# IMU ANALYSIS — BIAS AND NOISE
# ============================================================================

print("\n1. IMU ANALYSIS")
print("-" * 50)

# Accelerations (should be close to [0, 0, -9.81] when the robot is stationary)
ax = imu_data['acc_x'].values
ay = imu_data['acc_y'].values
az = imu_data['acc_z'].values

print(f"\nAccelerometer [m/s²]:")
print(f"  X - Mean: {np.mean(ax):7.3f}  Std: {np.std(ax):.3f}  Min/Max: {np.min(ax):.2f}/{np.max(ax):.2f}")
print(f"  Y - Mean: {np.mean(ay):7.3f}  Std: {np.std(ay):.3f}  Min/Max: {np.min(ay):.2f}/{np.max(ay):.2f}")
print(f"  Z - Mean: {np.mean(az):7.3f}  Std: {np.std(az):.3f}  Min/Max: {np.min(az):.2f}/{np.max(az):.2f}")
print(f"  Magnitude - Mean: {np.mean(np.sqrt(ax**2 + ay**2 + az**2)):.3f} m/s²")
print(f"  NOTE: Should be ~9.81 m/s² when the robot is stationary")

# Angular rates (should be ~0 when the robot is stationary)
gx = imu_data['gyr_x'].values
gy = imu_data['gyr_y'].values
gz = imu_data['gyr_z'].values

print(f"\nGyroscope [rad/s]:")
print(f"  X - Mean: {np.mean(gx):7.4f}  Std: {np.std(gx):.4f}  Min/Max: {np.min(gx):.3f}/{np.max(gx):.3f}")
print(f"  Y - Mean: {np.mean(gy):7.4f}  Std: {np.std(gy):.4f}  Min/Max: {np.min(gy):.3f}/{np.max(gy):.3f}")
print(f"  Z - Mean: {np.mean(gz):7.4f}  Std: {np.std(gz):.4f}  Min/Max: {np.min(gz):.3f}/{np.max(gz):.3f}")

# Identify significant biases
bias_threshold = 0.1        # m/s² for accelerometer
gyro_bias_threshold = 0.01  # rad/s for gyroscope

ax_bias = abs(np.mean(ax)) > bias_threshold
ay_bias = abs(np.mean(ay)) > bias_threshold
az_bias = abs(np.mean(az) + 9.81) > bias_threshold  # Should be -9.81

gx_bias = abs(np.mean(gx)) > gyro_bias_threshold
gy_bias = abs(np.mean(gy)) > gyro_bias_threshold
gz_bias = abs(np.mean(gz)) > gyro_bias_threshold

if ax_bias or ay_bias or az_bias:
    print(f"\n  WARNING: ACCELEROMETER BIAS DETECTED!")
if gx_bias or gy_bias or gz_bias:
    print(f"  WARNING: GYROSCOPE BIAS DETECTED!")

# Sampling frequency
imu_dt = np.diff(imu_data['timestamp_rel'].values)
print(f"\nSampling frequency:")
print(f"  Mean: {1/np.mean(imu_dt):.1f} Hz")
print(f"  Interval min/max: {np.min(imu_dt)*1000:.1f} / {np.max(imu_dt)*1000:.1f} ms")

# ============================================================================
# USBL ANALYSIS — FREQUENCY AND CONSISTENCY
# ============================================================================

print("\n2. USBL ANALYSIS")
print("-" * 50)

usbl_times = usbl_data['timestamp_rel'].values
usbl_ranges = usbl_data['range'].values
usbl_dt = np.diff(usbl_times)

print(f"\nUSBL fix frequency:")
print(f"  Number of fixes: {len(usbl_data)}")
print(f"  Mean interval: {np.mean(usbl_dt):.2f} s")
print(f"  Interval min/max: {np.min(usbl_dt):.2f} / {np.max(usbl_dt):.2f} s")

print(f"\nUSBL range:")
print(f"  Mean: {np.mean(usbl_ranges):.2f} m")
print(f"  Std: {np.std(usbl_ranges):.2f} m")
print(f"  Min/Max: {np.min(usbl_ranges):.2f} / {np.max(usbl_ranges):.2f} m")

# Range variation between consecutive fixes
range_changes = np.abs(np.diff(usbl_ranges))
print(f"\nRange variation between consecutive fixes:")
print(f"  Mean: {np.mean(range_changes):.3f} m")
print(f"  Max: {np.max(range_changes):.3f} m")

if np.max(range_changes) > 2.0:
    print(f"  WARNING: LARGE RANGE JUMPS DETECTED! Possible outliers or tracking loss")

# ============================================================================
# DEPTH ANALYSIS
# ============================================================================

print("\n3. DEPTH ANALYSIS")
print("-" * 50)

depth_values = depth_data['depth'].values
depth_dt = np.diff(depth_data['timestamp_rel'].values)

print(f"\nDepth [m]:")
print(f"  Mean: {np.mean(depth_values):.3f} m")
print(f"  Std: {np.std(depth_values):.3f} m")
print(f"  Min/Max: {np.min(depth_values):.3f} / {np.max(depth_values):.3f} m")

print(f"\nSampling frequency:")
print(f"  Mean: {1/np.mean(depth_dt):.1f} Hz")

# ============================================================================
# DIAGNOSTIC VISUALIZATION — 2×2 grid
# ============================================================================

fig = plt.figure(figsize=(14, 9))
gs = GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.3)

# Plot 1: Accelerometer RAW
ax1 = fig.add_subplot(gs[0, 0])
ax1.plot(imu_data['timestamp_rel'], ax, label='X', alpha=0.7)
ax1.plot(imu_data['timestamp_rel'], ay, label='Y', alpha=0.7)
ax1.plot(imu_data['timestamp_rel'], az, label='Z', alpha=0.7)
ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax1.set_xlabel('Time [s]')
ax1.set_ylabel('Acceleration [m/s²]')
ax1.set_title('Accelerometer RAW', fontweight='bold')
ax1.legend(fontsize=8)
ax1.grid(True, alpha=0.3)

# Plot 2: Gyroscope RAW
ax2 = fig.add_subplot(gs[0, 1])
ax2.plot(imu_data['timestamp_rel'], np.rad2deg(gx), label='X', alpha=0.7)
ax2.plot(imu_data['timestamp_rel'], np.rad2deg(gy), label='Y', alpha=0.7)
ax2.plot(imu_data['timestamp_rel'], np.rad2deg(gz), label='Z', alpha=0.7)
ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
ax2.set_xlabel('Time [s]')
ax2.set_ylabel('Angular rate [°/s]')
ax2.set_title('Gyroscope RAW', fontweight='bold')
ax2.legend(fontsize=8)
ax2.grid(True, alpha=0.3)

# Plot 3: USBL Range
ax3 = fig.add_subplot(gs[1, 0])
ax3.plot(usbl_times, usbl_ranges, 'o-', color='steelblue',
         markersize=6, linewidth=1.5, label='Measured range')
ax3.axhline(np.mean(usbl_ranges), color='blue', ls='--', alpha=0.4,
            linewidth=1, label='Mean range')
ax3.set_xlabel('Time [s]')
ax3.set_ylabel('Range [m]')
ax3.set_title('USBL Range', fontweight='bold')
ax3.legend(fontsize=8)
ax3.grid(True, alpha=0.3)

# Plot 4: Depth Profile — marine convention (surface at top, Y inverted)
ax4 = fig.add_subplot(gs[1, 1])
ax4.plot(depth_data['timestamp_rel'], depth_values,
         '-', color='teal', linewidth=1.5)
ax4.fill_between(depth_data['timestamp_rel'], depth_values,
                 alpha=0.15, color='teal')
ax4.invert_yaxis()
ax4.set_xlabel('Time [s]')
ax4.set_ylabel('Depth [m]')
ax4.set_title('Depth Profile', fontweight='bold')
ax4.grid(True, alpha=0.3)

plt.savefig('sensor_diagnostics.png', dpi=150, bbox_inches='tight')
plt.savefig('sensor_diagnostics.svg', bbox_inches='tight')
print("\n\nDiagnostic plots saved: sensor_diagnostics.png  |  sensor_diagnostics.svg")

# ============================================================================
# SENSOR QUALITY SUMMARY
# ============================================================================

print("\n" + "="*70)
print("SENSOR QUALITY SUMMARY")
print("="*70)

print("\nIMU:")
if ax_bias or ay_bias or az_bias:
    print("  WARNING: Accelerometer bias present")
else:
    print("  OK  Accelerometer")

if gx_bias or gy_bias or gz_bias:
    print("  WARNING: Gyroscope bias present")
else:
    print("  OK  Gyroscope")

print("\nUSBL:")
if len(usbl_data) < 10:
    print("  WARNING: Too few fixes available")
elif np.max(range_changes) > 2.0:
    print("  WARNING: Anomalous range jumps detected")
else:
    print("  OK  USBL consistent")

print("\nDepth:")
if np.std(depth_values) > 0.1:
    print("  WARNING: High noise on depth sensor")
else:
    print("  OK  Depth stable")

print("\n" + "="*70)