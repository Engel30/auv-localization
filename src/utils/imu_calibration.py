#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Calibrazione statica IMU - Estrazione bias e noise variance
Esegui con il robot FERMO per ~60 secondi
"""

import numpy as np
import pandas as pd
import json
import sys

def calibrate_imu_static(csv_file, output_file="imu_calibration.json"):
    """
    Calibra IMU da dati statici
    
    Args:
        csv_file: path al CSV con dati IMU statici
        output_file: path file output JSON
    
    Returns:
        dict con bias e varianze
    """
    print("="*70)
    print("CALIBRAZIONE STATICA IMU")
    print("="*70)
    
    # Carica dati
    print(f"\nCaricamento: {csv_file}")
    data = pd.read_csv(csv_file)
    
    n_samples = len(data)
    duration = data['timestamp_rel'].iloc[-1] - data['timestamp_rel'].iloc[0]
    
    print(f"  Campioni: {n_samples}")
    print(f"  Durata: {duration:.2f} s")
    print(f"  Frequenza: {n_samples/duration:.1f} Hz")
    
    # Estrai dati
    acc_x = data['acc_x'].values
    acc_y = data['acc_y'].values
    acc_z = data['acc_z'].values
    
    gyr_x = data['gyr_x'].values
    gyr_y = data['gyr_y'].values
    gyr_z = data['gyr_z'].values
    
    # Calcola BIAS (media)
    bias_acc_x = np.mean(acc_x)
    bias_acc_y = np.mean(acc_y)
    bias_acc_z = np.mean(acc_z)
    
    bias_gyr_x = np.mean(gyr_x)
    bias_gyr_y = np.mean(gyr_y)
    bias_gyr_z = np.mean(gyr_z)
    
    # Calcola NOISE VARIANCE (varianza)
    var_acc_x = np.var(acc_x)
    var_acc_y = np.var(acc_y)
    var_acc_z = np.var(acc_z)
    
    var_gyr_x = np.var(gyr_x)
    var_gyr_y = np.var(gyr_y)
    var_gyr_z = np.var(gyr_z)
    
    # Risultati
    print("\n" + "="*70)
    print("RISULTATI CALIBRAZIONE")
    print("="*70)
    
    print("\nBIAS Accelerometro [m/s²]:")
    print(f"  X: {bias_acc_x:+.6f}")
    print(f"  Y: {bias_acc_y:+.6f}")
    print(f"  Z: {bias_acc_z:+.6f}")
    
    print("\nNOISE STD Accelerometro [m/s²]:")
    print(f"  X: {np.sqrt(var_acc_x):.6f}")
    print(f"  Y: {np.sqrt(var_acc_y):.6f}")
    print(f"  Z: {np.sqrt(var_acc_z):.6f}")
    
    print("\nBIAS Giroscopio [rad/s]:")
    print(f"  X: {bias_gyr_x:+.6f}")
    print(f"  Y: {bias_gyr_y:+.6f}")
    print(f"  Z: {bias_gyr_z:+.6f}")
    
    print("\nNOISE STD Giroscopio [rad/s]:")
    print(f"  X: {np.sqrt(var_gyr_x):.6f}")
    print(f"  Y: {np.sqrt(var_gyr_y):.6f}")
    print(f"  Z: {np.sqrt(var_gyr_z):.6f}")
    
    # Crea dizionario calibrazione
    calibration = {
        'bias_acc': [bias_acc_x, bias_acc_y, bias_acc_z],
        'bias_gyr': [bias_gyr_x, bias_gyr_y, bias_gyr_z],
        'var_acc': [var_acc_x, var_acc_y, var_acc_z],
        'var_gyr': [var_gyr_x, var_gyr_y, var_gyr_z],
        'n_samples': int(n_samples),
        'duration': float(duration)
    }
    
    # Salva JSON
    with open(output_file, 'w') as f:
        json.dump(calibration, f, indent=4)
    
    print(f"\n✓ Calibrazione salvata: {output_file}")
    print("="*70)
    
    return calibration


if __name__ == "__main__":
    #if len(sys.argv) < 2:
    #    print("Usage: python imu_calibration.py <imu_static_data.csv> [output.json]")
    #    print("\nEsempio: python imu_calibration.py sensor_logs/imu_static.csv")
    #    sys.exit(1)
    
    calibrate_imu_static("sensor_logs/imu_20251218_155714.csv", "imu_calibration.json")

    #csv_file = "sensor_logs/imu_20251218_155714.csv"
    #output_file = sys.argv[2] if len(sys.argv) > 2 else "imu_calibration.json"
    
    #calibrate_imu_static(csv_file, output_file)