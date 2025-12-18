#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualizzazione dati sensoriali AUV
- Mappa 2D con traccia robot e riferimenti
- Range USBL nel tempo
"""

import os
import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from matplotlib.collections import LineCollection

# ============================================================================
# CONFIGURAZIONE
# ============================================================================

# Coordinate riferimenti (in metri)
BOA_COORDINATES = (0.0, 0.0)  # Posizione BOA (origine, cerchio rosso)
NORTH_MARKER_COORDINATES = (0.0, 8.0)  # Quadratino grigio a nord

# Parametri piscina
POOL_RADIUS = 8.0  # metri

# Directory dati
DATA_DIR = "sensor_logs"

# ============================================================================
# CARICAMENTO DATI
# ============================================================================

def find_latest_csv(directory, prefix):
    """Trova il file CSV più recente con il prefisso specificato"""
    pattern = os.path.join(directory, f"{prefix}_*.csv")
    files = glob.glob(pattern)
    if not files:
        raise FileNotFoundError(f"Nessun file trovato con pattern: {pattern}")
    # Prendi il file più recente se ce ne sono più di uno
    return max(files, key=os.path.getmtime)

print("Caricamento dati sensori...")

# Carica i CSV
ekf_file = find_latest_csv(DATA_DIR, "ekf")
usbl_file = find_latest_csv(DATA_DIR, "usbl")

print(f"  EKF: {os.path.basename(ekf_file)}")
print(f"  USBL: {os.path.basename(usbl_file)}")

ekf_data = pd.read_csv(ekf_file)
usbl_data = pd.read_csv(usbl_file)

# Estrai dati rilevanti
time_ekf = ekf_data['timestamp_rel'].values
x = ekf_data['x'].values
y = ekf_data['y'].values

time_usbl = usbl_data['timestamp_rel'].values
usbl_range = usbl_data['range'].values

print(f"\nDati caricati:")
print(f"  EKF samples: {len(ekf_data)}")
print(f"  USBL fixes: {len(usbl_data)}")

# ============================================================================
# VISUALIZZAZIONE
# ============================================================================

fig = plt.figure(figsize=(16, 7))

# --- SUBPLOT 1: Mappa 2D ---
ax1 = fig.add_subplot(121, aspect='equal')

# Piscina circolare
pool = Circle((0, 0), POOL_RADIUS, fill=False, edgecolor='blue', 
              linewidth=2, linestyle='--', label='Piscina (R=8m)')
ax1.add_patch(pool)

# BOA (origine)
boa = Circle(BOA_COORDINATES, 0.3, color='red', alpha=0.7, label='BOA')
ax1.add_patch(boa)

# Marker nord
ax1.plot(NORTH_MARKER_COORDINATES[0], NORTH_MARKER_COORDINATES[1], 
         's', color='gray', markersize=12, label='Nord', zorder=5)

# Traccia robot con colormap per il tempo
# Crea segmenti per LineCollection
points = np.array([x, y]).T.reshape(-1, 1, 2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)

# Normalizza il tempo per la colormap
norm = plt.Normalize(time_ekf.min(), time_ekf.max())
lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=2)
lc.set_array(time_ekf[:-1])  # Usa il tempo di inizio di ogni segmento

ax1.add_collection(lc)

# Marker posizione iniziale e finale
ax1.plot(x[0], y[0], 'go', markersize=10, label='Start', zorder=6)
ax1.plot(x[-1], y[-1], 'rs', markersize=10, label='End', zorder=6)

# Colorbar per il tempo
cbar = plt.colorbar(lc, ax=ax1, label='Tempo [s]')

# Impostazioni assi
ax1.set_xlabel('X [m]', fontsize=12)
ax1.set_ylabel('Y [m]', fontsize=12)
ax1.set_title('Traiettoria Robot', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.legend(loc='upper right')

# Limiti assi (leggermente più ampi della piscina)
margin = 1.5
ax1.set_xlim(-POOL_RADIUS - margin, POOL_RADIUS + margin)
ax1.set_ylim(-POOL_RADIUS - margin, POOL_RADIUS + margin)

# Aggiungi assi cartesiani
ax1.axhline(y=0, color='k', linewidth=0.5, alpha=0.3)
ax1.axvline(x=0, color='k', linewidth=0.5, alpha=0.3)

# --- SUBPLOT 2: Range USBL ---
ax2 = fig.add_subplot(122)

ax2.plot(time_usbl, usbl_range, 'o-', color='steelblue', 
         markersize=6, linewidth=1.5, label='USBL Range')

ax2.set_xlabel('Tempo [s]', fontsize=12)
ax2.set_ylabel('Range [m]', fontsize=12)
ax2.set_title('Fix USBL nel Tempo', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.legend()

# Statistiche range
mean_range = np.mean(usbl_range)
std_range = np.std(usbl_range)
ax2.axhline(y=mean_range, color='red', linestyle='--', 
            alpha=0.5, label=f'Media: {mean_range:.2f}m')
ax2.fill_between(time_usbl, mean_range - std_range, mean_range + std_range, 
                 alpha=0.2, color='red', label=f'±1σ: {std_range:.2f}m')
ax2.legend()

# Layout
plt.tight_layout()

# Stampa statistiche
print(f"\n=== STATISTICHE ===")
print(f"Range USBL:")
print(f"  Media: {mean_range:.3f} m")
print(f"  Std Dev: {std_range:.3f} m")
print(f"  Min: {np.min(usbl_range):.3f} m")
print(f"  Max: {np.max(usbl_range):.3f} m")
print(f"  Numero fix: {len(usbl_range)}")

print(f"\nTraiettoria:")
print(f"  Durata: {time_ekf[-1] - time_ekf[0]:.2f} s")
print(f"  Distanza percorsa: {np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2)):.2f} m")
print(f"  Posizione iniziale: ({x[0]:.3f}, {y[0]:.3f}) m")
print(f"  Posizione finale: ({x[-1]:.3f}, {y[-1]:.3f}) m")

plt.show()