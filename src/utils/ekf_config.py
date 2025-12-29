#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Configurazione EKF - Parametri di tuning

Modifica questi valori per sperimentare con diversi settaggi del filtro
"""

import numpy as np

# ============================================================================
# POSIZIONE INIZIALE
# ============================================================================
# Posizione iniziale del robot [x, y, z] in metri
INITIAL_POSITION = [6.5, 3.0, 0.0]  # Default: origine
#INITIAL_POSITION = [0.0, 0.0, 0.0]  # Default: origine

# Velocità iniziale del robot [vx, vy, vz] in m/s
INITIAL_VELOCITY = [0.0, 0.0, 0.0]  # Default: fermo

# ============================================================================
# PARAMETRI PROCESSO (Q Matrix)
# ============================================================================
# Quanto il sistema può cambiare tra un passo e l'altro
# Valori ALTI = più fiducia nelle misure, meno nel modello
# Valori BASSI = più fiducia nel modello, meno nelle misure

# Incertezza posizione [m²]
Q_POSITION = 0.01  # Default: 0.01
# Più basso = traiettoria più smooth
# Più alto = segue meglio i fix USBL

# Incertezza velocità [m²/s²]
Q_VELOCITY = 0.5   # Default: 0.5
# Più basso = velocità più costante
# Più alto = velocità può cambiare rapidamente

# Incertezza velocità verticale [m²/s²]
Q_VELOCITY_Z = 0.1  # Default: 0.1 (solitamente più basso di xy)

# ============================================================================
# PARAMETRI MISURE (R Matrix)
# ============================================================================
# Quanto ti fidi delle misure dei sensori
# Valori ALTI = meno fiducia nel sensore
# Valori BASSI = più fiducia nel sensore

# Fattore moltiplicativo per varianza USBL misurata
R_USBL_FACTOR = 1.0  # Default: 1.5
# Aumenta per dare meno peso all'USBL (se misure rumorose)
# Diminuisci per dare più peso all'USBL (se misure accurate)

# Varianza depth sensor minima [m²]
R_DEPTH_MIN = 0.01  # Default: 0.01
# MS5837 è molto preciso, usa questo valore minimo

# ============================================================================
# COVARIANZA INIZIALE (P0)
# ============================================================================
# Quanto sei incerto sulla posizione/velocità iniziale

# Posizione iniziale
P0_POSITION_XY = 1.0   # [m²] - incertezza x,y iniziale
P0_POSITION_Z = 0.1    # [m²] - incertezza z iniziale

# Velocità iniziale
P0_VELOCITY = 0.01     # [m²/s²] - robot parte da fermo

# ============================================================================
# BIAS IMU
# ============================================================================
# Applica correzione bias IMU (raccomandato)
APPLY_BIAS_CORRECTION = True

# ============================================================================
# INTEGRAZIONE IMU
# ============================================================================
# Peso per integrazione accelerazione nel modello constant-velocity
# 0.0 = ignora completamente accelerazione (puro constant velocity)
# 1.0 = fiducia completa nell'accelerazione
ACC_INTEGRATION_WEIGHT = 0.1  # Default: 0.1

# Raccomandazioni:
# - Per IMU molto rumorosa: 0.0 - 0.05
# - Per IMU media qualità: 0.05 - 0.2
# - Per IMU ottima: 0.2 - 0.5

# ============================================================================
# PARAMETRI VISUALIZZAZIONE E RIFERIMENTI
# ============================================================================
# Posizione della BOA (transponder USBL) [x, y, z] in metri
# IMPORTANTE: Il range USBL è misurato da questa posizione!
BOA_POSITION = [0.0, 0.0, 0.0]  # [x, y, z] in metri
# Se hai spostato la BOA fisica, imposta qui la nuova posizione
# Esempio: BOA spostata 5m est, 3m nord
# BOA_POSITION = [5.0, 3.0, 0.0]

# Coordinate per visualizzazione 2D (solo grafico)
BOA_COORDINATES = (0.0, 0.0)
NORTH_MARKER_COORDINATES = (0.0, 8.0)
POOL_RADIUS = 8.0

# Output verboso
VERBOSE = True

# ============================================================================
# SCENARI PRE-CONFIGURATI
# ============================================================================

def get_smooth_config():
    """Configurazione per traiettoria molto smooth (basso rumore)"""
    return {
        'Q_POSITION': 0.001,
        'Q_VELOCITY': 0.1,
        'Q_VELOCITY_Z': 0.05,
        'R_USBL_FACTOR': 2.0,
        'ACC_INTEGRATION_WEIGHT': 0.05
    }

def get_responsive_config():
    """Configurazione per seguire da vicino le misure USBL"""
    return {
        'Q_POSITION': 0.1,
        'Q_VELOCITY': 1.0,
        'Q_VELOCITY_Z': 0.5,
        'R_USBL_FACTOR': 0.5,
        'ACC_INTEGRATION_WEIGHT': 0.2
    }

def get_usbl_only_config():
    """Configurazione che si fida quasi solo dell'USBL"""
    return {
        'Q_POSITION': 0.01,
        'Q_VELOCITY': 2.0,
        'Q_VELOCITY_Z': 1.0,
        'R_USBL_FACTOR': 0.3,
        'ACC_INTEGRATION_WEIGHT': 0.0  # Ignora IMU
    }

def get_balanced_config():
    """Configurazione bilanciata (default raccomandato)"""
    return {
        'Q_POSITION': 0.01,
        'Q_VELOCITY': 0.06,
        'Q_VELOCITY_Z': 0.1,
        'R_USBL_FACTOR': 0.01,
        'ACC_INTEGRATION_WEIGHT': 0.04
    }

# ============================================================================
# GUIDA AL TUNING
# ============================================================================
"""
GUIDA RAPIDA AL TUNING:

1. PROBLEMA: Traiettoria troppo rumorosa (zig-zag)
   SOLUZIONE: 
   - Diminuisci Q_POSITION (es. 0.01 -> 0.001)
   - Diminuisci Q_VELOCITY (es. 0.5 -> 0.1)
   - Aumenta R_USBL_FACTOR (es. 1.5 -> 3.0)

2. PROBLEMA: Traiettoria non segue i fix USBL
   SOLUZIONE:
   - Aumenta Q_POSITION (es. 0.01 -> 0.1)
   - Aumenta Q_VELOCITY (es. 0.5 -> 2.0)
   - Diminuisci R_USBL_FACTOR (es. 1.5 -> 0.5)

3. PROBLEMA: Robot "vola via" tra fix USBL
   SOLUZIONE:
   - Diminuisci ACC_INTEGRATION_WEIGHT (es. 0.1 -> 0.0)
   - Aumenta Q_VELOCITY per permettere correzioni maggiori
   - Verifica bias IMU

4. PROBLEMA: Z (profondità) impazzisce
   SOLUZIONE:
   - Verifica sign convention del depth sensor
   - Diminuisci Q_VELOCITY_Z (es. 0.1 -> 0.01)
   - Diminuisci R_DEPTH_MIN (es. 0.01 -> 0.001)

5. PROBLEMA: Velocità stimata irrealistica
   SOLUZIONE:
   - Aumenta R_USBL_FACTOR per smoothing
   - Diminuisci Q_VELOCITY
   - Imposta ACC_INTEGRATION_WEIGHT = 0.0

BEST PRACTICES:
- Parti con get_balanced_config()
- Modifica UN parametro alla volta
- Confronta errore RMS range USBL prima/dopo
- Se RMS < 1m, il tuning è buono
- Se RMS > 3m, c'è un problema fondamentale

PARAMETRI CHIAVE PER QUICK TUNING:
1. Q_VELOCITY: controlla quanto può cambiare la velocità
2. R_USBL_FACTOR: controlla peso delle misure USBL
3. ACC_INTEGRATION_WEIGHT: controlla influenza IMU
"""

# ============================================================================
# APPLICAZIONE CONFIGURAZIONE
# ============================================================================

# Scegli la configurazione da usare:
# ACTIVE_CONFIG = get_smooth_config()
# ACTIVE_CONFIG = get_responsive_config()
# ACTIVE_CONFIG = get_usbl_only_config()
ACTIVE_CONFIG = get_balanced_config()  # <-- DEFAULT

# Applica configurazione attiva
if ACTIVE_CONFIG:
    Q_POSITION = ACTIVE_CONFIG['Q_POSITION']
    Q_VELOCITY = ACTIVE_CONFIG['Q_VELOCITY']
    Q_VELOCITY_Z = ACTIVE_CONFIG['Q_VELOCITY_Z']
    R_USBL_FACTOR = ACTIVE_CONFIG['R_USBL_FACTOR']
    ACC_INTEGRATION_WEIGHT = ACTIVE_CONFIG['ACC_INTEGRATION_WEIGHT']
    
    print("="*70)
    print("CONFIGURAZIONE EKF ATTIVA")
    print("="*70)
    print(f"\nQ_POSITION:             {Q_POSITION}")
    print(f"Q_VELOCITY:             {Q_VELOCITY}")
    print(f"Q_VELOCITY_Z:           {Q_VELOCITY_Z}")
    print(f"R_USBL_FACTOR:          {R_USBL_FACTOR}")
    print(f"ACC_INTEGRATION_WEIGHT: {ACC_INTEGRATION_WEIGHT}")
    print("="*70)