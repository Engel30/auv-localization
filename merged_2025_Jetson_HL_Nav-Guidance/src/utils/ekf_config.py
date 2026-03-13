#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Configuration — tuning parameters and visualization settings.

Edit the values below to experiment with different filter settings.
"""

import numpy as np

# ============================================================================
# INITIAL STATE
# ============================================================================
# Initial robot position [x, y, z] in metres
#INITIAL_POSITION = [0.80, -0.80, -0.35]
INITIAL_POSITION = [6.5, 3.0, 0.0]
# INITIAL_POSITION = [0.0, 0.0, 0.0]

# Initial robot velocity [vx, vy, vz] in m/s
INITIAL_VELOCITY = [0.0, 0.0, 0.0]   # starts from rest

# ============================================================================
# TIME WINDOW FILTER
# ============================================================================
# Restrict processing to a specific time range of the recording.
# Uses the timestamp_rel column of the raw CSV files.
# Set to None to use the full recording.
#
#   T_START: discard all data before this time [s]  (None = beginning)
#   T_END:   discard all data after  this time [s]  (None = end)
#
# Example — keep only the 30-90 s window:
#   T_START = 30.0
#   T_END   = 90.0
T_START = None
T_END   = 350 #None

# ============================================================================
# PROCESS NOISE (Q Matrix)
# ============================================================================
# Controls how much the state can change between steps.
# HIGH values → trust measurements more, model less
# LOW  values → trust model more, measurements less

# Position uncertainty [m²]
Q_POSITION = 0.01
# Lower  → smoother trajectory
# Higher → follows USBL fixes more closely

# XY velocity uncertainty [m²/s²]
Q_VELOCITY = 0.5
# Lower  → more constant velocity
# Higher → velocity can change rapidly

# Z (vertical) velocity uncertainty [m²/s²]
Q_VELOCITY_Z = 0.1   # usually lower than XY

# ============================================================================
# MEASUREMENT NOISE (R Matrix)
# ============================================================================
# Controls how much the filter trusts each sensor.
# HIGH values → less trust in the sensor
# LOW  values → more trust in the sensor

# Multiplicative factor on the measured USBL range variance
R_USBL_FACTOR = 1.0
# Increase to down-weight noisy USBL fixes
# Decrease to follow USBL more tightly

# Minimum depth sensor variance [m²]
R_DEPTH_MIN = 0.01   # MS5837 is very accurate; this is already a floor

# ============================================================================
# INITIAL COVARIANCE (P0)
# ============================================================================
# Uncertainty about the initial state

P0_POSITION_XY = 1.0    # [m²]    — initial XY position uncertainty
P0_POSITION_Z  = 0.1    # [m²]    — initial Z  position uncertainty
P0_VELOCITY    = 0.01   # [m²/s²] — starts from rest, low uncertainty

# ============================================================================
# IMU BIAS CORRECTION
# ============================================================================
APPLY_BIAS_CORRECTION = True   # recommended — uses imu_calibration.json

# ============================================================================
# IMU ACCELERATION INTEGRATION
# ============================================================================
# Weight applied to acceleration in the constant-velocity prediction step.
# 0.0 = ignore acceleration completely (pure constant-velocity)
# 1.0 = full trust in the acceleration reading
ACC_INTEGRATION_WEIGHT = 0.1

# Recommendations:
#   Very noisy IMU  : 0.00 – 0.05
#   Average IMU     : 0.05 – 0.20
#   High-quality IMU: 0.20 – 0.50

# ============================================================================
# VISUALIZATION — TRANSPONDER AND MAP GEOMETRY
# ============================================================================
# Physical position of the USBL transponder [x, y, z] in metres.
# IMPORTANT: USBL range is measured from this position!
BOA_POSITION = [0.0, 0.0, 0.0]
# If the transponder has been moved, update this value.
# Example: 5 m East, 3 m North → BOA_POSITION = [5.0, 3.0, 0.0]

# 2D display coordinates (for the plot only — keep in sync with BOA_POSITION)
BOA_COORDINATES          = (0.0, 0.0)
NORTH_MARKER_COORDINATES = (0.0, 8.0)
POOL_RADIUS              = 8.0        # pool circle radius [m]

# Verbose terminal output
VERBOSE = True

# ============================================================================
# VISUALIZATION — 2D MAP SHAPES AND REFERENCE POINTS
# ============================================================================
# All coordinates in metres, same reference frame as the EKF state.

# --- Rectangle obstacle / structure / zone of interest ---
# Centred at RECT_CENTER, then rotated by RECT_ANGLE degrees counterclockwise.
# Set SHOW_RECTANGLE = False to hide without deleting the parameters.
SHOW_RECTANGLE = True
RECT_CENTER    = [-8.0, 0.0]    # centre [x, y] in metres
RECT_WIDTH     = 2.5            # width  along local X axis before rotation [m]
RECT_HEIGHT    = 1.5            # height along local Y axis before rotation [m]
RECT_ANGLE     = 0.0            # rotation [deg], counterclockwise from East
RECT_COLOR     = 'orange'       # matplotlib colour name or hex string
RECT_LABEL     = 'Obstacle'     # legend label

# --- Reference points ---
# Marked on the 2D map with a diamond marker and a text label.
# Each entry: [x_metres, y_metres, 'Label string']
REFERENCE_POINTS = [
    [-3.0,  7.5, "Ref A"],
    [-3.0, -7.5, "Ref B"],
    [ 3.0,  7.5, "Ref C"],
]

# ============================================================================
# PRE-CONFIGURED TUNING SCENARIOS
# ============================================================================

def get_smooth_config():
    """Very smooth trajectory — low process noise, high measurement noise."""
    return {
        'Q_POSITION': 0.001,
        'Q_VELOCITY': 0.1,
        'Q_VELOCITY_Z': 0.05,
        'R_USBL_FACTOR': 2.0,
        'ACC_INTEGRATION_WEIGHT': 0.05
    }

def get_responsive_config():
    """Follows USBL fixes closely — high process noise, low measurement noise."""
    return {
        'Q_POSITION': 0.1,
        'Q_VELOCITY': 1.0,
        'Q_VELOCITY_Z': 0.5,
        'R_USBL_FACTOR': 0.5,
        'ACC_INTEGRATION_WEIGHT': 0.2
    }

def get_usbl_only_config():
    """Trusts USBL almost exclusively — IMU acceleration ignored."""
    return {
        'Q_POSITION': 0.01,
        'Q_VELOCITY': 2.0,
        'Q_VELOCITY_Z': 1.0,
        'R_USBL_FACTOR': 0.3,
        'ACC_INTEGRATION_WEIGHT': 0.0
    }

def get_balanced_config():
    """Balanced default — recommended starting point."""
    return {
        'Q_POSITION': 0.01,
        'Q_VELOCITY': 0.06,
        'Q_VELOCITY_Z': 0.1,
        'R_USBL_FACTOR': 0.01,
        'ACC_INTEGRATION_WEIGHT': 0.04
    }

# ============================================================================
# TUNING GUIDE
# ============================================================================
"""
QUICK TUNING GUIDE:

1. PROBLEM: Trajectory too noisy / zig-zag
   FIX:
   - Decrease Q_POSITION   (e.g. 0.01 → 0.001)
   - Decrease Q_VELOCITY   (e.g. 0.5  → 0.1)
   - Increase R_USBL_FACTOR(e.g. 1.5  → 3.0)

2. PROBLEM: Trajectory does not follow USBL fixes
   FIX:
   - Increase Q_POSITION   (e.g. 0.01 → 0.1)
   - Increase Q_VELOCITY   (e.g. 0.5  → 2.0)
   - Decrease R_USBL_FACTOR(e.g. 1.5  → 0.5)

3. PROBLEM: Robot drifts far between USBL fixes
   FIX:
   - Decrease ACC_INTEGRATION_WEIGHT (e.g. 0.1 → 0.0)
   - Increase Q_VELOCITY to allow larger corrections
   - Check IMU bias calibration

4. PROBLEM: Z (depth) diverges
   FIX:
   - Check sign convention of the depth sensor
   - Decrease Q_VELOCITY_Z (e.g. 0.1  → 0.01)
   - Decrease R_DEPTH_MIN  (e.g. 0.01 → 0.001)

5. PROBLEM: Estimated velocity is unrealistic
   FIX:
   - Increase R_USBL_FACTOR for smoother updates
   - Decrease Q_VELOCITY
   - Set ACC_INTEGRATION_WEIGHT = 0.0

BEST PRACTICES:
- Start with get_balanced_config()
- Change ONE parameter at a time
- Compare USBL range RMS error before/after
- RMS < 1 m → good tuning
- RMS > 3 m → something is fundamentally wrong

TIME WINDOW FILTER (T_START / T_END):
- Use to skip a noisy warm-up phase at the beginning
- Use to cut off a messy ending (e.g. surfacing)
- The filter trims imu, usbl and depth CSVs simultaneously so
  the EKF always receives consistent sensor data
"""

# ============================================================================
# APPLY ACTIVE CONFIGURATION
# ============================================================================

# Choose the configuration to use:
# ACTIVE_CONFIG = get_smooth_config()
# ACTIVE_CONFIG = get_responsive_config()
# ACTIVE_CONFIG = get_usbl_only_config()
ACTIVE_CONFIG = get_balanced_config()   # <-- DEFAULT

if ACTIVE_CONFIG:
    Q_POSITION             = ACTIVE_CONFIG['Q_POSITION']
    Q_VELOCITY             = ACTIVE_CONFIG['Q_VELOCITY']
    Q_VELOCITY_Z           = ACTIVE_CONFIG['Q_VELOCITY_Z']
    R_USBL_FACTOR          = ACTIVE_CONFIG['R_USBL_FACTOR']
    ACC_INTEGRATION_WEIGHT = ACTIVE_CONFIG['ACC_INTEGRATION_WEIGHT']

    print("="*70)
    print("ACTIVE EKF CONFIGURATION")
    print("="*70)
    print(f"\nQ_POSITION:             {Q_POSITION}")
    print(f"Q_VELOCITY:             {Q_VELOCITY}")
    print(f"Q_VELOCITY_Z:           {Q_VELOCITY_Z}")
    print(f"R_USBL_FACTOR:          {R_USBL_FACTOR}")
    print(f"ACC_INTEGRATION_WEIGHT: {ACC_INTEGRATION_WEIGHT}")
    if T_START is not None or T_END is not None:
        print(f"\nTime window:            [{T_START}, {T_END}] s")
    print("="*70)
