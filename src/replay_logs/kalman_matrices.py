# Calibrated matrices for Kalman Filter
import numpy as np

# ======================================================================
# MEASUREMENT NOISE MATRICES (R)
# ======================================================================

# Accelerometer measurement noise [ax, ay, az]
R_accelerometer = np.diag([0.00013573751128173603, 0.00011423643922768472, 0.00013666674076541824])

# Gyroscope measurement noise [wx, wy, wz] (angular velocity)
R_gyroscope = np.diag([3.914324631790724e-06, 5.15015932947352e-06, 2.372242510291903e-06])

# Orientation measurement noise [roll, pitch, yaw]
R_orientation = np.diag([0.0031391220099396874, 0.000743364790155149, 0.0188340105860731])

# Free acceleration measurement noise (gravity compensated)
R_free_acceleration = np.diag([0.00016682294916239618, 0.00023199424705483193, 0.0001340234597547104])

# Magnetometer measurement noise
R_magnetometer = np.diag([1.6621526405988674e-06, 1.730263363401796e-06, 2.8655127738024807e-06])

# Depth sensor measurement noise
R_depth = np.array([[8.441410479471646e-06]])

# USBL measurement noise (typical values - calibrate with real data)
R_usbl = np.diag([1.0, 1.0, 0.5])

# ======================================================================
# PROCESS NOISE MATRICES (Q)
# ======================================================================

# Process noise from acceleration [ax, ay, az]
Q_acceleration = np.diag([0.00013573751128173603, 0.00011423643922768472, 0.00013666674076541824])

# Process noise from angular velocity [wx, wy, wz]
Q_angular = np.diag([3.914324631790724e-06, 5.15015932947352e-06, 2.372242510291903e-06])

# ======================================================================
# KALMAN FILTER MATRICES (for your existing KF)
# ======================================================================

# Process noise (use acceleration-based Q)
Q = np.diag([0.00013573751128173603, 0.00011423643922768472, 0.00013666674076541824])

# R1: IMU measurement noise (free acceleration)
R1 = np.diag([0.00016682294916239618, 0.00023199424705483193, 0.0001340234597547104])

# R2: USBL measurement noise
R2 = np.diag([0.004, 0.004, 0.004])

# R3: Depth sensor measurement noise
R3 = np.array([[8.441410479471646e-06]])

# ======================================================================
# OBSERVATION MATRICES (C)
# ======================================================================

# C1: IMU observes acceleration (state vector indices 6:9)
C1 = np.zeros((3, 9))
C1[0:3, 6:9] = np.eye(3)

# C2: USBL observes position (state vector indices 0:3)
C2 = np.zeros((3, 9))
C2[0:3, 0:3] = np.eye(3)

# C3: Depth sensor observes Z position (state vector index 2)
C3 = np.zeros((1, 9))
C3[0, 2] = 1.0

# ======================================================================
# EXAMPLE: Initialize KF_profond.py Kalman Filter
# ======================================================================

# from KF_profond import KF
# kf = KF(Q=Q, R1=R1, R2=R2, R3=R3, C1=C1, C2=C2, C3=C3, T=0.05)
