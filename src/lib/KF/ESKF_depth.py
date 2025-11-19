#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import time
import matplotlib.pyplot as plt

# -------------------------- Quaternion utils --------------------------

def skew(w):
    wx, wy, wz = w
    return np.array([[ 0, -wz,  wy],
                     [ wz,  0, -wx],
                     [-wy, wx,   0]], dtype=float)

def q_mul(q1, q2):
    # Hamilton product, q = [w, x, y, z]
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def q_exp(dw):
    # small rotation vector -> unit quaternion
    theta = np.linalg.norm(dw)
    if theta < 1e-12:
        return np.array([1.0, 0.5*dw[0], 0.5*dw[1], 0.5*dw[2]], dtype=float)
    axis = dw / theta
    s = math.sin(0.5*theta)
    return np.array([math.cos(0.5*theta), *(s*axis)], dtype=float)

def q_normalize(q):
    n = np.linalg.norm(q)
    return q if n == 0 else (q / n)

def R_from_q(q):
    # Body->World rotation
    w,x,y,z = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1-2*(x*x+z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1-2*(x*x+y*y)]
    ], dtype=float)

def yaw_from_q(q):
    # ZYX yaw from quaternion
    w,x,y,z = q
    s = 2.0*(w*z + x*y)
    c = 1.0 - 2.0*(y*y + z*z)
    return math.atan2(s, c)

def rotation_to_quaternion(R):
    m00, m01, m02 = R[0]; m10, m11, m12 = R[1]; m20, m21, m22 = R[2]
    tr = m00 + m11 + m22
    if tr > 0:
        S = math.sqrt(tr+1.0)*2
        w = 0.25*S
        x = (m21 - m12)/S
        y = (m02 - m20)/S
        z = (m10 - m01)/S
    elif (m00 > m11) and (m00 > m22):
        S = math.sqrt(1.0 + m00 - m11 - m22)*2
        w = (m21 - m12)/S
        x = 0.25*S
        y = (m01 + m10)/S
        z = (m02 + m20)/S
    elif m11 > m22:
        S = math.sqrt(1.0 + m11 - m00 - m22)*2
        w = (m02 - m20)/S
        x = (m01 + m10)/S
        y = 0.25*S
        z = (m12 + m21)/S
    else:
        S = math.sqrt(1.0 + m22 - m00 - m11)*2
        w = (m10 - m01)/S
        x = (m02 + m20)/S
        y = (m12 + m21)/S
        z = 0.25*S
    return q_normalize(np.array([w,x,y,z], dtype=float))

# -------------------------- ESKF (15-state error) --------------------------

class ESKF15:
    """
    Nominal state: p(3), v(3), q(4), bg(3), ba(3)
    Error-state:   [dp dv dtheta dbg dba] -> 15
    """
    def __init__(self, Qc, R_pos, R_yaw, g_world=np.array([[0],[0],[-9.81]]), dt=0.01):
        self.p = np.zeros((3,1))
        self.v = np.zeros((3,1))
        self.q = np.array([1,0,0,0], dtype=float)
        self.bg = np.zeros((3,1))
        self.ba = np.zeros((3,1))

        # Start with large uncertainty (trust early measurements)
        self.P = np.diag([
            5.0,5.0,5.0,                        # pos (m^2)
            1.0,1.0,1.0,                        # vel (m/s)^2
            np.deg2rad(10)**2,                  # roll (rad^2)
            np.deg2rad(10)**2,                  # pitch
            np.deg2rad(15)**2,                  # yaw
            np.deg2rad(0.1)**2,                 # bgx (rad/s)^2
            np.deg2rad(0.1)**2,                 # bgy
            np.deg2rad(0.1)**2,                 # bgz
            0.1**2, 0.1**2, 0.1**2              # bax, bay, baz (m/s^2)^2
        ])

        self.Qc = Qc
        self.R_pos = R_pos
        self.R_yaw = R_yaw
        self.R_depth = np.array([[0.02**2]])  # tune as needed
        self.R_range = np.array([[0.25**2]])  # conservative vs. 0.15 m noise
        self.g = g_world
        self.dt = dt

    def propagate(self, gyro_meas, acc_meas):
        dt = self.dt
        omega_b = (gyro_meas - self.bg).reshape(3)
        acc_b   = (acc_meas  - self.ba).reshape(3)

        Rwb = R_from_q(self.q)
        a_w = (Rwb @ acc_b.reshape(3,1)) + self.g

        self.p = self.p + self.v*dt + 0.5*a_w*(dt**2)
        self.v = self.v + a_w*dt
        self.q = q_normalize(q_mul(self.q, q_exp(omega_b*dt)))

        # Error-state dynamics (discrete approx)
        Rwb = R_from_q(self.q)   # after update (midpoint approx)
        Om  = skew(omega_b)
        Ab  = skew(acc_b)

        Fc = np.zeros((15,15))
        Fc[0:3, 3:6]    = np.eye(3)
        Fc[3:6, 6:9]    = - Rwb @ Ab
        Fc[3:6, 12:15]  = - Rwb
        Fc[6:9, 6:9]    = - Om
        Fc[6:9, 9:12]   = - np.eye(3)

        Gc = np.zeros((15,12))
        Gc[6:9, 0:3]    = - np.eye(3)  # gyro noise -> dtheta
        Gc[3:6, 3:6]    = Rwb          # accel noise (world) -> dv
        Gc[9:12, 6:9]   = np.eye(3)    # gyro bias RW
        Gc[12:15, 9:12] = np.eye(3)    # accel bias RW

        Fd = np.eye(15) + Fc*dt
        Qd = Gc @ self.Qc @ Gc.T * dt
        self.P = Fd @ self.P @ Fd.T + Qd

    def update_pos(self, z_pos):
        H = np.zeros((3,15)); H[:,0:3] = np.eye(3)
        r = z_pos.reshape(3,1) - self.p
        S = H @ self.P @ H.T + self.R_pos
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r
        self._inject(dx)
        self.P = (np.eye(15) - K @ H) @ self.P

    def update_yaw(self, z_yaw):
        def wrap(a): return (a + np.pi) % (2*np.pi) - np.pi
        y_true = yaw_from_q(self.q)
        r = np.array([[wrap(z_yaw - y_true)]])
        H = np.zeros((1,15))
        H[0, 6+2] = 1.0  # ∂yaw/∂δθz (small-angle)
        S = H @ self.P @ H.T + self.R_yaw
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r
        self._inject(dx)
        self.P = (np.eye(15) - K @ H) @ self.P

    def update_depth(self, z_depth):
        H = np.zeros((1,15)); H[0,2] = 1.0
        r = np.array([[z_depth]]) - self.p[2:3,:]
        S = H @ self.P @ H.T + self.R_depth
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ r
        self._inject(dx); self.P = (np.eye(15) - K@H) @ self.P

    def update_range(self, z_r, p_buoy):
        dp = (self.p - p_buoy.reshape(3,1))
        rhat = float(np.linalg.norm(dp))
        if rhat < 1e-6:
            return
        H = np.zeros((1,15))
        H[0,0:3] = (dp.T / rhat)
        r = np.array([[z_r - rhat]])
        S = H @ self.P @ H.T + self.R_range
        # simple gate
        maha = float(r.T @ np.linalg.inv(S) @ r)
        if maha < 16.0:  # ~4-sigma
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ r
            self._inject(dx); self.P = (np.eye(15) - K@H) @ self.P

    def _inject(self, dx):
        dx = dx.reshape(15)
        self.p  += dx[0:3].reshape(3,1)
        self.v  += dx[3:6].reshape(3,1)
        self.q   = q_normalize(q_mul(self.q, q_exp(dx[6:9])))
        self.bg += dx[9:12].reshape(3,1)
        self.ba += dx[12:15].reshape(3,1)