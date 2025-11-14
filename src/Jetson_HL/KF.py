#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import time
from RMatrix import Rxyz  # body->world rotation

# Plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def A_m(roll, pitch, yaw, T):
    """Option A: velocity in WORLD, acceleration in BODY."""
    A = np.eye(9)
    R = Rxyz(roll, pitch, yaw)  # body -> world
    A[0:3, 3:6] = T * np.eye(3)              # ṗ = v
    A[0:3, 6:9] = 0.5 * (T**2) * R           # p integrates a_world
    A[3:6, 6:9] = T * R                      # v̇ = a_world
    return A

def B_m(n, m, T):
    """Process-input matrix (accel process noise mapped into p & v)."""
    B = np.zeros((n, m))
    B[0:3, 0:3] = (T**2)/2.0 * np.eye(3)
    B[3:6, 0:3] = T * np.eye(3)
    return B

class KF(object):
    """9-state KF: x=[p_w(3), v_w(3), a_b(3)]."""
    def __init__(self, Q, R1, R2, C1, C2, A=A_m, B=B_m, T=0.05, n=9, m=3):
        self.n, self.m, self.T = n, m, T
        self.Pkk = np.eye(n)
        self.x = np.zeros((n,1))
        self.A, self.B = A, B
        self.C1, self.C2 = C1, C2
        self.Q, self.R1, self.R2 = Q, R1, R2
        self.t = time.time()

    def setX(self, X):
        X = np.asarray(X)
        if X.shape != (self.n,1):
            raise ValueError("X must be shape ({},1)".format(self.n))
        self.x = X

    def getX(self):
        return self.x

    def update(self, yy, roll, pitch, yaw, queue, dt_override=None):
        """If dt_override is given, use it; otherwise fall back to wall-clock dt."""
        if dt_override is None:
            t_iter = time.time()
            tc = t_iter - self.t
            self.t = t_iter
        else:
            tc = float(dt_override)

        A = self.A(roll, pitch, yaw, tc)
        B = self.B(self.n, self.m, tc)
        Pk1k = A @ self.Pkk @ A.T + B @ self.Q @ B.T
        Xk1k = A @ self.x

        C, R = (self.C1, self.R1) if queue else (self.C2, self.R2)

        yy = np.asarray(yy)
        if yy.ndim == 1:
            yy = yy.reshape((-1,1))

        S = C @ Pk1k @ C.T + R
        K = Pk1k @ C.T @ np.linalg.inv(S)
        self.x = Xk1k + K @ (yy - C @ Xk1k)
        self.Pkk = (np.eye(self.n) - K @ C) @ Pk1k


# ---------------------- Simulation + Plots ----------------------
if __name__ == "__main__":
    np.random.seed(2)

    # Covariances
    Q = np.eye(3) * 0.05          # process noise on acceleration (world)
    R1 = np.eye(3) * 0.02         # accel (body) measurement noise
    R2 = np.eye(6) * 0.5          # accel + position
    R2[3:6, 3:6] *= 0.2           # tighter on position

    # Measurement matrices
    C1 = np.array([               # accel (a_body)
        [0,0,0, 0,0,0, 1,0,0],
        [0,0,0, 0,0,0, 0,1,0],
        [0,0,0, 0,0,0, 0,0,1],
    ], dtype=float)

    C2 = np.array([               # accel + world position
        [0,0,0, 0,0,0, 1,0,0],
        [0,0,0, 0,0,0, 0,1,0],
        [0,0,0, 0,0,0, 0,0,1],
        [1,0,0, 0,0,0, 0,0,0],
        [0,1,0, 0,0,0, 0,0,0],
        [0,0,1, 0,0,0, 0,0,0],
    ], dtype=float)

    kf = KF(Q, R1, R2, C1, C2)

    # Circle params
    dt = 0.05          # << fixed sim step
    steps = 400
    RADIUS = 10.0
    OMEGA = 0.15

    # Sensor noise (sim)
    sigma_acc = 0.03
    sigma_pos = 0.15
    sigma_yaw = np.deg2rad(1.0)  # yaw measurement noise (for plots)

    # Storage
    ts = []
    xs_true, ys_true, zs_true = [], [], []
    xs_est, ys_est, zs_est = [], [], []
    xs_meas, ys_meas, zs_meas = [], [], []
    ax_meas, ay_meas, az_meas = [], [], []
    yaw_true_list, yaw_meas_list = [], []

    # Init
    kf.setX(np.zeros((9,1)))
    t = 0.0

    for k in range(steps):
        theta = OMEGA * t

        # Truth (world)
        p_true = np.array([[RADIUS * math.cos(theta)],
                           [RADIUS * math.sin(theta)],
                           [0.0]])
        a_world = np.array([[-RADIUS * (OMEGA ** 2) * math.cos(theta)],
                            [-RADIUS * (OMEGA ** 2) * math.sin(theta)],
                            [0.0]])

        # Orientation (body x tangent)
        roll = 0.0
        pitch = 0.0
        yaw_true = theta + math.pi / 2.0

        # Body-frame accel measurement
        Rwb = Rxyz(roll, pitch, yaw_true)
        a_body_true = Rwb.T @ a_world
        a_meas = a_body_true + np.random.normal(0.0, sigma_acc, size=(3,1))

        # Yaw "measurement" (for MSE plot only)
        yaw_meas = yaw_true + np.random.normal(0.0, sigma_yaw)

        # Position fix every 5 steps
        have_pos = (k % 5 == 0)
        if have_pos:
            p_meas = p_true + np.random.normal(0.0, sigma_pos, size=(3,1))
            yy = np.vstack([a_meas, p_meas])
            kf.update(yy, roll=roll, pitch=pitch, yaw=yaw_true, queue=False, dt_override=dt)
            pxm, pym, pzm = p_meas[:,0]
        else:
            kf.update(a_meas, roll=roll, pitch=pitch, yaw=yaw_true, queue=True, dt_override=dt)
            pxm, pym, pzm = (np.nan, np.nan, np.nan)

        state = kf.getX()
        px, py, pz = state[0,0], state[1,0], state[2,0]

        # log
        ts.append(t)
        xs_true.append(p_true[0,0]); ys_true.append(p_true[1,0]); zs_true.append(p_true[2,0])
        xs_est.append(px); ys_est.append(py); zs_est.append(pz)
        xs_meas.append(pxm); ys_meas.append(pym); zs_meas.append(pzm)
        ax_meas.append(a_meas[0,0]); ay_meas.append(a_meas[1,0]); az_meas.append(a_meas[2,0])
        yaw_true_list.append(yaw_true); yaw_meas_list.append(yaw_meas)

        t += dt
        time.sleep(0.0)  # run fast; we integrate with dt_override

    # -------------------- Plots (same as before) --------------------
    # 3D trajectory
    fig1 = plt.figure()
    ax3d = fig1.add_subplot(111, projection='3d')
    ax3d.plot(xs_true, ys_true, zs_true, linestyle='--', label="Ground truth")
    ax3d.plot(xs_est,  ys_est,  zs_est,               label="KF estimate")
    ax3d.scatter(xs_meas, ys_meas, zs_meas, s=12, label="Noisy pos (when available)")
    ax3d.set_title("3D Trajectory")
    ax3d.set_xlabel("X [m]"); ax3d.set_ylabel("Y [m]"); ax3d.set_zlabel("Z [m]")
    ax3d.legend(loc="best")

    # Measurements over time
    fig2 = plt.figure()
    ax2 = fig2.add_subplot(111)
    ax2.plot(ts, ax_meas, label="ax (meas)")
    ax2.plot(ts, ay_meas, label="ay (meas)")
    ax2.plot(ts, az_meas, label="az (meas)")
    ax2.set_title("Acceleration Measurements (Body Frame)")
    ax2.set_xlabel("Time [s]"); ax2.set_ylabel("Acceleration [m/s²]")
    ax2.legend(loc="best")

    # Position over time
    fig3 = plt.figure()
    ax3 = fig3.add_subplot(311)
    ax3.plot(ts, xs_true, label="x true")
    ax3.plot(ts, xs_est,  label="x est")
    ax3.scatter(ts, xs_meas, s=8, label="x meas")
    ax3.set_ylabel("X [m]"); ax3.legend(loc="best")

    ax4 = fig3.add_subplot(312)
    ax4.plot(ts, ys_true, label="y true")
    ax4.plot(ts, ys_est,  label="y est")
    ax4.scatter(ts, ys_meas, s=8, label="y meas")
    ax4.set_ylabel("Y [m]"); ax4.legend(loc="best")

    ax5 = fig3.add_subplot(313)
    ax5.plot(ts, zs_true, label="z true")
    ax5.plot(ts, zs_est,  label="z est")
    ax5.scatter(ts, zs_meas, s=8, label="z meas")
    ax5.set_xlabel("Time [s]"); ax5.set_ylabel("Z [m]"); ax5.legend(loc="best")
    fig3.suptitle("Position Over Time")

    # Running MSEs
    true_pos = np.vstack([xs_true, ys_true, zs_true]).T
    est_pos  = np.vstack([xs_est,  ys_est,  zs_est ]).T
    pos_err = est_pos - true_pos
    pos_se  = pos_err**2
    pos_mse_running = np.cumsum(pos_se, axis=0) / (np.arange(len(ts))[:,None] + 1)
    pos_err_norm = np.linalg.norm(pos_err, axis=1)
    pos_mse_norm_running = np.cumsum(pos_err_norm**2) / (np.arange(len(ts)) + 1)

    yaw_err = (np.unwrap(yaw_meas_list) - np.unwrap(yaw_true_list) + np.pi) % (2*np.pi) - np.pi
    yaw_mse_running = np.cumsum(yaw_err**2) / (np.arange(len(ts)) + 1)

    fig4 = plt.figure()
    axm = fig4.add_subplot(111)
    axm.plot(ts, pos_mse_running[:,0], label="MSE x")
    axm.plot(ts, pos_mse_running[:,1], label="MSE y")
    axm.plot(ts, pos_mse_running[:,2], label="MSE z")
    axm.plot(ts, pos_mse_norm_running, label="MSE |pos| (norm)")
    axm.set_title("Running MSE of Position")
    axm.set_xlabel("Time [s]"); axm.set_ylabel("MSE [m²]")
    axm.legend(loc="best")

    fig5 = plt.figure()
    axr = fig5.add_subplot(111)
    axr.plot(ts, yaw_mse_running)
    axr.set_title("Running MSE of Yaw Measurement (rad²)")
    axr.set_xlabel("Time [s]"); axr.set_ylabel("MSE [rad²]")

    # Make 3D axes equal-ish
    def _lims(vals):
        vals = [v for v in vals if not (isinstance(v,float) and np.isnan(v))]
        if not vals: return (0.0, 1.0)
        vmin, vmax = min(vals), max(vals)
        if vmin == vmax: vmax = vmin + 1.0
        return vmin, vmax
    xlim, ylim, zlim = _lims(xs_true + xs_est + xs_meas), \
                       _lims(ys_true + ys_est + ys_meas), \
                       _lims(zs_true + zs_est + zs_meas)
    mid = lambda a, b: (a + b) / 2.0
    xr, yr, zr = xlim[1]-xlim[0], ylim[1]-ylim[0], zlim[1]-zlim[0]
    r = max(xr, yr, zr)
    cx, cy, cz = mid(*xlim), mid(*ylim), mid(*zlim)
    ax3d.set_xlim(cx - r/2, cx + r/2)
    ax3d.set_ylim(cy - r/2, cy + r/2)
    ax3d.set_zlim(cz - r/2, cz + r/2)

    plt.show()
