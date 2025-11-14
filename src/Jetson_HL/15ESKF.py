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

        # Start with *large* uncertainty (trust early measurements)
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

        # Error-state dynamics
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
        # gyro noise -> dtheta
        Gc[6:9, 0:3]    = - np.eye(3)
        # accel noise (world) -> dv
        Gc[3:6, 3:6]    = Rwb
        # gyro bias RW
        Gc[9:12, 6:9]   = np.eye(3)
        # accel bias RW
        Gc[12:15, 9:12] = np.eye(3)

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
        y_true = yaw_from_q(self.q)
        r = (z_yaw - y_true + np.pi) % (2*np.pi) - np.pi
        H = np.zeros((1,15))
        H[0, 6+2] = 1.0  # ∂yaw/∂δθz (small-angle)
        S = H @ self.P @ H.T + self.R_yaw
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K * r
        self._inject(dx)
        self.P = (np.eye(15) - K @ H) @ self.P

    def _inject(self, dx):
        dx = dx.reshape(15)
        self.p  += dx[0:3].reshape(3,1)
        self.v  += dx[3:6].reshape(3,1)
        self.q   = q_normalize(q_mul(self.q, q_exp(dx[6:9])))
        self.bg += dx[9:12].reshape(3,1)
        self.ba += dx[12:15].reshape(3,1)

# -------------------------- Simulation: tilted circle & warm-start --------------------------

def simulate_tilted_circle():
    np.random.seed(7)

    # Sim params
    dt    = 0.01
    steps = 6000      # 60 s
    Rrad  = 10.0
    OMEGA = 0.12
    tilt  = math.radians(20.0)  # rotate world about X by +20°

    # Noise/bias
    sigma_g  = np.deg2rad(0.05)   # rad/sqrt(s)
    sigma_a  = 0.05               # m/s^2/sqrt(s)
    sigma_bg = np.deg2rad(0.002)  # rad/s/sqrt(s)
    sigma_ba = 0.005              # m/s^2/sqrt(s)
    Qc = np.diag([sigma_g**2]*3 + [sigma_a**2]*3 + [sigma_bg**2]*3 + [sigma_ba**2]*3)

    R_pos = np.diag([0.10**2, 0.10**2, 0.10**2])    # USBL ~ 10 cm std
    R_yaw = np.array([[np.deg2rad(2.0)**2]])        # 2 deg

    pos_period = 0.5   # 2 Hz
    yaw_period = 0.1   # 10 Hz
    k_pos = int(round(pos_period/dt))
    k_yaw = int(round(yaw_period/dt))

    # True sensor biases
    bg_true = np.array([[np.deg2rad(0.2)], [np.deg2rad(-0.15)], [np.deg2rad(0.1)]])
    ba_true = np.array([[0.03], [-0.02], [0.01]])

    ekf = ESKF15(Qc=Qc, R_pos=R_pos, R_yaw=R_yaw, dt=dt)

    # Logs
    T = []
    P_true = []; V_true = []; Q_true = []
    P_est  = []; V_est  = []; Q_est = []
    P_meas = []
    yaw_true_log = []; yaw_meas_log = []; yaw_est_log = []

    # Rotation about X for tilt
    c = math.cos(tilt); s = math.sin(tilt)
    Rx = np.array([[1,0,0],[0,c,-s],[0,s,c]], dtype=float)

    t = 0.0
    first_bootstrap_done = False
    last_yaw_meas = None

    for k in range(steps):
        theta = OMEGA * t

        # Circle (before tilt)
        p0 = np.array([[ Rrad*math.cos(theta)],
                       [ Rrad*math.sin(theta)],
                       [ 0.0 ]])
        v0 = np.array([[-Rrad*OMEGA*math.sin(theta)],
                       [ Rrad*OMEGA*math.cos(theta)],
                       [ 0.0 ]])
        a0 = np.array([[-Rrad*(OMEGA**2)*math.cos(theta)],
                       [-Rrad*(OMEGA**2)*math.sin(theta)],
                       [ 0.0 ]])

        # Tilted into world
        p_w = Rx @ p0
        v_w = Rx @ v0
        a_w = Rx @ a0

        # Frenet triad -> Rwb (body axes in world)
        t_hat = (v_w / (np.linalg.norm(v_w)+1e-12)).reshape(3)
        n_hat = (-p_w / (np.linalg.norm(p_w)+1e-12)).reshape(3)
        b_hat = np.cross(t_hat, n_hat); b_hat /= (np.linalg.norm(b_hat)+1e-12)
        n_hat = np.cross(b_hat, t_hat); n_hat /= (np.linalg.norm(n_hat)+1e-12)
        Rwb_true = np.column_stack([t_hat, n_hat, b_hat])

        # IMU generation
        omega_w = OMEGA * b_hat.reshape(3,1)                  # angular rate about binormal
        omega_b_true = Rwb_true.T @ omega_w
        gyro_meas = omega_b_true + bg_true + np.random.normal(0, sigma_g/np.sqrt(dt), size=(3,1))

        g = np.array([[0],[0],[-9.81]])
        f_w = a_w - g
        f_b_true = Rwb_true.T @ f_w
        acc_meas = f_b_true + ba_true + np.random.normal(0, sigma_a/np.sqrt(dt), size=(3,1))

        # Propagate
        ekf.propagate(gyro_meas, acc_meas)

        # Measurements
        took_pos = False
        if k % k_pos == 0:
            z_pos = p_w + np.random.multivariate_normal([0,0,0], R_pos).reshape(3,1)
            # --- Warm-start on first USBL fix (+ first yaw if we have it) ---
            if not first_bootstrap_done:
                ekf.p = z_pos.copy()
                ekf.v[:] = 0.0  # or estimate from two early fixes
                # For demo: initialize attitude from *true* Rwb at this instant
                # (swap with accel+mag+yaw init in real deployment)
                ekf.q = rotation_to_quaternion(Rwb_true)
                first_bootstrap_done = True
            else:
                ekf.update_pos(z_pos)
            P_meas.append(z_pos.reshape(3))
            took_pos = True
        else:
            P_meas.append([np.nan, np.nan, np.nan])

        if k % k_yaw == 0:
            yaw_true = yaw_from_q(rotation_to_quaternion(Rwb_true))
            z_yaw = yaw_true + np.random.normal(0, np.sqrt(R_yaw[0,0]))
            ekf.update_yaw(z_yaw)
            yaw_true_log.append(yaw_true); yaw_meas_log.append(z_yaw)
            last_yaw_meas = z_yaw
        else:
            yaw_true_log.append(np.nan); yaw_meas_log.append(np.nan)

        # Log states
        T.append(t)
        P_true.append(p_w.reshape(3));  V_true.append(v_w.reshape(3)); Q_true.append(rotation_to_quaternion(Rwb_true))
        P_est.append(ekf.p.reshape(3)); V_est.append(ekf.v.reshape(3)); Q_est.append(ekf.q.copy())
        yaw_est_log.append(yaw_from_q(ekf.q))

        t += dt

    # Arrays
    T = np.array(T)
    P_true = np.array(P_true); V_true = np.array(V_true); Q_true = np.array(Q_true)
    P_est  = np.array(P_est);  V_est  = np.array(V_est);  Q_est  = np.array(Q_est)
    P_meas = np.array(P_meas)
    yaw_true_log = np.array(yaw_true_log); yaw_meas_log = np.array(yaw_meas_log)
    yaw_est_log  = np.array(yaw_est_log)

    # -------------------- Plots --------------------

    # 3D Trajectory
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(P_true[:,0], P_true[:,1], P_true[:,2], label="Ground truth")
    ax.plot(P_est[:,0],  P_est[:,1],  P_est[:,2],  label="EKF estimate")
    ax.scatter(P_meas[:,0], P_meas[:,1], P_meas[:,2], s=8, label="USBL pos", alpha=0.6)
    ax.set_title("Tilted Circular Trajectory (20° about X) — Warm-start")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
    ax.legend(loc="best")

    # Position over time
    fig2 = plt.figure()
    a1 = fig2.add_subplot(311); a2 = fig2.add_subplot(312); a3 = fig2.add_subplot(313)
    a1.plot(T, P_true[:,0], label="x true"); a1.plot(T, P_est[:,0], label="x est"); a1.scatter(T, P_meas[:,0], s=6, label="x meas")
    a2.plot(T, P_true[:,1], label="y true"); a2.plot(T, P_est[:,1], label="y est"); a2.scatter(T, P_meas[:,1], s=6, label="y meas")
    a3.plot(T, P_true[:,2], label="z true"); a3.plot(T, P_est[:,2], label="z est"); a3.scatter(T, P_meas[:,2], s=6, label="z meas")
    a1.set_ylabel("X [m]"); a2.set_ylabel("Y [m]"); a3.set_ylabel("Z [m]"); a3.set_xlabel("Time [s]")
    for axx in (a1,a2,a3): axx.legend(loc="best")

    # Running MSE — position
    pos_err = P_est - P_true
    mse_running = np.cumsum(pos_err**2, axis=0) / (np.arange(len(T))[:,None] + 1)
    fig3 = plt.figure()
    axm = fig3.add_subplot(111)
    axm.plot(T, mse_running[:,0], label="MSE x")
    axm.plot(T, mse_running[:,1], label="MSE y")
    axm.plot(T, mse_running[:,2], label="MSE z")
    axm.plot(T, np.mean(mse_running, axis=1), label="MSE mean")
    axm.set_title("Running MSE — Position"); axm.set_xlabel("Time [s]"); axm.set_ylabel("MSE [m²]")
    axm.legend(loc="best")

    # Running MSE — bearing (yaw)
    def wrap(a): return (a + np.pi) % (2*np.pi) - np.pi
    mask = ~np.isnan(yaw_true_log)
    yaw_err = wrap(yaw_est_log[mask] - yaw_true_log[mask])
    yaw_mse_run = np.cumsum(yaw_err**2) / (np.arange(np.count_nonzero(mask)) + 1)
    fig4 = plt.figure()
    axr = fig4.add_subplot(111)
    axr.plot(T[mask], yaw_mse_run)
    axr.set_title("Running MSE — Bearing (Yaw)"); axr.set_xlabel("Time [s]"); axr.set_ylabel("MSE [rad²]")

    plt.show()

if __name__ == "__main__":
    simulate_tilted_circle()
