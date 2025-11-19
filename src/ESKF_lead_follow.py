#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import matplotlib.pyplot as plt

# -------------------------- Quaternion utils --------------------------

def skew(w):
    wx, wy, wz = w
    return np.array([[ 0, -wz,  wy],
                     [ wz,  0, -wx],
                     [-wy,  wx,   0]], dtype=float)

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

# For quaternion derivative inside truth integrator
def Omega(omega):
    wx, wy, wz = omega
    return np.array([
        [0, -wx, -wy, -wz],
        [wx, 0,  wz, -wy],
        [wy, -wz, 0,  wx],
        [wz,  wy, -wx, 0]
    ], dtype=float)

# -------------------------- Kinematic leader ("fish") --------------------------

def leader_pose_circle(t, R=12.0, w=0.05, z0=-2.0, Az=0.5, wz=0.20):
    """
    Leader swims on a horizontal circle of radius R with a gentle vertical undulation.
    Returns (pL [3x1], vL [3x1], qL [4], wL_body [3x1 approx]).
    """
    x = R*math.cos(w*t)
    y = R*math.sin(w*t)
    z = z0 + Az*math.sin(wz*t)

    vx = -R*w*math.sin(w*t)
    vy =  R*w*math.cos(w*t)
    vz =  Az*wz*math.cos(wz*t)

    pL = np.array([[x],[y],[z]])
    vL = np.array([[vx],[vy],[vz]])

    speed_xy = math.hypot(vx, vy)
    if speed_xy < 1e-9:
        yaw = 0.0
        yawdot = 0.0
    else:
        yaw = math.atan2(vy, vx)
        ax = -R*w*w*math.cos(w*t)
        ay = -R*w*w*math.sin(w*t)
        yawdot = (vx*ay - vy*ax) / (speed_xy**2 + 1e-9)

    cy = math.cos(0.5*yaw); sy = math.sin(0.5*yaw)
    qL = np.array([cy, 0.0, 0.0, sy], dtype=float)
    wL_body = np.array([[0.0],[0.0],[yawdot]])
    return pL, vL, qL, wL_body


# -------------------------- Kinematic leader ("fish") — forward sinusoid --------------------------
def leader_pose(t, v=0.6, A=2.0, lam=15.0, z0=-2.0, Az=0.0, wz=0.20):
    """
    Forward motion along +X with lateral sinusoid in Y:
        x(t) = v * t
        y(t) = A * sin(2π/lam * x(t))        # sinusoid vs distance along x
    Vertical motion (optional):
        z(t) = z0 + Az * sin(wz * t)

    Returns:
        pL [3x1], vL [3x1], qL [4], wL_body [3x1]  (yaw-only orientation)
    """
    k = 2.0 * math.pi / max(lam, 1e-6)

    # Position
    x = v * t
    y = A * math.sin(k * x)
    z = z0 + Az * math.sin(wz * t)

    # Velocity
    vx = v
    vy = A * k * v * math.cos(k * x)
    vz = Az * wz * math.cos(wz * t)

    pL = np.array([[x],[y],[z]])
    vL = np.array([[vx],[vy],[vz]])

    # Heading aligned with the instantaneous velocity tangent
    yaw = math.atan2(vy, vx)

    # Yaw rate: ψ̇ = (ẋ ÿ − ẏ ẍ) / (ẋ² + ẏ²)
    xdd = 0.0
    ydd = -A * (k**2) * (v**2) * math.sin(k * x)
    denom = vx*vx + vy*vy + 1e-9
    yawdot = (vx * ydd - vy * xdd) / denom

    # Quaternion with yaw only (flat fish)
    cy = math.cos(0.5*yaw); sy = math.sin(0.5*yaw)
    qL = np.array([cy, 0.0, 0.0, sy], dtype=float)

    wL_body = np.array([[0.0],[0.0],[yawdot]])
    return pL, vL, qL, wL_body


# -------------------------- Tether model --------------------------

def tether_force(follower,
                 leader_p, leader_v,
                 r_attach_f_b=np.array([[0.30],[0.0],[0.0]]),
                 r_attach_L_w=np.array([[-0.20],[0.0],[0.0]]),
                 L0=2.0, k=120.0, c=35.0, T_max=250.0):
    """
    Returns (F_tether_w [3x1], tau_tether_b [3x1]) applied to the follower.
    Tension-only spring–damper. Moderate k & adequate c recommended for dt=0.01 s.
    """
    Rwb = R_from_q(follower.q)
    pF = follower.p + Rwb @ r_attach_f_b          # follower attach in world
    pL = leader_p + r_attach_L_w                   # leader tow point in world

    d = pF - pL
    ell = float(np.linalg.norm(d))
    if ell < 1e-9:
        return np.zeros((3,1)), np.zeros((3,1))
    d_hat = d / ell

    v_rel = (follower.v - leader_v)
    v_ax = float(d_hat.T @ v_rel)

    stretch = max(0.0, ell - L0)
    T = k*stretch + c*v_ax
    T = max(0.0, min(T, T_max))

    Fw = -T * d_hat
    Fb = Rwb.T @ Fw
    tau_b = np.cross(r_attach_f_b.reshape(3), Fb.reshape(3)).reshape(3,1)
    return Fw, tau_b

# -------------------------- Minimal actuators & 6-DoF underwater truth --------------------------

class FirstOrderActuator:
    def __init__(self, tau=0.15, umin=-1.0, umax=1.0):
        self.tau = tau
        self.umin = umin; self.umax = umax
        self.y = 0.0
    def step(self, dt, u_cmd):
        u_cmd = float(np.clip(u_cmd, self.umin, self.umax))
        self.y += dt*(u_cmd - self.y)/max(self.tau, 1e-6)
        return self.y

class AUVTruth:
    """
    Simple 6-DoF rigid-body with buoyancy, drag, current, and 2 actuators (surge force + yaw torque).
    State: p(3), v(3), q(4), w(3)
    Sensors: IMU (f, gyro), heading (yaw) with latency, range-to-buoy with latency, depth.
             (We will also simulate USBL global position fixes in the outer loop.)
    """
    def __init__(self, dt):
        self.dt = dt
        # Vehicle parameters (rough, stable)
        self.m = 18.0
        self.I = np.diag([0.45, 0.45, 0.80])
        self.g = np.array([[0],[0],[-9.81]])
        self.rho = 1025.0
        self.V = self.m/self.rho      # ~neutral buoyancy
        self.r_cb = np.array([[0.0],[0.0],[0.02]])  # CB above CoM -> righting

        # Drag (world-frame translational) + angular damping (body)
        self.Dv = np.diag([6.0, 8.0, 10.0])   # linear
        self.Kv = np.diag([4.0, 6.0, 10.0])   # quadratic
        self.Dw = np.diag([0.3, 0.3, 0.4])    # angular damping

        # State
        self.p = np.zeros((3,1))
        self.v = np.zeros((3,1))
        self.q = np.array([1.0,0,0,0], dtype=float)
        self.w = np.zeros((3,1))

        # Actuators: surge force (N) & yaw torque (N.m), with lag + saturation
        self.th_surge = FirstOrderActuator(tau=0.2, umin=-50.0, umax=50.0)
        self.th_yaw   = FirstOrderActuator(tau=0.25, umin=-6.0,  umax=6.0)

        # Current model (world-frame, slow OU-like)
        self.vc_w = np.zeros((3,1))
        self.vc_bias = np.array([[0.1],[0.0],[0.0]])  # 0.1 m/s eastward bias

        # IMU bias RW + noise (per sqrt(s))
        self.bg = np.deg2rad(np.array([[0.2],[-0.1],[0.05]]))
        self.ba = np.array([[0.03],[-0.02],[0.01]])
        self.sigma_g  = np.deg2rad(0.05)
        self.sigma_a  = 0.05
        self.sigma_bg = np.deg2rad(0.002)
        self.sigma_ba = 0.005

        # Range/depth/yaw noise
        self.c_true = 1480.0
        self.c_ass  = 1500.0
        self.sigma_r = 0.15
        self.sigma_z = 0.002
        self.sigma_yaw = np.deg2rad(2.0)

        # Buoy (moving) in world
        self.pb = np.array([[10.0],[0.0],[0.0]])
        self.pb_noise = np.array([[0.0],[0.0],[0.0]])

        # Latency buffers
        self.lat_buf_yaw   = []  # (t_available, yaw)
        self.lat_buf_range = []  # (t_available, range, buoy_pos_at_tx)

    def _buoy_update(self, t):
        # Small drift + bobbing (no vertical in this model)
        self.pb += np.array([[0.0005],[0.0002],[0.0]])  # slow drift
        self.pb_noise = np.array([
            [0.3*np.sin(0.2*t)],
            [0.3*np.cos(0.17*t)],
            [0.0]
        ])

    def _current_update(self, dt):
        alpha = 0.005
        self.vc_w += dt*(-alpha*(self.vc_w - self.vc_bias) + 0.02*np.random.randn(3,1))

    def _forces_torques(self, u_surge, u_yaw, Fext_w=None, tauext_b=None):
        Rwb = R_from_q(self.q)
        v_rel_w = self.v - self.vc_w
        Fext_w = np.zeros((3,1)) if Fext_w is None else Fext_w
        tauext_b = np.zeros((3,1)) if tauext_b is None else tauext_b

        # Drag (world)
        F_lin  = - self.Dv @ v_rel_w
        F_quad = - self.Kv @ (np.linalg.norm(v_rel_w, axis=0) * v_rel_w)
        F_drag = F_lin + F_quad

        # Gravity & buoyancy (world)
        F_grav = self.m * self.g
        F_buoy = - self.rho * self.V * self.g

        # Thruster force in body -> world
        F_thr_b = np.array([[u_surge],[0.0],[0.0]])
        F_thr_w = Rwb @ F_thr_b

        # Torques: yaw control + righting due to CB offset + angular damping
        tau_thr_b   = np.array([[0.0],[0.0],[u_yaw]])
        tau_hydro_b = - self.Dw @ self.w
        Fb_b        = Rwb.T @ F_buoy
        tau_right_b = np.cross(self.r_cb.reshape(3), Fb_b.reshape(3)).reshape(3,1)
        tau_b = tau_thr_b + tau_hydro_b + tau_right_b + tauext_b

        F_w  = F_grav + F_buoy + F_drag + F_thr_w + Fext_w
        return F_w, tau_b

    def step(self, t, dt, u_surge_cmd, u_yaw_cmd, Fext_w=None, tauext_b=None):
        # Environment
        self._current_update(dt)
        self._buoy_update(t)

        # Actuators
        u_surge = self.th_surge.step(dt, u_surge_cmd)
        u_yaw   = self.th_yaw.step(dt, u_yaw_cmd)

        # Dynamics (semi-implicit Euler)
        F_w, tau_b = self._forces_torques(u_surge, u_yaw, Fext_w, tauext_b)
        self.v += dt * (F_w / self.m)
        self.p += dt * self.v

        wdot = np.linalg.inv(self.I) @ (tau_b - np.cross(self.w.reshape(3), (self.I @ self.w).reshape(3)).reshape(3,1))
        self.w += dt * wdot
        qdot = 0.5 * (Omega(self.w.reshape(3)) @ self.q)
        self.q = q_normalize(self.q + dt*qdot)

        # IMU sim (specific force + gyro), add bias RW + noise
        self.bg += np.sqrt(dt)*self.sigma_bg*np.random.randn(3,1)
        self.ba += np.sqrt(dt)*self.sigma_ba*np.random.randn(3,1)
        Rwb = R_from_q(self.q)
        a_w = (F_w / self.m)  # includes gravity already
        f_b = Rwb.T @ (a_w - self.g)  # specific force in body
        gyro_meas = self.w + self.bg + (self.sigma_g/np.sqrt(dt))*np.random.randn(3,1)
        acc_meas  = f_b    + self.ba + (self.sigma_a/np.sqrt(dt))*np.random.randn(3,1)

        # Yaw (mag) with latency/dropouts
        yaw_true = yaw_from_q(self.q)
        yaw_meas = yaw_true + np.random.normal(0, self.sigma_yaw)
        if np.random.rand() < 0.8:  # 80% chance to get heading fix
            self.lat_buf_yaw.append((t+0.15, yaw_meas))

        # Range to buoy (with speed-of-sound mismatch) + latency
        pb_eff = self.pb + self.pb_noise
        r_true = float(np.linalg.norm(self.p - pb_eff))
        if np.random.rand() < 0.02:  # occasional outlier
            r_meas = r_true + np.random.normal(5.0, 1.0)
        else:
            r_meas = (r_true / self.c_true) * self.c_ass + np.random.normal(0, self.sigma_r)
        if np.random.rand() < 0.9:   # 90% chance to get a ping
            self.lat_buf_range.append((t+0.25, r_meas, pb_eff.copy()))

        # Depth (10 Hz nominal; caller can sub-sample)
        z_meas = float(self.p[2,0] + np.random.normal(0, self.sigma_z) + 0.01*np.sin(0.005*t))

        return gyro_meas, acc_meas, yaw_true, pb_eff, z_meas

    def pop_delayed_yaw(self, t):
        if self.lat_buf_yaw and self.lat_buf_yaw[0][0] <= t:
            return self.lat_buf_yaw.pop(0)[1]
        return None

    def pop_delayed_range(self, t):
        if self.lat_buf_range and self.lat_buf_range[0][0] <= t:
            _, r, pb = self.lat_buf_range.pop(0)
            return r, pb
        return None

# -------------------------- ESKF (15-state error) with extra updates --------------------------

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

# -------------------------- Simulation using the new model --------------------------

def simulate_underwater_dynamics():
    np.random.seed(7)

    # Sim params
    dt    = 0.01
    steps = 50000      # 200 s
    TOWED_MODE = True  # follower is dragged by the leader fish

    # IMU/bias spectral densities (per sqrt(s))
    sigma_g  = np.deg2rad(0.05)
    sigma_a  = 0.05
    sigma_bg = np.deg2rad(0.002)
    sigma_ba = 0.005
    Qc = np.diag([sigma_g**2]*3 + [sigma_a**2]*3 + [sigma_bg**2]*3 + [sigma_ba**2]*3)

    # Measurement covariances
    R_pos = np.diag([0.10**2, 0.10**2, 0.10**2])    # USBL global position fix
    R_yaw = np.array([[np.deg2rad(2.0)**2]])        # heading sensor

    # USBL fix schedule & latency (simulate global position like before, but delayed)
    usbl_period = 0.5    # 2 Hz fixes
    usbl_latency = 0.0  # 250 ms delay
    usbl_availability = 0.9  # 90% chance the fix is available

    # Optional absolute position fixes are ON (this is your USBL)
    USE_USBL_POS = True
    k_usbl = int(round(usbl_period/dt))

    ekf = ESKF15(Qc=Qc, R_pos=R_pos, R_yaw=R_yaw, dt=dt)
    truth = AUVTruth(dt=dt)

    # Leader parameters
    Rlead, wlead, z0, Az, wz = 12.0, 0.05, -2.0, 0.5, 0.20

    # Leader parameters (forward sinusoid)
    v_lead, A_lat, lam, z0, Az, wz = 1, 1.0, 60.0, -0.10, 0.0, 0.20

    # Controller gains (disabled in towed mode)
    v_des = 0.6 if not TOWED_MODE else 0.0
    kp_yaw = 2.5 if not TOWED_MODE else 0.0
    kd_yaw = 0.3 if not TOWED_MODE else 0.0
    k_v = 40.0 if not TOWED_MODE else 0.0
    u0_surge = 25.0 if not TOWED_MODE else 0.0

    # Logs
    T = []
    P_true = []; V_true = []; Q_true = []
    P_est  = []; V_est  = []; Q_est  = []
    yaw_true_log = []; yaw_meas_log = []; yaw_est_log = []
    P_meas = []            # scatter of USBL position fixes (global)
    PL_log = []            # leader path
    tension_log = []       # tether tension

    # USBL latency buffer (t_available, pos)
    usbl_lat_buf = []

    t = 0.0
    first_bootstrap_done = False

    for k in range(steps):
        # --- Leader kinematics ---
        #pL, vL, qL, wL = leader_pose_circle(t, Rlead, wlead, z0, Az, wz)
        pL, vL, qL, wL = leader_pose(t, v=v_lead, A=A_lat, lam=lam, z0=z0, Az=Az, wz=wz)
        PL_log.append(pL.reshape(3).copy())

        # --- Yaw/surge controller (kept for completeness; zeroed if towed) ---
        yaw_now = yaw_from_q(truth.q)
        Rwb = R_from_q(truth.q)
        v_b = Rwb.T @ truth.v
        vx_b = float(v_b[0,0])

        # "free" heading target (won't matter in TOWED_MODE)
        p_xy = truth.p[0:2,0]
        r2 = float(p_xy[0]**2 + p_xy[1]**2)
        if r2 < 1e-6:
            phi_des = yaw_now
        else:
            ex = p_xy[0] / math.sqrt(r2); ey = p_xy[1] / math.sqrt(r2)
            tx, ty = -ey, ex
            phi_des = math.atan2(ty, tx)

        def wrap(a): return (a + np.pi) % (2*np.pi) - np.pi
        e_yaw = wrap(phi_des - yaw_now)
        u_yaw_cmd = kp_yaw*e_yaw - kd_yaw*float(truth.w[2,0])
        u_surge_cmd = u0_surge + k_v*(v_des - vx_b)

        # --- Tether loads ---
        if TOWED_MODE:
            F_teth_w, tau_teth_b = tether_force(
                follower=truth,
                leader_p=pL, leader_v=vL,
                r_attach_f_b=np.array([[0.30],[0.0],[0.0]]),
                r_attach_L_w=np.array([[-0.20],[0.0],[0.0]]),
                L0=2.0, k=120.0, c=35.0, T_max=250.0
            )
            tension_log.append(float(np.linalg.norm(F_teth_w)))
        else:
            F_teth_w = np.zeros((3,1)); tau_teth_b = np.zeros((3,1))
            tension_log.append(0.0)

        # --- Truth propagation & sensor sim (pass external loads) ---
        gyro_meas, acc_meas, yaw_true, pb_eff, z_meas = truth.step(
            t, dt, u_surge_cmd, u_yaw_cmd,
            Fext_w=F_teth_w, tauext_b=tau_teth_b
        )

        # --- ESKF propagate ---
        ekf.propagate(gyro_meas, acc_meas)

        # --- Depth update at 10 Hz ---
        if k % 10 == 0:
            ekf.update_depth(z_meas)

        # --- Delayed Yaw ---
        y_delayed = truth.pop_delayed_yaw(t)
        if y_delayed is not None:
            ekf.update_yaw(y_delayed)
            yaw_meas_log.append(y_delayed)
        else:
            yaw_meas_log.append(np.nan)

        # --- Delayed Range ---
        r_delayed = truth.pop_delayed_range(t)
        if r_delayed is not None:
            r_meas, p_buoy = r_delayed
            ekf.update_range(r_meas, p_buoy)

        # --- USBL global position fix (like before, but with latency) ---
        if USE_USBL_POS and (k % k_usbl == 0):
            if np.random.rand() < usbl_availability:
                z_pos = truth.p + np.random.multivariate_normal([0,0,0], R_pos).reshape(3,1)
                usbl_lat_buf.append((t + usbl_latency, z_pos))
        # pop matured USBL fixes
        used_meas_this_step = np.array([np.nan, np.nan, np.nan])
        if usbl_lat_buf and usbl_lat_buf[0][0] <= t:
            _, z_pos_ready = usbl_lat_buf.pop(0)
            if not first_bootstrap_done:
                # bootstrap attitude and pos once (demo-friendly)
                ekf.p = z_pos_ready.copy()
                ekf.v[:] = 0.0
                ekf.q = truth.q.copy()
                first_bootstrap_done = True
            else:
                ekf.update_pos(z_pos_ready)
            used_meas_this_step = z_pos_ready.reshape(3)
        P_meas.append(used_meas_this_step.copy())

        if not first_bootstrap_done and not USE_USBL_POS:
            # Minimal bootstrap if USBL is off: start near truth attitude
            ekf.q = truth.q.copy()
            first_bootstrap_done = True

        # -------- Logs (use copies to avoid aliasing!) --------
        T.append(t)
        P_true.append(truth.p.reshape(3).copy())
        V_true.append(truth.v.reshape(3).copy())
        Q_true.append(truth.q.copy())

        P_est.append(ekf.p.reshape(3).copy())
        V_est.append(ekf.v.reshape(3).copy())
        Q_est.append(ekf.q.copy())

        yaw_true_log.append(yaw_true)
        yaw_est_log.append(yaw_from_q(ekf.q))
        # ------------------------------------------------------

        t += dt

    # Arrays
    T = np.array(T)
    P_true = np.array(P_true); V_true = np.array(V_true); Q_true = np.array(Q_true)
    P_est  = np.array(P_est);  V_est  = np.array(V_est);  Q_est  = np.array(Q_est)
    yaw_true_log = np.array(yaw_true_log); yaw_meas_log = np.array(yaw_meas_log)
    yaw_est_log  = np.array(yaw_est_log)
    P_meas = np.array(P_meas)
    PL_log = np.array(PL_log)
    tension_log = np.array(tension_log)

    # -------------------- Plots --------------------
    # 3D Trajectory: Leader vs Follower (truth) and EKF estimate
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(PL_log[:,0], PL_log[:,1], PL_log[:,2], label="Leader fish")
    ax.plot(P_true[:,0], P_true[:,1], P_true[:,2], label="Follower — truth")
    ax.plot(P_est[:,0],  P_est[:,1],  P_est[:,2],  label="Follower — ESKF")
    if np.isfinite(P_meas).any():
        ax.scatter(P_meas[:,0], P_meas[:,1], P_meas[:,2], s=8, label="USBL pos", alpha=0.6)
    ax.set_title("Towed AUV — Leader vs Follower (Truth & ESKF)")
    ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]"); ax.set_zlabel("Z [m]")
    ax.legend(loc="best")

    # Position over time (with USBL measurements)
    fig2 = plt.figure()
    a1 = fig2.add_subplot(311); a2 = fig2.add_subplot(312); a3 = fig2.add_subplot(313)
    a1.plot(T, P_true[:,0], label="x true"); a1.plot(T, P_est[:,0], label="x est")
    a1.scatter(T, P_meas[:,0], s=6, label="x USBL")
    a2.plot(T, P_true[:,1], label="y true"); a2.plot(T, P_est[:,1], label="y est")
    a2.scatter(T, P_meas[:,1], s=6, label="y USBL")
    a3.plot(T, P_true[:,2], label="z true"); a3.plot(T, P_est[:,2], label="z est")
    a3.scatter(T, P_meas[:,2], s=6, label="z USBL")
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
    def wrap_arr(a): return (a + np.pi) % (2*np.pi) - np.pi
    mask = ~np.isnan(yaw_true_log)
    yaw_err = wrap_arr(yaw_est_log[mask] - yaw_true_log[mask])
    yaw_mse_run = np.cumsum(yaw_err**2) / (np.arange(np.count_nonzero(mask)) + 1)
    fig4 = plt.figure()
    axr = fig4.add_subplot(111)
    axr.plot(T[mask], yaw_mse_run)
    axr.set_title("Running MSE — Bearing (Yaw)"); axr.set_xlabel("Time [s]"); axr.set_ylabel("MSE [rad²]")

    # Tether tension over time
    fig5 = plt.figure()
    a5 = fig5.add_subplot(111)
    a5.plot(T, tension_log)
    a5.set_title("Tether Tension vs Time")
    a5.set_xlabel("Time [s]"); a5.set_ylabel("Tension [N]")

    plt.show()

if __name__ == "__main__":
    simulate_underwater_dynamics()
