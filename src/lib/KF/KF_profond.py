#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import math
import time
#from RMatrix import Rxyz  # body->world rotation

# Plotting
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

def Rx(roll):
    c = np.cos(roll)
    s = np.sin(roll)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0,   c,  -s],
                     [0.0,   s,   c]], dtype=float)

def Ry(pitch):
    c = np.cos(pitch)
    s = np.sin(pitch)
    return np.array([[   c, 0.0,   s],
                     [ 0.0, 1.0, 0.0],
                     [  -s, 0.0,    c]], dtype=float)

def Rz(yaw):
    c = np.cos(yaw)
    s = np.sin(yaw)
    return np.array([[   c,  -s, 0.0],
                     [   s,   c, 0.0],
                     [ 0.0, 0.0, 1.0]], dtype=float)

def Rxyz(roll, pitch, yaw):
    # Rotation about X, then Y, then Z
    return Rx(roll) @ Ry(pitch) @ Rz(yaw)

def Rzyx(roll, pitch, yaw):
    # Rotation about Z, then Y, then X
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

def Rxy(roll, pitch):
    return Rx(roll) @ Ry(pitch)

def Ryx(roll, pitch):
    return Ry(pitch) @ Rx(roll)

def A_m(roll, pitch, yaw, T):
    A = np.eye(9)
    R = Rxyz(roll, pitch, yaw)
    A[0:3, 3:6] = T * np.eye(3)
    A[0:3, 6:9] = 0.5 * (T**2) * R
    A[3:6, 6:9] = T * R
    return A

def B_m(n, m, T):
    B = np.zeros((n, m))
    B[0:3, 0:3] = (T**2)/2.0 * np.eye(3)
    B[3:6, 0:3] = T * np.eye(3)
    return B

class KF(object):
    def __init__(self, Q, R1, R2, R3, C1, C2, C3, A=A_m, B=B_m, T=0.05, n=9, m=3):
        self.n, self.m, self.T = n, m, T
        self.Pkk = np.eye(n)
        self.x = np.zeros((n,1))
        self.A, self.B = A, B
        self.C1, self.C2, self.C3 = C1, C2, C3
        self.Q, self.R1, self.R2, self.R3 = Q, R1, R2, R3
        self.t = time.time()

    def setX(self, X):
        X = np.asarray(X)
        if X.shape != (self.n,1):
            raise ValueError("X must be shape ({},1)".format(self.n))
        self.x = X

    def getX(self):
        return self.x

    def update(self, yy, roll, pitch, yaw, mode='imu', dt_override=None):
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

        if mode == 'imu':
            C, R = self.C1, self.R1
        elif mode == 'usbl':
            C, R = self.C2, self.R2
        elif mode == 'depth':
            C, R = self.C3, self.R3
        else:
            raise ValueError("mode must be 'imu', 'usbl', or 'depth'")

        yy = np.asarray(yy)
        if yy.ndim == 1:
            yy = yy.reshape((-1,1))

        S = C @ Pk1k @ C.T + R
        K = Pk1k @ C.T @ np.linalg.inv(S)
        self.x = Xk1k + K @ (yy - C @ Xk1k)
        self.Pkk = (np.eye(self.n) - K @ C) @ Pk1k