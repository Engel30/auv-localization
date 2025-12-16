#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

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
