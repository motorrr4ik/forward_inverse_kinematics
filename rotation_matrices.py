import numpy as np
from math import sin, cos

def Rx(theta):
    rx = np.array([[1, 0, 0],
                [0, cos(theta), -sin(theta)],
                [0, sin(theta), cos(theta)]]) 
    return rx

def Ry(theta):
    ry = np.array([[cos(theta), 0, sin(theta)],
                [0, 1, 0],
                [-sin(theta), 0, cos(theta)]]) 
    return ry

def Rz(theta):
    rz = np.array([[cos(theta), -sin(theta), 0],
                [sin(theta), cos(theta), 0],
                [0, 0, 1]]) 
    return rz