import numpy as np
from math import cos, sin, pi, sqrt


def transformation_matrix(config_matrix, n, theta_angles = np.matrix([[0],[0],[0],[0],[0],[0]])):
    n = n - 1
    T_z_th = np.matrix([[cos(theta_angles[n]), -sin(theta_angles[n]), 0, 0],
                        [sin(theta_angles[n]), cos(theta_angles[n]), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])
    # T_z_th = np.matrix([[cos(theta_angles[n]), -sin(theta_angles[n]), 0, 0], [sin(theta_angles[n]), cos(theta_angles[n]), 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])

    T_z_d = np.matrix(np.identity(4))
    T_z_d[2,3] = config_matrix[n,2]
    T_x_a = np.matrix(np.identity(4))
    T_x_a[0, 3] = config_matrix[n, 0]
    T_x_alpha = np.matrix([[1, 0, 0, 0],
                           [0, cos(config_matrix[n, 1]), -sin(config_matrix[n, 1]), 0],
                           [0, sin(config_matrix[n, 1]), cos(config_matrix[n, 1]), 0],
                           [0, 0, 0, 1]])
    T_final = T_z_th*T_z_d*T_x_a*T_x_alpha
    return T_final

def forward_kinematics_solution(config_matrix, theta_angles):
    T01 = transformation_matrix(config_matrix, 1, theta_angles)
    T02 = transformation_matrix(config_matrix, 2, theta_angles)
    T03 = transformation_matrix(config_matrix, 3, theta_angles)
    T04 = transformation_matrix(config_matrix, 4, theta_angles)
    T05 = transformation_matrix(config_matrix, 5, theta_angles)
    T06 = transformation_matrix(config_matrix, 6, theta_angles)
    T = T01*T02*T03*T04*T05*T06
    return T