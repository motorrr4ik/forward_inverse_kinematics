import numpy as np
from math import cos, sin, pi, sqrt


def transformation_matrix(config_matrix, n, theta_angles = np.array([[0],[0],[0],[0],[0],[0]])):
    n = n - 1
    T_z_th = np.array([[cos(theta_angles[n]), -sin(theta_angles[n]), 0, 0],
                        [sin(theta_angles[n]), cos(theta_angles[n]), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])

    T_z_d = np.array(np.identity(4))
    T_z_d[2,3] = config_matrix[n,2]
    T_x_a = np.array(np.identity(4))
    T_x_a[0, 3] = config_matrix[n, 0]
    T_x_alpha = np.array([[1, 0, 0, 0],
                           [0, cos(config_matrix[n, 1]), -sin(config_matrix[n, 1]), 0],
                           [0, sin(config_matrix[n, 1]), cos(config_matrix[n, 1]), 0],
                           [0, 0, 0, 1]])
    T_final = T_z_th.dot(T_z_d).dot(T_x_a).dot(T_x_alpha)
    return T_final

def forward_kinematics_solution(config_matrix, joints, theta_angles):
    T_final = np.array(np.eye(4, dtype=int))
    for i in range(1,joints+1):
        T_cur = transformation_matrix(config_matrix, i, theta_angles)
        T_final = T_final.dot(T_cur)
    return T_final

