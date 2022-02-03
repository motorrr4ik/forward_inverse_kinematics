import numpy as np
from numpy import linalg
from math import cos, acos, atan2, sin, pi, sqrt, asin
from . import transformation_matrix

def inverse_kinematics(config_matrix, transformation_mat):

    theta = np.zeros((6,8))
    T = transformation_mat

    #theta 1
    P_05 = T.dot(np.array([[0],[0], [-config_matrix[5,2]], [1]]))- np.array([[0],[0],[0],[1]])
    psi = atan2(P_05[1], P_05[0])
    if P_05[0]**2 + P_05[1]**2 < config_matrix[3,2]:
        print("Impossible position")
        pass
    else:
        phi = acos(config_matrix[3,2]/sqrt(P_05[0]**2 + P_05[1]**2))
    # phi = acos(config_matrix[3,2]/sqrt(P_05[0]**2 + P_05[1]**2))
    theta[0, 0:4] = psi + phi + pi/2
    theta[0, 4:8] = psi - phi + pi/2
    #----------------------------------------------------------------------------
    #theta 5
    for i in {0, 4}:
            cos_theta5 = (T[0, 3] * sin(theta[0, i]) - T[1, 3] * cos(theta[0, i]) - 
                    (config_matrix[3,2])) / config_matrix[5, 2]

            if -1 <= cos_theta5 <= 1:
                theta5 = acos(cos_theta5)
            else:
                theta5 = 0

            theta[4, i:i + 2] = theta5
            theta[4, i + 2:i + 4] = -theta5
    #----------------------------------------------------------------------------
    #theta 6 
    for i in {0, 2, 4, 6}:

        T_inv = linalg.inv(T)
        if sin(theta[4,i]) == 0:
            theta6 = 0
        else:
            theta6 = atan2((-T_inv[1, 0] * sin(theta[0, i]) + T_inv[1, 1] * cos(theta[0, i]))/sin(theta[4,i]),
                    (T_inv[0, 0] * sin(theta[0, i]) - T_inv[0, 1] * cos(theta[0, i]))/sin(theta[4,i]))
        theta[5, i:i + 2] = theta6  
    #----------------------------------------------------------------------------
    #theta 3 
    for i in {0, 2, 4, 6}:
        T01 = transformation_matrix(config_matrix, 1, theta[:, i])
        T45 = transformation_matrix(config_matrix, 5, theta[:, i])
        T56 = transformation_matrix(config_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01).dot(T).dot(linalg.inv(T45.dot(T56)))
        P14 = T14.dot(np.array([[0], [-config_matrix[3, 2]], [0], [1]]))
        costh3 = ((P14[0] ** 2 + P14[1] ** 2 - config_matrix[1, 0] ** 2 - config_matrix[2, 0] ** 2) /
                  (2 * config_matrix[1, 0] * config_matrix[2, 0]))
        if 1 >= costh3 >= -1:
            th3 = acos(costh3)
        else:
            th3 = 0
        theta[2, i] = th3
        theta[2, i + 1] = -th3
    #----------------------------------------------------------------------------
    #theta 2, 4
    for i in range(8):
        T01 = transformation_matrix(config_matrix, 1, theta[:, i])
        T45 = transformation_matrix(config_matrix, 5, theta[:, i])
        T56 = transformation_matrix(config_matrix, 6, theta[:, i])
        T14 = linalg.inv(T01).dot(T).dot(linalg.inv(T45.dot(T56)))
        P13 = T14.dot(np.array([[0], [-config_matrix[3, 2]], [0], [1]]))

        theta[1, i] = atan2(-P13[1], -P13[0]) - asin(
            -config_matrix[2, 0] * sin(theta[2, i]) / sqrt(P13[0] ** 2 + P13[1] ** 2)
        )
        T32 = linalg.inv(transformation_matrix(config_matrix, 3, theta[:, i]))
        T21 = linalg.inv(transformation_matrix(config_matrix, 2, theta[:, i]))
        T34 = T32.dot(T21).dot(T14)
        theta[3, i] = atan2(T34[1, 0], T34[0, 0]) 
    return theta