import numpy as np
from math import pi
from forward_kinematics import transformation_matrix

config_matrix = np.matrix([[0, pi/2.0, 0.1625],
                        [-0.425, 0, 0],
                        [-0.3922, 0, 0],
                        [0, pi/2.0, 0.1333],
                        [0, -pi/2.0, 0.0997],
                        [0, 0, 0.0996]])

ed = np.matrix([[1.572584629058838], [-1.566467599277832], [-0.0026149749755859375], [-1.568673924808838],
                    [-0.009446446095601857], [0.007950782775878906]])

def jacobian_solution(config_matrix, joints, theta_angles):
    joints = joints - 1
    p0 = np.matrix([[0],[0],[0]])
    z0 = np.matrix([[0],[0],[1]])
    p6 = transformation_matrix(config_matrix, joints, theta_angles)[:3,3]

    

    print(p6)

jacobian_solution(config_matrix, 6, ed)