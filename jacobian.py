from forward_kinematics import forward_kinematics_solution
from forward_kinematics import transformation_matrix
import numpy as np
from math import pi

config_matrix = np.matrix([[0, pi/2.0, 0.1625],
                        [-0.425, 0, 0],
                        [-0.3922, 0, 0],
                        [0, pi/2.0, 0.1333],
                        [0, -pi/2.0, 0.0997],
                        [0, 0, 0.0996]])

ed = np.matrix([[1.572584629058838], [-1.566467599277832], [-0.0026149749755859375], [-1.568673924808838],
                    [-0.009446446095601857], [0.007950782775878906]])

def jacobian_solution(config_matrix, joints, theta_angles):
    jacobian = np.matrix(np.zeros([6, joints]))
    #p0 = np.matrix([[0],[0],[0]])
    z0 = np.matrix([[0],[0],[1]])

    joints = joints - 1
    p6 = forward_kinematics_solution(config_matrix, theta_angles)[:3,3]

    for i in range(1, joints):
        tm = forward_kinematics_solution(config_matrix, i, theta_angles)
        jacobian[:3,i] = np.cross(tm[:3,2].T,(p6 - tm[:3,3]).T).T
        jacobian[3:,i] = tm[:3,2]

    print(jacobian)

jacobian_solution(config_matrix, 6, ed)