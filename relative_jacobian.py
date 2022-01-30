from forward_kinematics import forward_kinematics_solution, transformation_matrix
import numpy as np 
import matplotlib.pyplot as plt

def relative_jacobian_matrix(left_T_mat, right_T_mat, left_J, right_J, y_position):
    relative_jacobian = np.array(np.zeros((6,6)))

    relative_jacobian[:3,:3] = right_T_mat[:3,:3].T 
    relative_jacobian[3:, 3:] = right_T_mat[:3,:3].T 


    return relative_jacobian

def __gamma_matrix(T_mat):
    gamma_mat = np.array(np.zeros((6,6)))
    gamma_mat[:3, :3] = np.array(np.eye(3))
    gamma_mat[3:, 3:] = np.array(np.eye(3))
    gamma_mat[3, 3] = y_position -  T_mat[:3, 3]
    return gamma_mat


# relative_jacobian_matrix(0, 0, 0, 0)