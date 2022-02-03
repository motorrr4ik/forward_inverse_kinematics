from forward_kinematics import forward_kinematics_solution, transformation_matrix
import numpy as np
from math import cos, sin, pi, sqrt
import matplotlib.pyplot as plt

config_matrix = np.array([[0, pi/2.0, 0.1625],
                        [-0.425, 0, 0],
                        [-0.3922, 0, 0],
                        [0, pi/2.0, 0.1333],
                        [0, -pi/2.0, 0.0997],
                        [0, 0, 0.0996]])

left_ang = np.array([[0.841471], [0.841471], [0.841471], [0.841471],
                    [0.841471], [0.841471]])

right_ang = np.array([[2*0.841471], [2*0.841471], [2*0.841471], [2*0.841471],
                    [2*0.841471], [2*0.841471]])

def jacobian_solution(config_matrix, joints, theta_angles):

    jacobian = np.array(np.zeros((6, joints)))
    p6 = forward_kinematics_solution(config_matrix, joints, theta_angles)[:3,3]
    cur_z = np.array([[0],[0],[1]])
    cur_p = np.array([[0],[0],[0]])

    jacobian[:3,0] = np.cross(cur_z.flatten(),(p6 - cur_p.flatten()))
    jacobian[3:,0] = cur_z.flatten()

    for i in range(1, joints):
        tm = forward_kinematics_solution(config_matrix, i, theta_angles)
        jacobian[:3,i] = np.cross(tm[:3,2].flatten(),(p6 - tm[:3,3]).flatten()).T
        jacobian[3:,i] = tm[:3,2]

    # print(jacobian)
    return jacobian

def coop_jacobian(left_J, right_J):
    coop_J = np.array(np.zeros((12,12)))
    coop_J[:6, :6] = 0.5*left_J
    coop_J[:6, 6:] = 0.5*right_J
    coop_J[6:,:6] = left_J
    coop_J[6:,6:] = right_J 

    return coop_J




def manipulability_metrics(jacobian_matrix):
    return sqrt(np.linalg.det(jacobian_matrix.dot(jacobian_matrix.T)))

def jacobian_test():
    T = 1e-3
    time = np.arange(1, 16, T)
    q = np.zeros((len(time), 6))
    dot_q = np.zeros((len(time),6))
    x = np.array(np.zeros((3,len(q))))
    dot_x = np.array(np.zeros((3,len(q))))
    dot_x_arr = []
    w_array = [] 

    sin_arr = np.sin(time)
    cos_arr = np.cos(time)

    for i in range(6):
        # q = np.insert(q, i, sin_arr, axis=1)
        # dot_q = np.insert(dot_q, i, cos_arr, axis=1)
        q[:,i] = sin_arr.T
        dot_q[:,i] = cos_arr.T

    for i in range(len(q)):
        x[:,i] = forward_kinematics_solution(config_matrix, 6, q[i])[:3,3]
        
    new_dot_x = np.diff(x)/T
    jacobian_dot_x = np.array(np.zeros((6, len(time))))

    for i in range(len(q)-1):
        cur_J = jacobian_solution(config_matrix, 6, q[i])
        jacobian_dot_x[:,i] = cur_J.dot(np.array(dot_q[i]).T)
        dot_x_arr.append(jacobian_dot_x[:,i])
        w_array.append(manipulability_metrics(cur_J))

    # print(jacobian_dot_x[:3,:3])
    # print(new_dot_x[:3,:3])
    # print(dot_x)
    # print("kek")
    # print(new_dot_x)
    # print(dot_x_arr)
    # dot_x_arr = np.array(dot_x_arr)[:, :, 0].T
    # print(dot_x_arr)
    diff_matrix = jacobian_dot_x[:3, :np.shape(new_dot_x)[1]] - new_dot_x
    dot_x_arr = np.array(jacobian_dot_x)
    w_array = np.array(w_array)
    plt.grid(True)

    # plt.plot(np.array(dot_x_arr[0, :]).flatten(), label = 'x')
    # plt.plot(np.array(new_dot_x[0, :]).flatten())
    # plt.plot(np.array(dot_x_arr[1, :]).flatten(), label = 'y')
    # plt.plot(np.array(new_dot_x[1, :]).flatten())
    # plt.plot(np.array(dot_x_arr[2, :]).flatten(), label = 'z')
    # plt.plot(np.array(new_dot_x[2, :]).flatten())

    # plt.plot(np.array(diff_matrix[0, :]).flatten(), label = 'e_x')
    # plt.plot(np.array(diff_matrix[1, :]).flatten(), label = 'e_y')
    # plt.plot(np.array(diff_matrix[2, :]).flatten(), label = 'e_z')
    plt.plot(w_array.flatten())
    plt.legend()
    plt.show()

if __name__ ==  "__main__":
    J_left = jacobian_solution(config_matrix, 6, left_ang)
    J_right = jacobian_solution(config_matrix, 6, right_ang)
    coop_J = coop_jacobian(J_left, J_right)

# w = manipulability_metrics(J)
# print(w)
# jacobian_test()