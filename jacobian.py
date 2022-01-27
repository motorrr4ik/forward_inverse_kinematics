from forward_kinematics import forward_kinematics_solution, transformation_matrix
import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

config_matrix = np.array([[0, pi/2.0, 0.1625],
                        [-0.425, 0, 0],
                        [-0.3922, 0, 0],
                        [0, pi/2.0, 0.1333],
                        [0, -pi/2.0, 0.0997],
                        [0, 0, 0.0996]])

ed = np.array([[1.572584629058838], [-1.566467599277832], [-0.0026149749755859375], [-1.568673924808838],
                    [-0.009446446095601857], [0.007950782775878906]])

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

def jacobian_test():
    T = 0.05
    time = np.arange(1, 16, T)
    q = np.zeros((len(time), 6))
    dot_q = np.zeros((len(time),6))
    x = np.array(np.zeros((3,len(q))))
    dot_x = np.array(np.zeros((3,len(q))))

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

    dot_x_arr = []
    for i in range(len(q)-1):
        jacobian_dot_x[:,i] = jacobian_solution(config_matrix, 6, q[i]).dot(np.array(dot_q[i]).T)
        dot_x_arr.append(jacobian_dot_x[:,i])

    # print(jacobian_dot_x[:3,:3])
    # print(new_dot_x[:3,:3])
    # print(dot_x)
    # print("kek")
    # print(new_dot_x)
    # print(dot_x_arr)
    # dot_x_arr = np.array(dot_x_arr)[:, :, 0].T
    # print(dot_x_arr)
    diff_matrix = jacobian_dot_x[:3, :299] - new_dot_x
    dot_x_arr = np.array(jacobian_dot_x)
    plt.grid(True)

    plt.plot(np.array(dot_x_arr[0, :]).flatten(), label = 'x')
    plt.plot(np.array(new_dot_x[0, :]).flatten())
    plt.plot(np.array(dot_x_arr[1, :]).flatten(), label = 'y')
    plt.plot(np.array(new_dot_x[1, :]).flatten())
    plt.plot(np.array(dot_x_arr[2, :]).flatten(), label = 'z')
    plt.plot(np.array(new_dot_x[2, :]).flatten())

    plt.plot(np.array(diff_matrix[0, :]).flatten(), label = 'e_x')
    plt.plot(np.array(diff_matrix[1, :]).flatten(), label = 'e_y')
    plt.plot(np.array(diff_matrix[2, :]).flatten(), label = 'e_z')
    plt.legend()
    plt.show()

# jacobian_solution(config_matrix, 6, ed)
jacobian_test()