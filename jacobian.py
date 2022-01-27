from forward_kinematics import forward_kinematics_solution, transformation_matrix
import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

config_matrix = np.matrix([[0, pi/2.0, 0.1625],
                        [-0.425, 0, 0],
                        [-0.3922, 0, 0],
                        [0, pi/2.0, 0.1333],
                        [0, -pi/2.0, 0.0997],
                        [0, 0, 0.0996]])

ed = np.matrix([[1.572584629058838], [-1.566467599277832], [-0.0026149749755859375], [-1.568673924808838],
                    [-0.009446446095601857], [0.007950782775878906]])

def jacobian_solution(config_matrix, joints, theta_angles):

    jacobian = np.matrix(np.zeros((6, joints)))
    p6 = forward_kinematics_solution(config_matrix, joints, theta_angles)[:3,3]
    cur_z = np.matrix([[0],[0],[1]])
    cur_p = np.matrix([[0],[0],[0]])

    jacobian[:3,0] = np.cross(cur_z.T,(p6 - cur_p).T).T
    jacobian[3:,0] = cur_z

    for i in range(1, joints):
        tm = forward_kinematics_solution(config_matrix, i, theta_angles)
        jacobian[:3,i] = np.cross(tm[:3,2].flatten(),(p6 - tm[:3,3]).flatten()).T
        jacobian[3:,i] = tm[:3,2]


    # print(jacobian)
    return jacobian

def jacobian_test():
    T = 0.05
    time = np.arange(1, 16, T)
    q = np.zeros((len(time), 3))
    dot_q = np.zeros((len(time),3))
    x = np.matrix(np.zeros((3,len(q))))
    dot_x = np.matrix(np.zeros((3,len(q))))

    sin_arr = np.sin(time)
    cos_arr = np.cos(time)

    for i in (1,3,5):
        q = np.insert(q, i, sin_arr, axis=1)
        dot_q = np.insert(dot_q, i, cos_arr, axis=1)

    for i in range(len(q)):
        x[:,i] = forward_kinematics_solution(config_matrix, 6, q[i])[:3,3]
        
    new_dot_x = np.diff(x)/T
    jacobian_dot_x = np.matrix(np.zeros((6, len(time))))


    # print(x)
    # print(np.array(x[0, :]).flatten())
    # plt.grid(True)
    # plt.plot(np.array(x[0, :]).flatten())
    # plt.plot(np.array(new_dot_x[0, :]).flatten())
    # plt.show()

    dot_x_arr = []
    for i in range(len(q)-1):
        jacobian_dot_x[:,i] = jacobian_solution(config_matrix, 6, q[i])*np.matrix(dot_q[i]).T
        dot_x_arr.append(jacobian_dot_x[:,i])

    # print(np.array_equal(jacobian_dot_x[:3,:], new_dot_x))

    # print(dot_x)
    # print("kek")
    # print(new_dot_x)
    # print(dot_x_arr)
    # dot_x_arr = np.array(dot_x_arr)[:, :, 0].T
    # print(dot_x_arr)
    dot_x_arr = np.array(jacobian_dot_x)
    plt.grid(True)
    plt.plot(np.array(dot_x_arr[0, :]).flatten())
    plt.plot(np.array(new_dot_x[0, :]).flatten())
    plt.show()

    #print(dot_q)
    #print(len(time))
# jacobian_solution(config_matrix, 6, ed)
jacobian_test()