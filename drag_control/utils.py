import numpy as np

def get_rotation(theta):
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta),  np.cos(theta), 0],
                                [            0,              0, 1]])
    return rotation_matrix

def get_jacobian(x_r, y_r):
    
    jacobian_matrix = np.eye(3)
    jacobian_matrix[0, 2] = -y_r
    jacobian_matrix[1, 2] =  x_r

    return jacobian_matrix
