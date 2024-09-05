# Modeling for quasi-static analysis of planar sliding
# Assume ellipsoid object with major axis(10.0cm) and minor axis(10.0cm) lie on (0,0) on general coordinate
# Object weights 1kg and exerts uniform pressure on surface
# Friction coefficient between surface and object is 0.2. 
# Friction coefficient between object and hand is 0.8
# Hand exerts 7N with Hertzian pressure and circular contact area radius is 2.0cm

import numpy as np
import copy
from scipy.linalg import eig
from utils import *

# parameter for simulation
deg2rad = 0.0174533
rad2deg = 57.2958
pi = np.pi

gravity = 9.80665
object_weight = 1.0

major_axis = 0.10
minor_axis = 0.05

eq_radius_o = np.sqrt(major_axis * minor_axis)
eq_radius_h = 0.01

mu1 = 0.2
mu2 = 0.8

Ow  = gravity * object_weight
Hw  = 7.0

c_o = 0.6
c_h = 0.6

# define a 3x3 positive definite matrix A
element = np.array([mu1*(Ow + Hw), mu1*(Ow + Hw), eq_radius_o*c_o*mu1*(Ow + Hw)])
A_cop = np.diag(element)
A_cop = np.linalg.inv(A_cop)**2

# consider shift of pressure becasuse of patch
# parameter can be changed to fit the experimental data
c_p = 0.9642
delta = 1.324
s = 1 - np.power(c_p*Hw/Ow + 1, -delta)

# define a 3x3 positive definite matrix B
element = np.array([mu2*Hw, mu2*Hw, eq_radius_h*c_h*mu2*Hw])
B = np.diag(element)
B = np.linalg.inv(B)**2

# position of object and hand
init_q_o = np.array([  0.0, 0.0, 0.0]).T
init_q_h = np.array([-0.05, 0.0, 0.0]).T

q_o = copy.deepcopy(init_q_o)
q_h = copy.deepcopy(init_q_h)
q_rel = get_rotation(-q_o[2]) @ (q_h - q_o)

# simulation implement
n=1
flag_Simul = 1
flag_Draw  = 1

start_t = 0.0
delta_t = 30.0
finish_t = 30.0

time_Simul = np.arange(start_t, finish_t, delta_t)

if flag_Simul == True:
    for time in time_Simul:
        # quasi static analysis kinematics algorithm for simulation
        # generalized eigenvalue decomposition

        # A denote LS at the object frame
        A = get_jacobian(-s*q_rel[0], -s*q_rel[1]) @ A_cop @ get_jacobian(-s*q_rel[0], -s*q_rel[1]).T
        G = get_rotation(q_rel[2]).T @ get_jacobian(q_rel[0], q_rel[1])

        # A_dot denote LS at hand frame
        A_dot = G @ A @ G.T
        
        eigen_values, eigen_vectors = eig(A_dot, B)
        lmda = np.diag(eigen_values)
        phi = eigen_vectors

        # print(phi)
        # print(lmda)
        # print("first", phi.T)
        # print(phi @ B @ phi )
        # print("same?", phi.T)
        # print(phi.T @ B @ phi)
        # print(phi.T @ A_dot @ phi)

        # print("same??")
        # print(A_dot@phi)
        # print(B @ phi @ lmda)

        # print(-s*q_rel[0], -s*q_rel[1])
        # print(get_jacobian(-s*q_rel[0], -s*q_rel[1]))
        # print(A_cop)
        # print(A)