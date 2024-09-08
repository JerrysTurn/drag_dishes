# Modeling for quasi-static analysis of planar sliding
# Assume ellipsoid object with major axis(10.0cm) and minor axis(10.0cm) lie on (0,0) on general coordinate
# Object weights 1kg and exerts uniform pressure on surface
# Friction coefficient between surface and object is 0.2. 
# Friction coefficient between object and hand is 0.8
# Hand exerts 7N with Hertzian pressure and circular contact area radius is 2.0cm

import numpy as np
import copy

from numpy.linalg import inv
from scipy.linalg import eigh
from scipy.optimize import fsolve
from scipy.optimize import brentq
from scipy.optimize import bisect
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

mu1 = 0.15
mu2 = 0.8

Ow  = gravity * object_weight
Hw  = 2.5

c_o = 0.6
c_h = 0.6

# define a 3x3 positive definite matrix A
element = np.array([mu1*(Ow + Hw), mu1*(Ow + Hw), eq_radius_o*c_o*mu1*(Ow + Hw)])
A_cop = np.diag(element)
A_cop = inv(A_cop)**2

# consider shift of pressure becasuse of patch
# parameter can be changed to fit the experimental data
c_p = 0.9642
delta = 1.324
s = 1 - np.power(c_p*Hw/Ow + 1, -delta)

# define a 3x3 positive definite matrix B
element = np.array([mu2*Hw, mu2*Hw, eq_radius_h*c_h*mu2*Hw])
B = np.diag(element)
B = inv(B)**2

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
delta_t = 0.01
finish_t = 30.0

time_Simul = np.arange(start_t, finish_t, delta_t)

velocity_candidate = np.empty((1,3), dtype=np.float32)
count = 0

if flag_Simul == True:
    for time in time_Simul:
        # quasi static analysis kinematics algorithm for simulation
        # generalized eigenvalue decomposition

        # A denote LS at the object frame
        A = get_jacobian(-s*q_rel[0], -s*q_rel[1]) @ A_cop @ get_jacobian(-s*q_rel[0], -s*q_rel[1]).T
        G = get_rotation(q_rel[2]).T @ get_jacobian(q_rel[0], q_rel[1])

        # A_dot denote LS at hand frame
        A_dot = G @ A @ G.T
        
        eigen_values, eigen_vectors = eigh(B, A_dot)
        lmda = np.diag(eigen_values)
        phi = eigen_vectors

        C = lmda - np.eye(3)

        # ========== MODE SELECTION ALGORITHM ==========
        theta = np.arange(0, 2*np.pi, 2*np.pi/len(time_Simul))
        q_h_dot = np.array([0.05 * np.cos(theta[count]), 0.05 * np.sin(theta[count]), 0.0]).T
        count += 1
        # q_h_dot = np.array([0.0, 0.02, 0.0]).T
        # v_h = get_rotation(q_h[2]).T @ q_h_dot
        v_h = get_rotation(q_h[2]).T @ get_jacobian(q_h[0], q_h[1]) @ q_h_dot
        v_bar_h = phi.T @ v_h

        if np.linalg.norm(q_h) == 0:
            # sticking mode
            if np.all((eigen_values - 1) > 0) is True:
                mode = 0
            # slipping mode
            elif np.all((eigen_values - 1) < 0) is True:
                mode = 1
            # pivoting mode
            else:
                mode = 2

        else:
            # sticking mode
            if v_bar_h.T @ C @ v_bar_h < 0:
                mode = 0
            # slipping mode
            elif v_bar_h.T @ C @ inv(lmda)**2 @ v_bar_h >= 0:
                mode = 1
            # pivoting mode
            else:
                mode = 2

        # ========== VELOCITY CALCULATION ==========
        print("mode: ", mode)

        class Equation:
            def __init__(self, v_bar_h, lmda, C):
                self.v_bar_h = v_bar_h
                self.lmda = lmda
                self.C = C

            def equation(self, alpha):
                return (self.C[0,0] * (self.v_bar_h[0] / (alpha * self.lmda[0,0] + 1))**2 + \
                        self.C[1,1] * (self.v_bar_h[1] / (alpha * self.lmda[1,1] + 1))**2 + \
                        self.C[2,2] * (self.v_bar_h[2] / (alpha * self.lmda[2,2] + 1))**2)

        # sticking mode
        if mode == 0:
            v_o = inv(G) @ v_h
            v_rel = np.array([0.0, 0.0, 0.0]).T

        # slipping mode
        elif mode == 1:
            v_o = np.array([0.0, 0.0, 0.0]).T
            v_rel = v_h

        # pivoting mode
        else:
            eq = Equation(v_bar_h, lmda, C)

            try:
                alpha = brentq(eq.equation, 0, 500)
            except ValueError as e:
                print("previous velocity will be used")

            v_o = inv(G) @ inv(np.eye(3) + alpha * B @ inv(A_dot)) @ v_h

        velocity_candidate = np.vstack((velocity_candidate, v_o))

# show_limit_surface(eigen_values)
show_possible_velocity(velocity_candidate)