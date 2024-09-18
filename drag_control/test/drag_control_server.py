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
from scipy.optimize import brentq
from utils import *

class Drag_Control():
    def __init__(self, config=None):
        # initialize constant
        self.deg2rad = 0.0174533
        self.rad2deg = 57.2958
        self.pi = np.pi
        self.gravity = 9.80665

        default_config = {
            'object_weight': 1.0,
            'major_axis': 0.001,
            'minor_axis': 10,
            'eq_radius_o': None,
            'eq_radius_h': 0.01,
            'mu1': 0.15,
            'mu2': 0.8,
            'Ow': None,
            'Hw': 30.0,
            'c_o': 0.6,
            'c_h': 0.6,
            'c_p': 0.9642,
            'delta': 1.324,
            'start_t': 0.0,
            'delta_t': 0.1,
            'finish_t': 30.0
        }
        
        # update custom config dictionary
        if config:
            default_config.update(config)
        
        # update dependent variable
        default_config['eq_radius_o'] = np.sqrt(default_config['major_axis'] * default_config['minor_axis'])
        default_config['Ow'] = self.gravity * default_config['object_weight']
        
        self.object_weight = default_config['object_weight']
        self.major_axis = default_config['major_axis']
        self.minor_axis = default_config['minor_axis']
        self.eq_radius_o = default_config['eq_radius_o']
        self.eq_radius_h = default_config['eq_radius_h']
        self.mu1 = default_config['mu1']
        self.mu2 = default_config['mu2']
        self.Ow = default_config['Ow']
        self.Hw = default_config['Hw']
        self.c_o = default_config['c_o']
        self.c_h = default_config['c_h']
        self.c_p = default_config['c_p']
        self.delta = default_config['delta']
        self.start_t = default_config['start_t']
        self.delta_t = default_config['delta_t']
        self.finish_t = default_config['finish_t']
        self.time_Simul = np.arange(self.start_t, self.finish_t, self.delta_t)
        self.velocity_candidate = np.empty((1,3), dtype=np.float32)

        # position of object and hand
        self.init_q_o = np.array([  0.0, 0.0, 0.0]).T
        self.init_q_h = np.array([-0.05, 0.0, 0.0]).T

        self.q_o = copy.deepcopy(self.init_q_o)
        self.q_h = copy.deepcopy(self.init_q_h)
        self.q_rel = get_rotation(-self.q_o[2]) @ (self.q_h - self.q_o)

    def update_limit_surface_A(self):
        # define a 3x3 positive definite matrix A
        element = np.array([self.mu1*(self.Ow + self.Hw), self.mu1*(self.Ow + self.Hw), self.eq_radius_o*self.c_o*self.mu1*(self.Ow + self.Hw)])
        self.A_cop = np.diag(element)
        self.A_cop = inv(self.A_cop)**2

        # consider shift of pressure becasuse of patch
        # parameter can be changed to fit the experimental data
        s = 1 - np.power(self.c_p*self.Hw/self.Ow + 1, -self.delta)
        self.A = get_jacobian(-s*self.q_rel[0], -s*self.q_rel[1]) @ self.A_cop @ get_jacobian(-s*self.q_rel[0], -s*self.q_rel[1]).T

    def update_limit_surface_B(self):
        # define a 3x3 positive definite matrix B
        element = np.array([self.mu2 * self.Hw, self.mu2 * self.Hw, self.eq_radius_h * self.c_h * self.mu2 * self.Hw])
        self.B = np.diag(element)
        self.B = inv(self.B)**2

    def object_velocity_calculation(self, q_h_dot):
        v_h = get_rotation(self.q_h[2]).T @ get_jacobian(self.q_h[0], self.q_h[1]) @ q_h_dot
        v_bar_h = self.phi.T @ v_h

        # ========== MODE SELECTION ALGORITHM ==========
        if np.linalg.norm(v_h) == 0:
            # sticking mode
            if np.all((self.lmda - 1) > 0) is True:
                mode = 0
            # slipping mode
            elif np.all((self.lmda - 1) < 0) is True:
                mode = 1
            # pivoting mode
            else:
                mode = 2
        else:
            # sticking mode
            if v_bar_h.T @ self.C @ v_bar_h < 0:
                mode = 0
            # slipping mode
            elif v_bar_h.T @ self.C @ inv(self.lmda)**2 @ v_bar_h >= 0:
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
            v_o = inv(self.G) @ v_h
            v_rel = np.array([0.0, 0.0, 0.0]).T

        # slipping mode
        elif mode == 1:
            v_o = np.array([0.0, 0.0, 0.0]).T
            v_rel = v_h

        # pivoting mode
        else:
            eq = Equation(v_bar_h, self.lmda, self.C)

            try:
                alpha = brentq(eq.equation, 0, 500)
                v_o = inv(self.G) @ inv(np.eye(3) + alpha * self.B @ inv(self.A_dot)) @ v_h
            except ValueError as e:
                print("previous velocity will be used") 

        q_o_dot = get_rotation(self.q_o[2]) @ v_o
        self.velocity_candidate = np.vstack((self.velocity_candidate, q_o_dot))
        return q_o_dot

    def drag_velocity_candidate(self):
        drag_velocity_candidate = []
        for force in range(3,18):
            self.Hw = force
            self.update_limit_surface_A()
            self.update_limit_surface_B()
            self.G = get_rotation(self.q_rel[2]).T @ get_jacobian(self.q_rel[0], self.q_rel[1])
            self.A_dot = self.G @ self.A @ self.G.T

            # generalized eigenvalue decomposition
            eigen_values, eigen_vectors = eigh(self.B, self.A_dot)
            self.lmda = np.diag(eigen_values)
            self.phi = eigen_vectors
            self.C = self.lmda - np.eye(3)
        
            angles = np.arange(0, 2*np.pi, 2*np.pi/20)
            
            for theta in angles:
                q_h_dot = np.array([0.05 * np.cos(theta), 0.05 * np.sin(theta), 0.0]).T
                q_o_dot = self.object_velocity_calculation(q_h_dot)
                drag_velocity_candidate.append([[0, 0, force], q_h_dot, q_o_dot])

        return np.array(drag_velocity_candidate)
    
    def run(self):
        velocity_candidate = self.drag_velocity_candidate()
        # choose velocity index
        idx = 7
        self.q_h = self.q_h + velocity_candidate[idx][1] * self.delta_t
        self.q_o = self.q_o + velocity_candidate[idx][2] * self.delta_t
        self.q_rel = get_rotation(-self.q_o[2]) @ (self.q_h - self.q_o)
        
if __name__ == '__main__':
    drag_interface = Drag_Control()
    
    drag_interface.run()

    show_possible_velocity(drag_interface.velocity_candidate)