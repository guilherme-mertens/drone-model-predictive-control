"""
A Simple Pendulum Gate

# ----------------------
# p = pivot point
# c = center
# 1,2,3,4 = corners
# ----------------------
#           p           
#           |
#           |
#           |
#           |
#           |
#           |
#   2 - - - - - - - 1 
#   |               |
#   |       c       |
#   |               |
#   4 - - - - - - - 3 
#
"""

import numpy as np
from common.pend_index import *
from common.util import Point

class Pendulum_v2(object):
    #
    def __init__(self, center_point, dt):
        self.s_dim = 3
        self.a_dim = 0
        
        self._dt = dt
        self.center_point = center_point # e.g., np.array([2.0, 0.0, 2.0])
        
        self._state = np.zeros(shape=self.s_dim)
        self._state[:2] = self.center_point[:2]
        
        # initial state
        self._yaw_box = np.array([-1, 1]) * np.pi
        self._angular_velocity = 0.1
        self._radius = 1.5

        # self._dot_theta_box = np.array([0, 0]) * np.pi

        # self._theta_box = np.array([0.5, 0.5]) * self._pi
        # self._dot_theta_box = np.array([-0.0, 0.0]) * self._pi

        # x, y, z, roll, pitch, yaw, vx, vy, vz
        self.obs_low = np.array([-10, -10, -10, -np.pi, -np.pi, -np.pi, -10, -10, -10])
        self.obs_high = np.array([10, 10, 10, np.pi, np.pi, np.pi, 10, 10, 10])

        self.length = 2.0  # distance between pivot point to the gate center
        self.width = 1.0   # gate width (for visualization only)
        self.height = 0.5  # gate heiht (for visualization only)
            
        #
        self._init_corners()
        self.reset()
        self._t = 0.0
    
    def _init_corners(self):
        # compute distance between pivot point to four corners
        # and the 4 angles (for visualization)
        edge1, edge2 = self.width/2,  self.length-self.height/2
        self.length1 = np.sqrt( (edge1)**2 + (edge2)**2 )
        self.delta_theta1 = np.arctan2(edge1, edge2)
        #
        self.length2 = self.length1
        self.delta_theta2 = -self.delta_theta1

        #
        edge1, edge2 = self.width/2, self.length+self.height/2
        self.length3 = np.sqrt( (edge1)**2 + (edge2)**2 )
        self.delta_theta3 = np.arctan2(edge1, edge2)
        #
        self.length4 = self.length3
        self.delta_theta4 = -self.delta_theta3
        
    def reset(self, init_theta=None):
        if init_theta is not None:
            self._state[kYawZ] = init_theta
        else:
            self._state[kYawZ] = np.random.uniform( \
                low=self._yaw_box[0], high=self._yaw_box[1])
            
        self._t = 0.0
        # print("init pendulum: ", self._state)
        return self._state

    def run(self,):
        self._t = self._t + self._dt
        
        
        X = self._state
        X[kYawZ] = X[kYawZ] + self._angular_velocity * self._dt
        X[kPosX] = self._radius * np.cos(X[kYawZ])
        X[kPosY] = self._radius * np.sin(X[kYawZ])

        self._state = X
        return self._state
    
    def plan(self, T, dt_plan, sigma, t0=0):
        plans, pred_traj = [], []
        X = self._state.copy()

        # Jogando para frente o ponto inicial
        plus = 0.01
        X[kYawZ] = X[kYawZ] + (self._angular_velocity + plus) * 0.02 * t0/dt_plan
        X[kPosX] = self._radius * np.cos(X[kYawZ]) 
        X[kPosY] = self._radius * np.sin(X[kYawZ]) 




        for i in range(int(T/dt_plan)):
            X[kYawZ] = X[kYawZ] - self._angular_velocity * dt_plan # negatif
            X[kPosX] = self._radius * np.cos(X[kYawZ])
            X[kPosY] = self._radius * np.sin(X[kYawZ])

            state = X
            vx = -self._radius * self._angular_velocity * np.cos(X[kYawZ])
            vy = self._radius * self._angular_velocity * np.sin(X[kYawZ])
            

            traj_euler_point = [state[0], state[1], self.center_point[-1], 0, 0, state[-1] + np.pi/2, vx, vy, 0]

            # plan trajectory and optimal time & optimal vx
            traj_quat_point = [state[0], state[1], self.center_point[-1]] +  self.get_quaternion_planning(state[-1] + np.pi/2) + [vx, vy, 0]
            # traj_quat_point[kPosX] = opt_vx

            current_t = - i * dt_plan # negatif
            plan_i = traj_quat_point + [current_t, 1.0, sigma]

            #
            plans += plan_i
            pred_traj.append(traj_euler_point)

        return plans, pred_traj

    def get_quaternion(self, ):
        roll, pitch, yaw = self.get_euler()
        yaw = yaw + np.pi/2 
        #
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)  # 1
        sp = np.sin(pitch * 0.5)  # 0
        cr = np.cos(roll * 0.5)  # 1
        sr = np.sin(roll * 0.5)  # 0
        # 
        qw = cy * cp * cr + sy * sp * sr  # cy
        qx = cy * cp * sr - sy * sp * cr  # 0
        qy = sy * cp * sr + cy * sp * cr  # 0
        qz = sy * cp * cr - cy * sp * sr  # sy
        #
        return [qw, qx, qy, qz]

    def get_quaternion_planning(self, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp, sp, cr, sr = 1.0, 0.0, 1.0, 0.0
        qw = cy * cp * cr + sy * sp * sr  # cy
        qx = cy * cp * sr - sy * sp * cr  # 0
        qy = sy * cp * sr + cy * sp * cr  # 0
        qz = sy * cp * cr - cy * sp * sr  # sy

        return [qw, qx, qy, qz]





    def get_state(self,):
        return self._state
        
    def get_cartesian_state(self):
        cartesian_state = np.zeros(shape=9)
        cartesian_state[0:3] = self.get_position()
        cartesian_state[3:6] = self.get_euler()
        cartesian_state[6:9] = self.get_veloctiy()
        return cartesian_state
    
    def get_position(self,):
        pos = np.zeros(shape=3)
        pos[2] = self.center_point[2]
        pos[0:2] = self._state[:2]
        return pos
    
    def get_state(self,):
        state = self._state
        return state

    def get_veloctiy(self, ):
        vel = np.zeros(shape=3)
        vel[0] = -self._radius * self._angular_velocity * np.cos(self._state[kYawZ])
        vel[1] = self._radius * self._angular_velocity * np.sin(self._state[kYawZ])
        vel[2] = 0.0
        return vel

    def get_euler(self,):
        euler = np.zeros(shape=3)
        euler[0] = 0.0
        euler[1] = 0.0 
        euler[2] = self._state[kYawZ]
        return euler

    @property
    def t(self):
        return self._t

    @staticmethod
    def _to_planar_coordinates(pos, h, w, theta):
        x = pos[0] + w*np.cos(theta)
        y = pos[1] + w*np.sin(theta)
        z = pos[2] + h
        return x, y, z
    
    def t0(self, quad_pos):
        d = np.linalg.norm(np.array(quad_pos) - np.array(self.get_position()))
        return 0.1 * np.exp(-d/2)



    def get_3d_corners(self,):
        theta = self._state[kYawZ]
        pos = list(self._state[:2]) + [self.center_point[-1]]
        x1, y1, z1 = self._to_planar_coordinates(pos, -self.height/2, -self.width, theta)
        x2, y2, z2 = self._to_planar_coordinates(pos, -self.height/2, self.width, theta)
        x3, y3, z3 = self._to_planar_coordinates(pos, self.height/2, -self.width, theta)
        x4, y4, z4 = self._to_planar_coordinates(pos, self.height/2, self.width, theta)
        #
        corners_3d = [[x1, y1, z1], [x2, y2, z2 ], [x3, y3, z3 ], [x4, y4, z4]]
        return corners_3d




if __name__ == "__main__":
    # test run
    import matplotlib.pyplot as plt
    dt = 0.02
    tf = 1.0
    #
    pivot = [5.0, 0.0, 0.0] # x, y, z

    # # # # # # # # # # # # # # # # # # #
    # -- test Pendulum v0
    # # # # # # # # # # # # # # # # # #
    env = Pendulum_v2(pivot, dt=0.02)
    l_t, l_pos, l_vel, l_theta  = [], [], [], []
    #
    env.reset(0.0)
    plt.figure()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.axis('equal')
    while env.t < tf:
        pose = env.get_state() + np.array([0, 0, np.pi / 2])
        print(pose)
        plt.arrow(pose[0], pose[1], 0.5 * np.cos(pose[2]), 0.5 * np.sin(pose[2]), head_width=0.2, head_length=0.3, fc='red', ec='red')
        plt.pause(0.01)
        #
        env.run()




        
