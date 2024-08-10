import numpy as np
#
from simulation.quadrotor import Quadrotor_v0
from simulation.pendulum_v0 import Pendulum_v0
from simulation.pendulum_v1 import Pendulum_v1
from simulation.pendulum_v2 import Pendulum_v2
#
from common.quad_index import *

#
class Space(object):

    def __init__(self, low, high):
        self.low = low
        self.high = high
        self.shape = self.low.shape

    def sample(self):
        return np.random.uniform(self.low, self.high)

class DynamicGap(object):

    def __init__(self, mpc, plan_T, plan_dt):
        #
        self.mpc = mpc
        self.plan_T = plan_T
        self.plan_dt = plan_dt

        # 
        self.pivot_point1 = np.array([5.0, 0.0, 1.0])
        self.pivot_point2 = np.array([-2.5, 4.33, 1.0])
        self.pivot_point3 = np.array([-2.5, -4.33, 1.0])
        self.goal_point = self.pivot_point1
        



        # simulation parameters ....
        self.sim_T = 10.0    # Episode length, seconds
        self.sim_dt = 0.02  # simulation time step
        self.max_episode_steps = int(self.sim_T/self.sim_dt)
        # Simulators, a quadrotor and a pendulum
        self.quad = Quadrotor_v0(dt=self.sim_dt)
        self.pend1 = Pendulum_v2(self.pivot_point1, dt=self.sim_dt)
        self.pend2 = Pendulum_v2(self.pivot_point2, dt=self.sim_dt)
        self.pend3 = Pendulum_v2(self.pivot_point3, dt=self.sim_dt)

        #self.planner = Pendulum_v1(pivot_point=self.pivot_point, sigma=10, \
        #    T=self.plan_T, dt=self.plan_dt)
    

        #
        # self.observation_space = Space(
        #    low=np.array([-10.0, -10.0, -10.0, -2*np.pi, -2*np.pi, -2*np.pi, -10.0, -10.0, -10.0]),
        #    high=np.array([10.0, 10.0, 10.0, 2*np.pi, 2*np.pi, 2*np.pi, 10.0, 10.0, 10.0]),
        # )
        self.observation_space = Space(
            low=np.array([-20.0, -20.0, -10.0, -2*np.pi, -2*np.pi, -2*np.pi, -10.0, -10.0, -10.0]),
            high=np.array([20.0, 20.0, 10.0, 2*np.pi, 2*np.pi, 2*np.pi, 10.0, 10.0, 10.0]),
        )

        self.action_space = Space(
            low=np.array([0.0]),
            high=np.array([2*self.plan_T])
        )

        # reset the environment
        self.t = 0
        self.current_traj = 1
        self.reset()
    
    def seed(self, seed):
        np.random.seed(seed=seed)
    
    def reset(self, init_theta=None):
        self.t = 0
        # state for ODE
        self.quad_state = self.quad.reset()
        if init_theta is not None:
            self.pend_state = self.pend1.reset(0.0)
            self.pend_state2 = self.pend2.reset(2*np.pi/3)
            self.pend_state3 = self.pend3.reset(4*np.pi/3)
        else:
            self.pend_state = self.pend1.reset(0.0)
            self.pend_state2 = self.pend2.reset(2*np.pi/3)
            self.pend_state3 = self.pend3.reset(4*np.pi/3)
        
        # observation, can be part of the state, e.g., postion
        # or a cartesian representation of the state
        quad_obs = self.quad.get_cartesian_state()
        pend_obs = self.pend1.get_cartesian_state()
        #
        obs = (quad_obs - pend_obs).tolist()
        
        return obs

    def step(self, u=0):
        self.t += self.sim_dt
        opt_t = u
        
        #
        if self.current_traj == 1:
            quad_s0 = self.quad_state.tolist()
            t0 = self.pend1.t0(quad_s0[:3])
            plan_pend_traj, pred_pend_traj_cart = self.pend1.plan(self.plan_T, self.plan_dt, 10, t0)

            pred_pend_traj_cart = np.array(pred_pend_traj_cart)
        
            #
            obj_state = self.pend1.get_position().tolist() + self.pend1.get_quaternion() + [0.0, 0.0, 0.0] 
            goal = plan_pend_traj[-13:-6] + list(np.array(plan_pend_traj[-6:-3])*4)
            ref_traj = quad_s0 + plan_pend_traj + obj_state


        if self.current_traj == 2:
            quad_s0 = self.quad_state.tolist()
            t0 = self.pend2.t0(quad_s0[:3])
            plan_pend_traj, pred_pend_traj_cart = self.pend2.plan(self.plan_T, self.plan_dt, 10, t0)

            pred_pend_traj_cart = np.array(pred_pend_traj_cart)
        
            #
            obj_state = self.pend2.get_position().tolist() + self.pend2.get_quaternion() + [0.0, 0.0, 0.0]
            # obj_state = plan_pend_traj[-13:-6] + [0.0, 0.0, 0.0]
            goal = plan_pend_traj[-13:-6] + list(np.array(plan_pend_traj[-6:-3]))
            # obj_state = plan_pend_traj[-13:-6] + list(np.array(quad_s0[-3:])*2)
            #print(self.pend2.get_veloctiy())
            #print(quad_s0[-3:])
            #print("===========")
            ref_traj = quad_s0 + plan_pend_traj + obj_state

        if self.current_traj == 3:
            quad_s0 = self.quad_state.tolist()
            t0 = self.pend3.t0(quad_s0[:3])
            plan_pend_traj, pred_pend_traj_cart = self.pend3.plan(self.plan_T, self.plan_dt, 10, t0)

            pred_pend_traj_cart = np.array(pred_pend_traj_cart)
        
            #
            obj_state = self.pend3.get_position().tolist() + self.pend3.get_quaternion() + [0.0, 0.0, 0.0] 
            goal = plan_pend_traj[-13:-6] + list(np.array(plan_pend_traj[-6:-3])*4)
            ref_traj = quad_s0 + plan_pend_traj + obj_state

        print(quad_s0)

        # run nonliear model predictive control
        quad_act, pred_traj = self.mpc.solve(ref_traj)
        print(len(pred_traj))
        print(pred_traj[0:10])

        # run the actual control command on the quadrotor
        self.quad_state = self.quad.run(quad_act)
        # print("vz: " + str(self.quad_state[-1]))
        # simulate one step pendulum
        self.pend_state = self.pend1.run()
        self.pend_state2 = self.pend2.run()
        self.pend_state3 = self.pend3.run()
        
        # update the observation.
        quad_obs = self.quad.get_cartesian_state()
        pend_obs = self.pend1.get_cartesian_state()
        pend_obs2 = self.pend2.get_cartesian_state()
        pend_obs3 = self.pend3.get_cartesian_state()
        obs = (quad_obs - pend_obs).tolist()
        #
        info = {
            "quad_obs": quad_obs, 
            "quad_act": quad_act, 
            "quad_axes": self.quad.get_axes(),
            "pend_obs": pend_obs,
            "pend_obs2": pend_obs2,
            "pend_obs3": pend_obs3,
            "pend_corners": self.pend1.get_3d_corners(),
            "pend_corners2": self.pend2.get_3d_corners(),
            "pend_corners3": self.pend3.get_3d_corners(),
            "pred_quad_traj": pred_traj, 
            "pred_pend_traj": pred_pend_traj_cart, 
            "opt_t": opt_t, "plan_dt": self.plan_dt}
        done = False

        if np.linalg.norm(obj_state[:3] - quad_obs[:3]) < 0.4:
            self.current_traj = (self.current_traj % 3) + 1


        if self.t >= (self.sim_T-self.sim_dt):
            done = True

        return obs, 0, done, info
    
    def episode(self, u):
        opt_t = u
        #
        plan_pend_traj, pred_pend_traj_cart = self.planner.plan(self.pend_state, opt_t)
        pred_pend_traj_cart = np.array(pred_pend_traj_cart)
        
        #
        quad_s0 = self.quad_state.tolist()
        ref_traj = quad_s0 + plan_pend_traj + self.quad_sT
    
        _, pred_traj = self.mpc.solve(ref_traj)
        
        opt_node = np.clip( int(opt_t/self.plan_dt), 0, pred_traj.shape[0]-1)
        # if quad_s0[kPosX] >= self.pivot_point[kPosX]+0.5:
        #     # obs = self.reset()
        #     loss = np.linalg.norm(pred_traj[opt_node, kPosX:kPosZ+1] - np.tile(self.goal_point, (pred_traj.shape[0], 1)))
        #     rew = - np.mean(loss)             
        # else:    
        opt_min = np.clip(opt_node-10, 0, pred_traj.shape[0]-1)
        opt_max = np.clip(opt_node+5, 0, pred_traj.shape[0]-1)
        # opt_min = np.clip(opt_node-1, 0, pred_traj.shape[0]-1)
        # opt_max = np.clip(opt_node+1, 0, pred_traj.shape[0]-1)
        #
        loss = np.linalg.norm(pred_traj[opt_min:opt_max, kPosX:kPosZ+1]  - pred_pend_traj_cart[opt_min:opt_max, kPosX:kPosZ+1])
        rew = - loss
        #    
        return rew, opt_node
    
    @staticmethod
    def _is_within_gap(gap_corners, point):
        A, B, C = [], [], []    
        for i in range(len(gap_corners)):
            p1 = gap_corners[i]
            p2 = gap_corners[(i + 1) % len(gap_corners)]
            
            # calculate A, B and C
            a = -(p2.y - p1.y)
            b = p2.x - p1.x
            c = -(a * p1.x + b * p1.y)

            A.append(a)
            B.append(b)
            C.append(c)
        D = []
        for i in range(len(A)):
            d = A[i] * point.x + B[i] * point.y + C[i]
            D.append(d)

        t1 = all(d >= 0 for d in D)
        t2 = all(d <= 0 for d in D)
        return t1 or t2

    def close(self,):
        return True

    def render(self,):
        return False
    