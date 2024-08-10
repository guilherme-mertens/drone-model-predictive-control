#!/usr/bin/python3
'''
    CentraleSupelec TP 2A/3A
    Aarsh THAKKER,2023
    (all variables in SI unit)

###########################################################################################
============================ READ THIS BEFORE STARTING TO CODE ============================
    You ONLY modify the part that is marked "to be modified" in the functions
    variables used by the functions of this script
        - robotNo: no of the current robot of same type for which control is counted (1 .. nbRobots)
        - nbTB3: number of total tb3 robots in the fleet (>=0)
        - nbCF: number of total crazyflie nano drones in the fleet (>=0)
        - nbRMTT: number of total dji robomaster TT drones in the fleet (>=0)
        - nbRMS1: number of total dji robomaster S1 in the fleet (>=0)  (YOU CAN ONLY CONTROL THIS ROBOT MANUALLY USING YOUR MOBILE PHONE AND GET POSITION IN HERE TO BE USED)
        - nbOBSTACLE: number of total obstacle positions in the environment (>=0)
        - tb3_poses:  size (3 x nbTB3) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    tb3_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    tb3_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    tb3_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    tb3_poses[2,robotNo-1]: orientation angle of robot (in rad)
        - cf_poses:  size (3 x nbCF) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    cf_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    cf_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    cf_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    cf_poses[2,robotNo-1]: z-coordinate of robot position (in m)
        - rmtt_poses:  size (3 x nbRMTT) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    rmtt_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    rmtt_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    rmtt_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    rmtt_poses[2,robotNo-1]: z-coordinate of robot position (in m)
        - rms1_poses:  size (3 x nbRMS1) 
            eg. of use: for robot number 'robotNo', position of the robot can be obtained by: 
                    tb3_poses[:,robotNo-1]   (indexes in Python start from 0 !)
                    tb3_poses[0,robotNo-1]: x-coordinate of robot position (in m)
                    tb3_poses[1,robotNo-1]: y-coordinate of robot position (in m)
                    tb3_poses[2,robotNo-1]: orientation angle of robot (in rad)
        - obstacle_pose:  size (5 x nbOBSTACLE)  
                    This can be used to define cube/sphere shaped obstacle in the environment.
                    obstacle_pose[:,nbOBSTACLE-1]   (indexes in Python start from 0 !)
                    obstacle_pose[0,nbOBSTACLE-1]: x-coordinate of center position of obstacle (in m)
                    obstacle_pose[1,nbOBSTACLE-1]: y-coordinate of center position of obstacle (in m)
                    obstacle_pose[2,nbOBSTACLE-1]: z-coordinate of center position of obstacle (in m)
                    obstacle_pose[3,nbOBSTACLE-1]: size of the obstacle (from center to the side of the cube/radius of sphere)
                    obstacle_pose[4,nbOBSTACLE-1]: type of obstacle (0= sphere, 1= cube)


###########################################################################################

'''

import numpy as np
import math
import rospy

# ==============   "GLOBAL" VARIABLES KNOWN BY ALL THE FUNCTIONS ===================
# all variables declared here will be known by functions below
# use keyword "global" inside a function if the variable needs to be modified by the function

first_call = True




# MAIN
from simulation.dynamic_gap import DynamicGap
from mpc.mpc import MPC

import numpy as np
from simulation.quadrotor import Quadrotor_v0
from simulation.pendulum_v2 import Pendulum_v2
from common.quad_index import *

plan_T = 1.0   # Prediction horizon for MPC and local planner
plan_dt = 0.1 # Sampling time step for MPC and local planner
so_path = "mpc/saved/mpc_v1.so" # saved mpc model (casadi code generation)

mpc = MPC(T=plan_T, dt=plan_dt, so_path=so_path)
env = None


radius = 0.5
omega = 0.1


def firstDroneCall(initial_point1, initial_point2, initial_point3):
    global env, first_call, mpc, plan_T, plan_dt
    env = DynamicGap(mpc, plan_T, plan_dt, initial_point1, initial_point2, initial_point3)
    env.reset()
    first_call = False
    return True #takeoff

def otherDroneCalls(robots_pose1, robots_pose2, robots_pose3, drone_pose, t):
    # step
    info, velocities = env.step(robots_pose1, robots_pose2, robots_pose3, drone_pose, t)
    return velocities



# ===================================================================================

# Control function for turtlebot3 ground vehicle to be modified
# should ONLY return (vx,vy) for the robot command
# max useable numbers of robots = 6 (or 1 for waffle model)
# ====================================
def tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
# ====================================

    nbTB3= len(tb3_poses[0]) # number of total tb3 robots in the use
    nbCF = len(cf_poses[0]) # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0]) # number of total dji rms1 in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    ## Convertion to control_algo.py's format
    N = nbTB3 # number of robots (short notation)
    i = robotNo - 1 # get index of current robot  (short notation)
    x = tb3_poses[0:2,:].T # get positions of all robots
    t = ros_time.to_time()

    xi = x[i]
    x0 = x[0]
    vx = 0.0
    vy = 0.0

    
    #  ----------------- TO BE MODIFIED ---------------------- 
    # Adjacencdy matrix of communication graph
    A = np.ones((N, N)) - np.eye(N)

    # Formation shape
    global radius, omega
    rx = radius
    ry = radius
    angles = np.linspace(0, 2*np.pi, N, endpoint=False) + omega*t
    vertices = [[rx*np.cos(angle), ry*np.sin(angle)] for angle in angles]
    xfref = np.array(vertices)

    rref = np.zeros((N,N,2))
    for k in range(N):
        for j in range(N):
            rref[k,j,:] = xfref[k,:] - xfref[j,:]
        
    rrefd = 0
    
    # Control partition
    ui = np.zeros(2)
    kp = 0.1
    
    # Control law
    ui = -np.sum(kp*A[i,:]*(x[i,:]-x-rref[i,:,:]).T, axis=1)     
    # -------------------------------------------------
    
    # retrieve values from control input vector
    vx = ui[0]
    vy = ui[1]

    if i==2:
        rospy.loginfo(f"{t=}") #debug
        rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)


    return vx,vy
# ====================================        


# ====================================        
# Control function for dji rmtt drones to be modified
# should ONLY return (vx,vy,vz,takeoff,land,led) for the robot command
# max useable numbers of drones = 1
# ====================================
def rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
# ====================================
    nbTB3= len(tb3_poses[0]) # number of total tb3 robots in the use
    nbCF = len(cf_poses[0]) # number of total crazyflie nano drones in the use
    nbRMTT = len(rmtt_poses[0]) # number of total dji rmtt drones in the use
    nbRMS1 = len(rms1_poses[0]) # number of total dji rms1 in the use
    nbOBSTACLE = len(obstacle_pose[0]) # number of total obstacle positions in the environment

    ## Convertion to control_algo.py's format
    x = rmtt_poses[0] # get drone position
    y = rmtt_poses[1]
    z = rmtt_poses[2]
    t = ros_time.to_time()
    robots_pos = tb3_poses[0:2,:].T # get positions of all robots

    ## Base state
    vx = 0.0
    vy = 0.0
    vz = 0.0
    yaw = 0.0
    takeoff = False
    land = False
    led = [0,0,0] #RGB
    
    #
    global first_call
    if first_call:
        takeoff = firstDroneCall(robots_pos[0,:], robots_pos[1,:], robots_pos[2,:])
    else:
        vel = otherDroneCalls(robots_pos[0,:], robots_pos[1,:], robots_pos[2,:], [x,y,z])
        [vx, vy, vz] = vel

    # -----------------------

    return vx,vy, vz, yaw, takeoff, land, led
# ====================================    



# ======== ! DO NOT MODIFY ! ============
def tb3_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx,vy = tb3_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)
    return vx,vy

# def cf_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
#     vx, vy, vz, takeoff, land = cf_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)
#     return vx, vy, vz, takeoff, land

def rmtt_controller(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time):
    vx, vy, vz, takeoff, land, led = rmtt_control_fn(robotNo, tb3_poses, cf_poses, rmtt_poses, rms1_poses, obstacle_pose, ros_time)
    return vx, vy, vz, takeoff, land, led
# =======================================