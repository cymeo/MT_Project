"""Arm_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Supervisor 
import random 
import numpy as np
# create the Robot instance.
#robot = Robot()
supervisor = Supervisor()
# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())
arm = supervisor.getFromDef("Arm")
arm_trans = arm.getField("translation")

arm_vel =  np.array([random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1,1)])* timestep/1000 # Random x,y, z movement in m/sec]

print(timestep)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1:
    position = arm_trans.getSFVec3f()
    
    new_position = [
        position[0] + arm_vel[0],  # Move in X
        position[1] + arm_vel[1],
        position[2] + arm_vel[2],   # Move in Z
    ]
    
    new_position[0] = np.clip(new_position[0], 0.25,1.3)
    new_position[1] = np.clip(new_position[1], -0.7,0.7)
    new_position[2] = np.clip(new_position[2], 0.1,0.5)
    #print(new_position)
    arm_trans.setSFVec3f(new_position)
    
    #print(arm_trans)
    if random.random()< 0.1: 
        arm_vel = [0,0,0]
    
    if random.random() > 0.9:
        arm_vel =  np.array([random.uniform(-1, 1), random.uniform(-1, 1), random.uniform(-1,1)])* timestep/1000 # Random x,y, z movement in m/sec]
    pass

# Enter here exit cleanup code.
