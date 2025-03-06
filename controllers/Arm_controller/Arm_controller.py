from controller import Supervisor
import random
import numpy as np
import time

# Initialize Webots Supervisor
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
# Get reference to the arm node
arm = robot.getFromDef("Arm")

# Get the arm's translation field
arm_translation = arm.getField("translation")


#startbounds 
xs_min, xs_max = 0.5, 0.6 
ys_min, ys_max = -0.4, 0.4
zs_min, zs_max = 0, 0.3

# goal bounds
x_min, x_max = 0.2, 0.5
y_min, y_max = -0.3, 0.3
z_min, z_max = 0.1, 0.2

# Speed constraints
speed_min = 0.01  # Minimum movement speed
speed_max = 0.05  # Maximum movement speed

# random stop time
time_min, time_max = 1500,2500  #milisec

current_time = 0 
total_time = random.uniform(time_min,time_max)
pausetime = random.uniform(1500,3000)

start_position= np.array([random.uniform(xs_min, xs_max), random.uniform(ys_min, ys_max), random.uniform(zs_min, zs_max) ])
end_position = np.array([random.uniform(x_min, x_max), random.uniform(y_min, y_max), random.uniform(z_min, z_max) ])  

to_start = -1
to_goal = 1

arm_pos_prev = np.array(arm_translation.getSFVec3f())
arm_pos_cur = np.array(arm_translation.getSFVec3f())
while robot.step(timestep) != -1:
    arm_pos_cur = np.array(arm_translation.getSFVec3f())
    v_arm = np.linalg.norm(arm_pos_cur-arm_pos_prev)/timestep*1000
    arm_pos_prev = arm_pos_cur
    #print("Varm ", v_arm)

    current_time += timestep    
    t = current_time/ total_time
    t = min(max(t, 0), 1)  # Ensure t stays between 0 and 1
    
    # jerk equation 
    smooth_t = 10 * t**3 - 15 * t**4 + 6 * t**5
    
    # Compute new position
    new_position = [
        start_position[i] + (end_position[i] - start_position[i]) * smooth_t
        for i in range(3)
    ]

    arm_translation.setSFVec3f(new_position)
    
    # reach goal_position, wait for max 3 sec and go to new start/endpos 
    if (np.linalg.norm(new_position-end_position)<= 0.01): 
        
        if current_time > (pausetime + total_time):

            to_goal = to_goal* -1 
            to_start = to_start * -1  

            # set timer for position calculation
            current_time = 0 
            total_time = random.uniform(time_min,time_max)
            pausetime = random.uniform(1500,3000)
            
            if to_goal == 1 : 
                start_position = end_position
                end_position = np.array([random.uniform(x_min, x_max), random.uniform(y_min, y_max), random.uniform(z_min, z_max) ])       
            
            if to_start == 1 :
                start_position = end_position
                end_position = np.array([random.uniform(xs_min, xs_max), random.uniform(ys_min, ys_max), random.uniform(zs_min, zs_max) ])
        
        else: 
            continue

    
                
        

