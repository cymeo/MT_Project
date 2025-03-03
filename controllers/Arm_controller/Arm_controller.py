from controller import Supervisor
import random
import numpy as np
import time

# Initialize Webots Supervisor
robot = Supervisor()
timestep = int(robot.getBasicTimeStep())
print ("timestep", timestep)
# Get reference to the arm node
arm = robot.getFromDef("Arm")

# Get the arm's translation field
arm_translation = arm.getField("translation")


#startbounds 
xs_min, xs_max = 1, 1.5
ys_min, ys_max = -0.7, 0.7
zs_min, zs_max = 0, 0.5

# goal bounds
x_min, x_max = 0.3, 0.5
y_min, y_max = -0.5, 0.5
z_min, z_max = 0, 0.2

# Speed constraints
speed_min = 0.01  # Minimum movement speed
speed_max = 0.05  # Maximum movement speed

# random stop time
time_min, time_max = 1500,2500  #milisec

current_time = 0 
total_time = random.uniform(time_min,time_max)

start_position= np.array([random.uniform(xs_min, xs_max), random.uniform(ys_min, ys_max), random.uniform(zs_min, zs_max) ])
end_position = np.array([random.uniform(x_min, x_max), random.uniform(y_min, y_max), random.uniform(z_min, z_max) ])  

to_start = -1
to_goal = 1

while robot.step(timestep) != -1:
    
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
        to_goal = to_goal* -1 
        to_start = to_start * -1  
        time.sleep(random.uniform(0.5,3))
        
        current_time = 0 
        total_time = random.uniform(time_min,time_max)

        if to_goal == 1 : 
            start_position = end_position
            end_position = np.array([random.uniform(x_min, x_max), random.uniform(y_min, y_max), random.uniform(z_min, z_max) ])       
        
        if to_start == 1 :
            start_position = end_position
            end_position = np.array([random.uniform(xs_min, xs_max), random.uniform(ys_min, ys_max), random.uniform(zs_min, zs_max) ])
        print("start",start_position)
        print("end", end_position)
    
        

