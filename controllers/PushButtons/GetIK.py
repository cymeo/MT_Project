############### Get IK for UR5E ############

### Libraries 
import ikpy.inverse_kinematics
import numpy as np
from scipy.optimize import minimize
import ikpy
from ikpy.chain import Chain
import numpy as np
from math import pi  
from controller import Robot, Supervisor, Connector


supervisor = Supervisor()
robot = supervisor.getFromDef("Robot")
motors = []
motors.append(supervisor.getDevice('shoulder_pan_joint'))
motors.append(supervisor.getDevice('shoulder_lift_joint'))
motors.append(supervisor.getDevice('elbow_joint'))
motors.append(supervisor.getDevice('wrist_1_joint'))
motors.append(supervisor.getDevice('wrist_2_joint'))
motors.append(supervisor.getDevice('wrist_3_joint'))


robot_chain = Chain.from_urdf_file("ur5e.urdf", active_links_mask=[False,False,True,True,True,True,True,True,False,False])

bounds = [
    [0,0],
    [0,0],# baselink inertia
    [-2*pi,2*pi],    # shoulder pan            
    [-2*pi,0],    # shoulder lift Link? // i changed the bounds in the ur5e, might have to change this back!!
    [-2*pi,2*pi],    # elbow         
    [-2*pi,2*pi],    # wrist 1
    [-2*pi,2*pi],    # wrist 2
    [-2*pi,2*pi],    # wrist 3
    [0,0],
    [0,0]# flange tool
]


def getAngles(Pos_target):
    return( robot_chain.inverse_kinematics(Pos_target, [0,0,-1], "Z"))

def getIK(Pos_target, rotation = [0,0,-1], xyz = "Z"): 
    return(robot_chain.inverse_kinematics(Pos_target, rotation, xyz))


def moveMotors(motorangles):
    for n, motor in enumerate(motors):
        motor.setPosition(motorangles[n+2])
    pass  

def motorSensorInfos(timestep):
    sensors = np.zeros(10)
    for n, motor in enumerate(motors):
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        sensors[n+2]= sensor.getValue()
    return sensors 

def getTipPose(Sensorsinfos): 
    return robot_chain.forward_kinematics(Sensorsinfos)



def cost_function(joint_angles, target_position):
    # Calculate the end-effector position with the current joint angles
    full_joint_angles = [0, 0] + list(joint_angles) + [0, 0]
    end_effector_position = robot_chain.forward_kinematics(full_joint_angles)[:3,3]
    # Calculate the position error
    position_error = np.linalg.norm(end_effector_position - target_position)

    # Add penalties for joint limits
    penalty = 0
    for i, angle in enumerate(joint_angles):
        lower, upper = bounds[i]
        if angle < lower:
            penalty += (lower - angle) ** 2  # Penalize if angle is below the lower limit
        elif angle > upper:
            penalty += (angle - upper) ** 2  # Penalize if angle is above the upper limit

    # The total cost is a combination of position error and penalty
    print("postion_error: ", position_error, "penalty: ", penalty )
    
    return 100*position_error + 1000 * penalty  # Adjust the multiplier as needed

def getAnglesOptimised(target_position):
    # Initial guess for joint angles (e.g., all zeros)   
    motorunblocked = True 
    goal_reached = False 
    
    while (not (goal_reached and motorunblocked)): 
        print("inwhile")
        ik_angles = robot_chain.inverse_kinematics(target_position, [0,0,-1], "Z")
        for i, angle in enumerate(ik_angles):
            lower, upper = bounds[i]
            if angle < lower:
                motorunblocked = False # Penalize if angle is below the lower limit
                print("motor blocked")
            elif angle > upper:
                motorunblocked = False # Penalize if angle is above the upper limit  
                print("motor blocked")
                        
        Tip_position = robot_chain.forward_kinematics(ik_angles)[:3,3]
        target_distance = np.linalg.norm(Tip_position - target_position)
        
        if (target_distance <= 0.01): 
            #print("target reached")
            goal_reached = True     
    return ik_angles

    
    
    






