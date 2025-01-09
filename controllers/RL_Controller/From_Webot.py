"""controller_test controller."""
from controller import Robot, Supervisor,Connector
from ikpy.chain import Chain
import numpy as np
from scipy.spatial.transform import Rotation as R

###### Controll Robot and Arm Movements ######################################

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()
# Load the chain by specifying the elements you want to include
robot_chain = Chain.from_urdf_file("ur5e.urdf")
# Print the chain to verify the correct links are included
#print(robot_chain)
####define basic time step
timestep = int(supervisor.getBasicTimeStep())

motors = []
motors.append(supervisor.getDevice('shoulder_pan_joint'))
motors.append(supervisor.getDevice('shoulder_lift_joint'))
motors.append(supervisor.getDevice('elbow_joint'))
motors.append(supervisor.getDevice('wrist_1_joint'))
motors.append(supervisor.getDevice('wrist_2_joint'))
motors.append(supervisor.getDevice('wrist_3_joint'))

def get_motor_pos(): 
    sensors= []
    for n, motor in enumerate(motors):
        sensor = motor.getPositionSensor()
        sensor.enable(timestep)
        sensors.append(sensor.getValue())
    return(np.array(sensors))

def get_forward_kinematics(angles): 
    
    angles_trans = np.concatenate((np.array([0]), np.array(angles), np.array([0,0,0])))
    #print(angles_trans)
    robotTipMatrix = robot_chain.forward_kinematics(angles_trans)
    #rotationrot = robotTipMatrix[:3,:3]
    # transpose = robotTipMAtrix[3,:3]
    return(robotTipMatrix)

def check_crash():
    
    crashed = False
    # todo
    return crashed 

def reset_sim():
    supervisor.simulationReset()
    pass

def move_robot(angles):
    for n, motor in enumerate(motors):
        motor.setPosition(angles[n])
    pass





timestep = int(supervisor.getBasicTimeStep())
