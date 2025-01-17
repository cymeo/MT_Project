"""controller_test controller."""
from controller import Robot, Supervisor,Connector
from ikpy.chain import Chain
import numpy as np
from scipy.spatial.transform import Rotation as R
import time

###### Controll Robot and Arm Movements ######################################

#-------------------------------------------------------------------
# define a Supervisor
supervisor = Supervisor()
# Load the chain by specifying the elements you want to include
robot_chain = Chain.from_urdf_file("ur5e.urdf")
# Print the chain to verify the correct links are included
#print(robot_chain)

#define basic time step
timestep = int(supervisor.getBasicTimeStep())

#get Robot 
robot_node = supervisor.getFromDef('Robot') 
box = supervisor.getFromDef('Box')
arm = supervisor.getFromDef('Arm')
table = supervisor.getFromDef('Table')


def get_dist(): 
    pass       
        
#get motors
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
    # Check for contact points
    contact_robot = robot_node.getContactPoints(includeDescendants=True)
    contact_table = table.getContactPoints(includeDescendants = True)
    crashed = False
    if contact_table: 
        #print("table crash")
        crashed = True
    if contact_robot: 
        #print("robot crash")
        crashed = True
    return crashed
    

def reset_sim():
    supervisor.simulationReset()
    pass

def move_robot(angles):
    crashed = False
    sensors= np.zeros(6)        
    start_time = time.time()
    
    while True: 
        supervisor.step(timestep)
        current_pos = get_motor_pos()
        crashed = check_crash()
        if check_crash() == True: 
            break
        
        if all (abs(current_pos-angles) <= 0.01): 
            #print("moved")
            break
        
        for n, motor in enumerate(motors):
            motor.setPosition(angles[n])
            sensor = motor.getPositionSensor()
            sensor.enable(timestep)
            sensors[n] = sensor.getValue()
        
        
        # Check for timeout
        if time.time() - start_time > 5:
            print("Warning: Motor movement timeout!")
            break
        time.sleep(0.01)
    
    return sensors, crashed


def show_goal(position):
    #print('pos =' , position.tolist())
    translation = box.getField("translation")
    #position = [position[0],position[1], position[2] ]
    #print('transl before:', translation.getSFVec3f() )
    translation.setSFVec3f(position.tolist())
    pass
    


